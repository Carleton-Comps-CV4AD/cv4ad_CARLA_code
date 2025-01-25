#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Modified by cv4ad

import glob
import os
import sys
import carla
import time
import cv2
import numpy as np
from queue import Queue, Empty

from ego_vehicle import Ego_Vehicle, Camera
from utilities import World
import bounding_boxes as bb
from bounding_boxes import get_image_point, configure_matrices

DEBUG = False

# TODO:
# - Fix data organization re: ego vehicle class and weather class
# - improve parameterization of vehicle, ego vehicle, and walker spawning
#     - that is, we should be able to specify num cars / motorcycles, lane change percent, speeds, etc
# - When spawning fails, we should handle that more elegantly than just reporting it and then moving on
# ! - We need to verify that car crashes / traffic jams no longer occur
# ! - Decide on a final resolution to produce our image sets at
# ! - set the ego vehicle model manually --> some of the models are not rigged with lights, and also the transform for the cameras is hard coded right now
# - Implement deterministic mode so that we can recreate datasets
# * - Add arg parsing so that we can specify the following from the command line:
#     - num images per weather
#     - num cars, pedestrians
#     - name of weather config folder
#     - map
#     - photos per second / seconds per photo
# - BUG: checking for dead pedestrians seems to happen twice in a row whenever it triggers
# * - Will increasing the shutter speed decrease the blurryness of our images? --> it does not appear so
#     - We already turned off motion blur and changed anti-aliasing mode to 1 in the engine configuration
#     - The latter of these seems to have helped a little, but there is still substatial blurring

SECONDS_PER_TICK = 3

def main():
    # * Configure how many images we want per weather scenario from weathers.yaml. Should probably be around 1200/n, where n is the number of cities. Leave at 2 for testing.
    num_images_per_weather = 20
    
    try:
        sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
            sys.version_info.major,
            sys.version_info.minor,
            'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
    except IndexError:
        pass

    try:
        sensor_queue = Queue()
        our_world = World('localhost', 2000)
        
        # Read our weather configurations from yaml and then set the first configuration to be the current weather
        our_world.load_weathers('six_weathers.yaml')
        our_world.update_weather()
        print("Setting initial weather")
        time.sleep(1)

        ego = Ego_Vehicle(our_world.world)

        out_dir = "/Data/tracking_videos"

        world_delta_seconds = our_world.world.get_settings().fixed_delta_seconds
        ticks_per_image = SECONDS_PER_TICK // world_delta_seconds
        assert ticks_per_image > 1
        seconds_per_tick = world_delta_seconds * ticks_per_image

        rgb_cam = Camera(our_world.world, sensor_queue, 'sensor.camera.rgb', carla.Transform(carla.Location(x=1.5, z=2.4)), 
                         name = 'rgb', file_type = 'png', cc = carla.ColorConverter.Raw, out_dir = out_dir,
                         seconds_per_tick = seconds_per_tick)
        rgb_seg = Camera(our_world.world, sensor_queue, 'sensor.camera.semantic_segmentation', carla.Transform(carla.Location(x=1.5, z=2.4)),
                         name = 'rgb_seg', file_type = 'png', out_dir = out_dir,
                         seconds_per_tick = seconds_per_tick)
        lidar_cam = Camera(our_world.world, sensor_queue, 'sensor.lidar.ray_cast', carla.Transform(carla.Location(x=1.5, z=2.4)),
                           name = 'lidar', file_type = 'ply', out_dir = out_dir,
                           seconds_per_tick = seconds_per_tick)
        lidar_seg = Camera(our_world.world, sensor_queue, 'sensor.lidar.ray_cast_semantic', carla.Transform(carla.Location(x=1.5, z=2.4)),
                           name = 'lidar_seg', file_type = 'ply', out_dir = out_dir,
                           seconds_per_tick = seconds_per_tick)
        instance_seg = Camera(our_world.world, sensor_queue, 'sensor.camera.instance_segmentation', carla.Transform(carla.Location(x=1.5, z=2.4)),
                           name = 'instance_seg', file_type = 'png', out_dir = out_dir,
                           seconds_per_tick = seconds_per_tick)
        
        rgb_cam.set_image_size()
        instance_seg.set_image_size()
        rgb_seg.set_image_size()
        rgb_cam.set_shutter_speed(250)

        ego.add_camera(rgb_cam)
        ego.add_camera(rgb_seg)
        ego.add_camera(lidar_cam)
        ego.add_camera(lidar_seg)
        ego.add_camera(instance_seg)
        ego.configure_experiment(num_images_per_weather, [state['name'] for state in our_world.weather.states])

        car_count = 40
        walker_count = 0

        for i in range(1):
            spawned = our_world.spawn_car(number = car_count)
            print(f"spawned {spawned}/{car_count} attempted cars")
        
        for i in range(1):
            spawned = our_world.spawn_walker(number = walker_count)
            print(f"spawned {spawned}/{walker_count} attempted walkers")
        
        cv2.namedWindow('ImageWindowName', cv2.WINDOW_AUTOSIZE)
        last_photo_count = -1
        check_for_dead = True
        while True:
            our_world.world.tick()  
            # Try and progress the weather to the next state if we have taken enough images for the current weather
            if ego.cameras[0].counter != last_photo_count and ego.cameras[0].counter % num_images_per_weather == 0:
                last_photo_count = ego.cameras[0].counter
                # for camera in ego.cameras:
                #     camera.counter = 0

                result = our_world.weather.next()
                if result < 0:
                    break
                time.sleep(1)

                if our_world.weather._sun.altitude < 15:
                    ego.lights_on()
                else:
                    ego.lights_off()
                our_world.update_weather()

            # Check for dead people every 15 frames. Delete their actors and spawn new ones
            if ego.cameras[0].counter % 5 == 0 and check_for_dead == True:
                our_world.replace_dead_walkers()
                check_for_dead = False

            if ego.cameras[0].has_new_image:
                try:
                    for i in range(len(ego.cameras)):
                        img, sensor_name = sensor_queue.get(True, 2.0)
                        # Only try to draw bb on separate screen if the cur image is the rgb image
                        if sensor_name == 'sensor.camera.rgb' and ego.cameras[0].has_new_image:
                            assert ego.cameras[0].blueprint == 'sensor.camera.rgb'
                            camera = ego.cameras[0] # ! This should be the rgb camera
                            bb_img = bb.get_bb_img(our_world.world, ego.vehicle, img, camera)
                            cv2.imwrite(os.path.join("test_bb", f"bb_img_{img.frame}.png"), bb_img)
                            if DEBUG or True:
                                cv2.imshow('ImageWindowName',bb_img)

                                if cv2.waitKey(10000) == ord('q'):
                                    break
                                cv2.destroyAllWindows()
                except Empty:
                    print("    Some of the sensor information is missed")
                ego.cameras[0].has_new_image = False
                check_for_dead = True       

    finally:
        # These cameras are our camera objects so they need to destroy themselves
        for camera in ego.cameras:
            camera.destroy()

        our_world.clean_up()


if __name__ == '__main__':
    main()
