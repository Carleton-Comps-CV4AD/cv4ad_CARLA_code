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
import carla # type: ignore <-- what is this
import random
import math
import time
import yaml
from queue import Queue
from queue import Empty

from ego_vehicle import Ego_Vehicle, Camera
from utilities import World

DEBUG = False

def main():
    # * Configure how many images we want per weather scenario from weathers.yaml. Should probably be around 1200/n, where n is the number of cities. Leave at 2 for testing.
    num_images_per_weather = 30
    
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

        ego = Ego_Vehicle(our_world.world)

        rgb_cam = Camera(our_world.world, sensor_queue, 'sensor.camera.rgb', carla.Transform(carla.Location(x=1.5, z=2.4)), 
                         out_dir = '_outRaw', file_type = 'png', cc = carla.ColorConverter.Raw)
        rgb_seg = Camera(our_world.world, sensor_queue, 'sensor.camera.semantic_segmentation', carla.Transform(carla.Location(x=1.5, z=2.4)),
                         out_dir = '_outSeg', file_type = 'png')
        lidar_cam = Camera(our_world.world, sensor_queue, 'sensor.lidar.ray_cast', carla.Transform(carla.Location(x=1.5, z=2.4)),
                           out_dir = '_outLIDAR', file_type = 'ply')
        lidar_seg = Camera(our_world.world, sensor_queue, 'sensor.lidar.ray_cast_semantic', carla.Transform(carla.Location(x=1.5, z=2.4)),
                           out_dir = '_outLIDARseg', file_type = 'ply')
        
        rgb_cam.set_image_size()
        rgb_seg.set_image_size()

        ego.add_camera(rgb_cam)
        ego.add_camera(rgb_seg)
        ego.add_camera(lidar_cam)
        ego.add_camera(lidar_seg)
        ego.configure_experiment(num_images_per_weather, [state['name'] for state in our_world.weather.states])


        car_count = 40
        walker_count = 60

        for i in range(1):
            spawned = our_world.spawn_car(number = car_count)
            print(f"spawned {spawned}/{car_count} attempted cars")
        
        for i in range(1):
            spawned = our_world.spawn_walker(number = walker_count)
            print(f"spawned {spawned}/{walker_count} attempted walkers")

        last_photo_count = -1
        check_for_dead = True
        while True:

            # Try and progress the weather to the next state if we have taken enough images for the current weather
            if ego.cameras[0].counter != last_photo_count and ego.cameras[0].counter % num_images_per_weather == 0:
                last_photo_count = ego.cameras[0].counter
                # for camera in ego.cameras:
                #     camera.counter = 0
                try:
                    our_world.weather.next()
                    time.sleep(1)
                    
                except StopIteration:
                    break

                if our_world.weather._sun.altitude < 15:
                    ego.lights_on()
                else:
                    ego.lights_off()
                our_world.update_weather()

            our_world.world.tick()

            # Check for dead people every 15 frames. Delete their actors and spawn new ones
            if ego.cameras[0].counter % 5 == 0 and check_for_dead == True:
                our_world.replace_dead_walkers()
                check_for_dead = False

            if ego.cameras[0].has_new_image:
                try:
                    for _ in range(len(ego.cameras)):
                        s_frame = sensor_queue.get(True, 1.0)
                        if DEBUG:
                            print("    Frame: %d   Sensor: %s" % (s_frame[0], s_frame[1]))
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
