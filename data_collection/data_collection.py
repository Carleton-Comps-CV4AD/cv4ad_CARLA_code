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
import argparse
import json

from ego_vehicle import Ego_Vehicle, Camera
from world import World
import bounding_boxes as bb
from bounding_boxes import get_image_point, configure_matrices
from utilities import quantize_to_tick, check_next_weather, check_dead, check_has_image

# TODO:
# - Fix data organization re: ego vehicle class and weather class
# - improve parameterization of vehicle, ego vehicle, and walker spawning
#     - that is, we should be able to specify num cars / motorcycles, lane change percent, speeds, etc
# - When spawning fails, we should handle that more elegantly than just reporting it and then moving on
# ! - We need to verify that car crashes / traffic jams no longer occur
# ! - Decide on a final resolution to produce our image sets at
# ! - set the ego vehicle model manually --> some of the models are not rigged with lights, and also the transform for the cameras is hard coded right now
# - Write something to qualify the difficulty of our images using the bounding boxes
#        - That is, define some function of overlappingness and stuff
# - We should not draw bounding boxes of hidden vehicles
# * - Add arg parsing so that we can specify the following from the command line:
#     - map
# - BUG: checking for dead pedestrians seems to happen twice in a row whenever it triggers
# * - Will increasing the shutter speed decrease the blurryness of our images? --> it does not appear so
#     - We already turned off motion blur and changed anti-aliasing mode to 1 in the engine configuration
#     - The latter of these seems to have helped a little, but there is still substatial blurring

# - BUG: when frame rate is really high, the print statments for the counters for the camera but the images are synched so its weird
# ! - We need to save the arguments to the output folder

# python3 data_collection.py --seconds_per_tick 0.166667 --random_seed 123456 --video_mode --video_images_saved 18 --videos_wanted 500 --video_images_wait 30


def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Data Collection')
    parser.add_argument('--num_images_per_weather', type=int, default=1250, help='Number of images to take per weather scenario')
    parser.add_argument('--weather_config', type=str, default='clear_day.yaml', help='Name of the weather configuration folder')
    parser.add_argument('--seconds_per_tick', type=float, default=3, help='Number of seconds between each photo taken')
    parser.add_argument('--debug', action='store_true', help='Run in debug mode')
    parser.add_argument('--car_count', type=int, default=40, help='Number of cars to spawn')
    parser.add_argument('--walker_count', type=int, default=40, help='Number of walkers to spawn')
    parser.add_argument('--output_dir', type=str, default=os.path.join('/Data', time.strftime("%m%d-%H%M")), help='Output directory for images')
    parser.add_argument('--draw_bounding_box', action='store_true', help='Draw bounding boxes on images')
    parser.add_argument('--random_seed', type=int, help='Use deterministic mode with this seed', default=None)
    parser.add_argument('--video_mode', action='store_true', help='Turn on video mode')
    parser.add_argument('--video_images_saved', type=int, default=None, help='Number of images to save in video mode')
    parser.add_argument('--videos_wanted', type=int, default=None, help='Number of images to wait for in video mode')
    parser.add_argument('--video_images_wait', type=int, default=None, help='Number of images to wait for in video mode')


    args = parser.parse_args()
    seconds_per_tick = args.seconds_per_tick
    num_images_per_weather = args.num_images_per_weather
    weather_config = args.weather_config
    car_count = args.car_count
    walker_count = args.walker_count
    out_dir = args.output_dir
    draw_bounding_box = args.draw_bounding_box
    debug = args.debug
    random_seed = args.random_seed
    video_mode = args.video_mode
    video_images_saved = args.video_images_saved
    video_images_wait = args.video_images_wait
    videos_wanted = args.videos_wanted



    if video_mode:
        if not video_images_saved or not videos_wanted:
            # saved < wait
            raise Exception("In video mode, but no images will be saved for. \n Please enter both --video_images_saved and --videos_wanted flags")
        num_images_per_weather = videos_wanted * (video_images_saved + video_images_wait)

        print(f"Setting num images per weather to {num_images_per_weather} to capture {videos_wanted} videos with {video_images_saved} images each, waiting {video_images_wait} images between videos")

    if random_seed:
        out_dir = out_dir + f"_seed:{random_seed}"

    # save the arguments and configurations to a text file in the same output diectory
    path_to_args = os.path.join(out_dir, 'arguments.json')
    os.makedirs(out_dir, exist_ok=True)

    with open(path_to_args, 'w') as arg_file:
        json.dump(vars(args), arg_file, indent=4)

    
    try:
        sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
            sys.version_info.major,
            sys.version_info.minor,
            'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
    except IndexError:
        pass

    try:
        sensor_queue = Queue()
        our_world = World('localhost', 2000, random_seed = random_seed) 
        
        # Read our weather configurations from yaml and then set the first configuration to be the current weather
        our_world.load_weathers(weather_config)
        our_world.update_weather()
        print(f"Setting initial weather using {weather_config}")
        time.sleep(1)

        # Quantize the seconds per tick to the nearest multiple of the world delta seconds
        seconds_per_tick = quantize_to_tick(seconds_per_tick, our_world.world.get_settings().fixed_delta_seconds)

        ego = Ego_Vehicle(our_world.world)

        # I don't like how the ego vehicle and camera classes are organized. It's really unclear who is actually responsible for
        # configuring and spawning the camera actors. It feels like the camera class should be responsible for this, but it's not.
        # And we also configure a bunch of stuff in main and its all kind of terrible :/
        rgb_cam = Camera(our_world.world, sensor_queue, 'sensor.camera.rgb', carla.Transform(carla.Location(x=1.5, z=2.4)), 
                         name = 'rgb', file_type = 'png', cc = carla.ColorConverter.Raw, out_dir = out_dir,
                         seconds_per_tick = seconds_per_tick, video_mode_state = video_mode, video_wait = video_images_wait, video_images_saved=video_images_saved)
        rgb_seg = Camera(our_world.world, sensor_queue, 'sensor.camera.semantic_segmentation', carla.Transform(carla.Location(x=1.5, z=2.4)),
                         name = 'rgb_seg', file_type = 'png', out_dir = out_dir,
                         seconds_per_tick = seconds_per_tick, video_mode_state = video_mode, video_wait = video_images_wait, video_images_saved=video_images_saved)

        rgb_cam.set_image_size()
        rgb_seg.set_image_size()
        rgb_cam.set_shutter_speed(250) # Really not clear if this does anything for us, but it doesn't hurt. We were trying to reduce motion blur

        ego.add_camera(rgb_cam)
        ego.add_camera(rgb_seg)

        if not video_mode:
            lidar_cam = Camera(our_world.world, sensor_queue, 'sensor.lidar.ray_cast', carla.Transform(carla.Location(x=1.5, z=2.4)),
                            name = 'lidar', file_type = 'ply', out_dir = out_dir,
                            seconds_per_tick = seconds_per_tick, video_mode_state = video_mode, video_wait = video_images_wait, video_images_saved=video_images_saved)
            # lidar_seg = Camera(our_world.world, sensor_queue, 'sensor.lidar.ray_cast_semantic', carla.Transform(carla.Location(x=1.5, z=2.4)),
            #                 name = 'lidar_seg', file_type = 'ply', out_dir = out_dir,
            #                 seconds_per_tick = seconds_per_tick, video_mode_state = video_mode, video_wait = video_images_wait, video_images_saved=video_images_saved)
            lidar_cam.set_lidar_settings(our_world.world.get_settings().fixed_delta_seconds)
            ego.add_camera(lidar_cam)
            # ego.add_camera(lidar_seg)
        else:
            instance_seg = Camera(our_world.world, sensor_queue, 'sensor.camera.instance_segmentation', carla.Transform(carla.Location(x=1.5, z=2.4)),
                            name = 'instance_seg', file_type = 'png', out_dir = out_dir,
                            seconds_per_tick = seconds_per_tick, video_mode_state = video_mode, video_wait = video_images_wait, video_images_saved=video_images_saved)
            instance_seg.set_image_size()
            ego.add_camera(instance_seg)

        # The way we have this organized is the car knows how many images to take per weather, and how many weathers there are.
        # It then passes this information to the camera objects, which are responsible for saving the images to different folders
        # based on which weather is currently active. This is a little convoluted, but it works.
        ego.configure_experiment(num_images_per_weather, [state['name'] for state in our_world.weather.states])

        # Spawn cars and walkers
        spawned = our_world.spawn_car(number = car_count)
        print(f"spawned {spawned}/{car_count} attempted cars")
        spawned = our_world.spawn_walker(number = walker_count)
        print(f"spawned {spawned}/{walker_count} attempted walkers")
        
        # If we are drawing bounding boxes and visualizing them as we go, we need to create a window to display the images
        if draw_bounding_box and debug:
            cv2.namedWindow('ImageWindowName', cv2.WINDOW_AUTOSIZE)

        last_photo_count = -1
        check_for_dead = True

        while True:
            our_world.world.tick()  
            # Try and progress the weather to the next state if we have taken enough images for the current weather
            last_photo_count = check_next_weather(ego, our_world, num_images_per_weather, last_photo_count)
            if last_photo_count < -1:
                print("at last photo count")
                break

            # Check for dead people every 15 frames. Delete their actors and spawn new ones
            check_for_dead = check_dead(cur_image_num = ego.cameras[0].counter, check_for_dead = check_for_dead, world = our_world, interval = 15)

            # Check if we have a new image and if so, process it
            check_has_image(ego, sensor_queue, our_world, debug, draw_bounding_box, out_dir)
            
            check_for_dead = True       

    finally:
        # These cameras are our camera objects so they need to destroy themselves
        for camera in ego.cameras:
            camera.destroy()

        our_world.clean_up()


if __name__ == '__main__':
    main()
