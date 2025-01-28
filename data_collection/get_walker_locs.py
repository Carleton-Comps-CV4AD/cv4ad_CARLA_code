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

from ego_vehicle import Ego_Vehicle, Camera
from world import World
import bounding_boxes as bb
from bounding_boxes import get_image_point, configure_matrices
from utilities import quantize_to_tick, check_next_weather, check_dead, check_has_image

def main():
    
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

        locs = []
        for i in range(10000):
            loc = our_world.world.get_random_location_from_navigation() 
            if loc is not None and loc not in locs:
                locs.append(loc)
        
        with open('locations.txt', 'w') as f:
            for loc in locs:
                f.write(f'{loc}\n')
  
    except Exception as e:
        print(e)
        print('Error initializing world')
        return


if __name__ == '__main__':
    main()
