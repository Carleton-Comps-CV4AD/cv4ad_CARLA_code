from queue import Queue, Empty
from world import World
from ego_vehicle import Ego_Vehicle
import time
import os
import cv2
import bounding_boxes as bb
from lidar_projection import project
import numpy as np

def quantize_to_tick(seconds_per_tick: int, world_delta_seconds: int) -> int:
    ticks_per_image = seconds_per_tick // world_delta_seconds
    assert ticks_per_image > 1 # Do not set the seconds per tick to be less than the world delta seconds. I don't know what will happen, but it doesn't seem like a good idea
    seconds_per_tick = world_delta_seconds * ticks_per_image
    return seconds_per_tick

# Check if we need to progress the weather to the next state and if so, do it
def check_next_weather(ego: Ego_Vehicle, world: World, num_images_per_weather: int, last_photo_count: int) -> int:
    if ego.cameras[0].counter != last_photo_count and ego.cameras[0].counter > 0 and ego.cameras[0].counter % num_images_per_weather == 0:

        # ! logic used to identify 
        # print("last phooto count: ", last_photo_count)


        # print("\n\n this is the camera counter followed by if they equal: ", ego.cameras[0].counter)
        # print(ego.cameras[0].counter != last_photo_count)

        # print("this is the number of images we want per weather: ", num_images_per_weather)

        # print(ego.cameras[0].counter % num_images_per_weather == 0)
        last_photo_count = ego.cameras[0].counter

        result = world.weather.next()
        if result < 0:
            return -5
        time.sleep(1)

        if world.weather._sun.altitude < 15:
            ego.lights_on()
        else:
            ego.lights_off()
        world.update_weather()
    return last_photo_count

# Check if we need to replace dead walkers and if so, do it
def check_dead(cur_image_num: int, check_for_dead: bool, world: World, interval: int) -> bool:
    if cur_image_num % interval == 0 and check_for_dead:
        world.replace_dead_walkers()
        return False
    return check_for_dead

def check_has_image(ego: Ego_Vehicle, sensor_queue: Queue, world: World, debug: bool, draw_bounding_box: bool, out_dir: str) -> None:
    if ego.cameras[-1].has_new_image:
        try:
            image = None
            image_camera = None
            lidar = None
            lidar_camera = None

            for i in range(len(ego.cameras)):
                img, sensor_name = sensor_queue.get(True, 2.0)
                if sensor_name == 'sensor.camera.rgb':
                    image = img
                    image_camera = ego.cameras[i]
                elif sensor_name == 'sensor.lidar.ray_cast':
                    lidar = img
                    lidar_camera = ego.cameras[i]
                
                if draw_bounding_box:
                    # Only try to draw bb on separate screen if the cur image is the rgb image
                    if sensor_name == 'sensor.camera.rgb' and ego.cameras[0].has_new_image:
                        assert ego.cameras[0].blueprint == 'sensor.camera.rgb'
                        camera = ego.cameras[0] # ! This should be the rgb camera, but is not guaranteed to be
                        bb_img = bb.get_bb_img(world.world, ego.vehicle, img, camera)
                        cv2.imwrite(os.path.join("test_bb", f"bb_img_{img.frame}.png"), bb_img)
                        if debug:
                            cv2.imshow('ImageWindowName',bb_img)

                            if cv2.waitKey(10000) == ord('q'):
                                break
                            cv2.destroyAllWindows()
            if image and lidar:
                lidar_2d = project(image_data = image, lidar_data = lidar,
                    camera = image_camera.camera, camera_bp = image_camera.camera_blueprint, 
                    lidar = lidar_camera.camera, 
                    camera_transform = image_camera.transform_at_last_image,
                    lidar_transform = lidar_camera.transform_at_last_image)

                lidar_out_dir = os.path.join(out_dir, image_camera.weathers[image_camera.counter // image_camera.num_images_per_weather], 'lidar_2d')
                if not os.path.exists(lidar_out_dir):
                    os.makedirs(lidar_out_dir)
                np.save(os.path.join(lidar_out_dir, f"{image_camera.counter}.npy"), lidar_2d)

            else:
                print("    Some of the sensor information is missed")
        except Empty:
            print("    Some of the sensor information is missed")
        ego.cameras[-1].has_new_image = False
        # time.sleep(0.7)