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

# Global configuration
SECONDS_PER_TICK = 5.0 # seconds per tick

#WEATHER STUFF

class Sun(object):
    def __init__(self, azimuth, altitude):
        self.azimuth = azimuth
        self.altitude = altitude
        self._t = 0.0

    def set_azimuth(self, azimuth):
        self.azimuth = azimuth
    
    def set_altitude(self, altitude):
        self.altitude = altitude

    def __str__(self):
        return 'Sun(alt: %.2f, azm: %.2f)' % (self.altitude, self.azimuth)

class Weather(object):
    def __init__(self, weather, configs):
        self.weather = weather
        self._sun = Sun(weather.sun_azimuth_angle, weather.sun_altitude_angle)
        with open(configs, 'r') as file:
            self.states = yaml.safe_load(file)['states']
            self.states_iter = iter(self.states)

    def next(self):
        state = self.states_iter.__next__()
        print(f"setting weather to", state['name'])

        # self._sun.set_azimuth(state.azimuth) see the comment on alititude in the yaml
        self._sun.set_altitude(state['altitude'])

        self.weather.cloudiness = state['cloudiness']
        self.weather.precipitation = state['precipitation']
        self.weather.precipitation_deposits = state['precipitation_deposits']
        self.weather.wind_intensity = state['wind_intensity']
        self.weather.fog_density = state['fog_density']
        self.weather.wetness = state['wetness']
        
        self.weather.sun_azimuth_angle = self._sun.azimuth
        self.weather.sun_altitude_angle = self._sun.altitude

    def __str__(self):
        return '%s %s' % (self._sun)
    
class Ego_Vehicle():
    def __init__(self, world, blueprint, spawn_points, spawn_point = None):
        self.bp = blueprint
        self.spawn_points = spawn_points
        if spawn_point is None:
            spawn_point = random.choice(spawn_points)
        else:
            spawn_point = spawn_points[spawn_point]
        
        self.cameras = []
        self.world = world

        # So let's tell the world to spawn the vehicle.
        self.vehicle = world.spawn_actor(blueprint, spawn_point)
        self.vehicle.set_autopilot(True)
        print('created %s' % self.vehicle.type_id)

    def add_camera(self, camera):
        camera_actor = self.world.spawn_actor(camera.camera, camera.transform, attach_to=self.vehicle)
        camera.camera_actor = camera_actor
        camera_actor.listen(lambda image: camera.listen(image))
        self.cameras.append(camera)
        print('created %s' % camera.blueprint)

    def configure_experiment(self, num_images_per_weather, weathers):
        for camera in self.cameras:
            camera.configure_experiment(num_images_per_weather, weathers)

    def lights_on(self):
        print("turning on lights")
        current_lights = carla.VehicleLightState.NONE
        current_lights |= carla.VehicleLightState.LowBeam
        # current_lights |= carla.VehicleLightState.HighBeam
        current_lights |= carla.VehicleLightState.Position
        self.vehicle.set_light_state(carla.VehicleLightState(current_lights))
        self.vehicle.set_autopilot(True)

    def lights_off(self):
        print("turning off lights")
        current_lights = carla.VehicleLightState.NONE
        self.vehicle.set_light_state(carla.VehicleLightState(current_lights))
        self.vehicle.set_autopilot(True)

class Camera():
    def __init__(self, world, sensor_queue, blueprint, transform, out_dir, file_type, cc = None):
        self.blueprint = blueprint
        self.transform = transform
        self.counter = 0
        self.sensor_queue = sensor_queue

        blueprint_library = world.get_blueprint_library()
        self.camera = blueprint_library.find(blueprint)
        self.transform = carla.Transform(carla.Location(x=1.5, z=2.4))
            
        self.camera.set_attribute('sensor_tick', str(SECONDS_PER_TICK))

        self.out_dir = out_dir
        self.file_type = file_type
        self.cc = cc

        self.has_new_image = False

    def configure_experiment(self, num_images_per_weather, weathers):
        self.num_images_per_weather = num_images_per_weather
        self.weathers = weathers
    
    def listen(self, image):
        weather_name = self.weathers[self.counter // self.num_images_per_weather]
        image_path = os.path.join(weather_name, self.out_dir, f'{self.counter}.{self.file_type}')
        if self.cc:
            image.save_to_disk(image_path, self.cc)
        else:
            image.save_to_disk(image_path)
        self.increment()
        self.sensor_queue.put((image.frame, self.blueprint))

        self.has_new_image = True
    
    def increment(self):
        self.counter += 1

    def destroy(self):
        self.camera_actor.destroy()

# ! Temporary function to fill the scene with vehicles and pedestrians. We should paratmetrize and organize this so that we can configure the scene easily
def initialize_v(world, client, actor_list, spawn_points):
    traffic_manager = client.get_trafficmanager()
    traffic_manager.set_synchronous_mode(True)
    # Select some models from the blueprint library
    models = ['dodge', 'audi', 'model3', 'mini', 'mustang', 'lincoln', 'prius', 'nissan', 'crown', 'impala']
    blueprints = []
    for v in world.get_blueprint_library().filter('*vehicle*'):
        if any(model in v.id for model in models):
            blueprints.append(v)

    # Set a max number of vehicles and prepare a list for those we spawn
    max_vehicles = 30
    max_vehicles = min([max_vehicles, len(spawn_points)])
    vehicles = []
    
    for i, spawn_point in enumerate(random.sample(spawn_points, max_vehicles)):
        temp = world.try_spawn_actor(random.choice(blueprints), spawn_point)
        if temp is not None:
            vehicles.append(temp)

    for v in vehicles:
        v.set_autopilot(True) 
        traffic_manager.random_left_lanechange_percentage(v, .1)
        traffic_manager.random_right_lanechange_percentage(v, .1)
        traffic_manager.auto_lane_change(v, True)  
    
    return vehicles

def initialize_w(world, client, actor_listw, spawn_points):
    # Spawn walkers
    blueprint_library = world.get_blueprint_library()
    walker_bp = blueprint_library.filter('walker.pedestrian.*')
    walker_controller_bp = blueprint_library.find('controller.ai.walker')

    walkers = []
    controllers = []

    spawn_points = []
    for i in range(10):
        spawn_point = carla.Transform()
        spawn_point.location = world.get_random_location_from_navigation()
        if spawn_point.location is not None:
            spawn_points.append(spawn_point)

    sbatch = []
    for spawn_point in spawn_points:
        walker_bp_choice = random.choice(walker_bp)
        if walker_bp_choice.has_attribute('is_invincible'):
            walker_bp_choice.set_attribute('is_invincible', 'false')

        sbatch.append(carla.command.SpawnActor(walker_bp_choice, spawn_point))

    results = client.apply_batch_sync(sbatch, True)

    for result in results:
        if not result.error:
            walkers.append(result.actor_id)

    batch = []
    for walker_id in walkers:
        batch.append(carla.command.SpawnActor(walker_controller_bp, carla.Transform(), walker_id))

    results = client.apply_batch_sync(batch, True)

    for result in results:
        if not result.error:
            controllers.append(result.actor_id)

    walker_actors = [world.get_actor(w) for w in walkers]
    controller_actors = [world.get_actor(c) for c in controllers]

    for controller in controller_actors:
        controller.start()
        controller.go_to_location(world.get_random_location_from_navigation())
        controller.set_max_speed(random.uniform(1.0, 3.0))  # Random speeds



    print(f"Spawned {len(walkers)} walkers.")

    actor_listw.extend(walker_actors)
    actor_listw.extend(controller_actors)

    return walkers

def initialize_a_w(world, client, actor_listw, spawn_points):
    # Spawn walkers
    blueprint_library = world.get_blueprint_library()
    walker_bp = blueprint_library.filter('walker.pedestrian.*')
    walker_controller_bp = blueprint_library.find('controller.ai.walker')

    walkers = []
    controllers = []

    spawn_points = []
    
    spawn_point = carla.Transform()
    spawn_point.location = world.get_random_location_from_navigation()
    if spawn_point.location is not None:
        spawn_points.append(spawn_point)

    sbatch = []
    for spawn_point in spawn_points:
        walker_bp_choice = random.choice(walker_bp)
        if walker_bp_choice.has_attribute('is_invincible'):
            walker_bp_choice.set_attribute('is_invincible', 'false')

        sbatch.append(carla.command.SpawnActor(walker_bp_choice, spawn_point))

    results = client.apply_batch_sync(sbatch, True)

    for result in results:
        if not result.error:
            walkers.append(result.actor_id)

    batch = []
    for walker_id in walkers:
        batch.append(carla.command.SpawnActor(walker_controller_bp, carla.Transform(), walker_id))

    results = client.apply_batch_sync(batch, True)

    for result in results:
        if not result.error:
            controllers.append(result.actor_id)

    walker_actors = [world.get_actor(w) for w in walkers]
    controller_actors = [world.get_actor(c) for c in controllers]

    for controller in controller_actors:
        controller.start()
        controller.go_to_location(world.get_random_location_from_navigation())
        controller.set_max_speed(random.uniform(1.0, 3.0))  # Random speeds



    print(f"Spawned {len(walkers)} walkers.")

    actor_listw.extend(walker_actors)
    actor_listw.extend(controller_actors)

    return walker_actors

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

    actor_list = []
    actor_listw = []

    try:
        # Create client and connect to server (the simulator)
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)

        # Retrieve the world that is running
        world = client.get_world()
        original_settings = world.get_settings()
        settings = world.get_settings()

        # Set CARLA syncronous mode
        settings.fixed_delta_seconds = 0.1
        settings.synchronous_mode = True
        world.apply_settings(settings)

        sensor_queue = Queue()

        # Read our weather configurations from yaml and then set the first configuration to be the current weather
        weather = Weather(world.get_weather(), 'weathers.yaml')

        # The world contains the list of blueprints that we can use for adding new actors into the simulation.
        blueprint_library = world.get_blueprint_library()

        vehicle_blueprint = random.choice(blueprint_library.filter('vehicle.*.*'))
        spawn_points = world.get_map().get_spawn_points()
        ego = Ego_Vehicle(world, vehicle_blueprint, spawn_points)

        rgb_cam = Camera(world, sensor_queue, 'sensor.camera.rgb', carla.Transform(carla.Location(x=1.5, z=2.4)), 
                         out_dir = '_outRaw', file_type = 'png', cc = carla.ColorConverter.Raw)
        rgb_seg = Camera(world, sensor_queue, 'sensor.camera.semantic_segmentation', carla.Transform(carla.Location(x=1.5, z=2.4)),
                         out_dir = '_outSeg', file_type = 'png')
        lidar_cam = Camera(world, sensor_queue, 'sensor.lidar.ray_cast', carla.Transform(carla.Location(x=1.5, z=2.4)),
                           out_dir = '_outLIDAR', file_type = 'ply')
        lidar_seg = Camera(world, sensor_queue, 'sensor.lidar.ray_cast_semantic', carla.Transform(carla.Location(x=1.5, z=2.4)),
                           out_dir = '_outLIDARseg', file_type = 'ply')

        ego.add_camera(rgb_cam)
        ego.add_camera(rgb_seg)
        ego.add_camera(lidar_cam)
        ego.add_camera(lidar_seg)
        ego.configure_experiment(num_images_per_weather, [state['name'] for state in weather.states])

        # Store so we can delete later. Actors do not get removed automatically
        actor_list.append(ego.vehicle)

        vehicles = initialize_v(world, client, actor_list, spawn_points)
        walkers = initialize_w(world, client, actor_listw, spawn_points)
        

        last_value = -1
        print("printing walkers", walkers)
        while True:

            # Try and progress the weather to the next state if we have taken enough images for the current weather
            if ego.cameras[0].counter != last_value and ego.cameras[0].counter % num_images_per_weather == 0:
                last_value = ego.cameras[0].counter
                # for camera in ego.cameras:
                #     camera.counter = 0
                try:
                    weather.next()
                    time.sleep(1)
                    for w in walkers:
                        print(w.is_alive)
                    
                except StopIteration:
                    break

                if weather._sun.altitude < 15:
                    ego.lights_on()
                else:
                    ego.lights_off()
                world.set_weather(weather.weather)


            world.tick()

            if ego.cameras[0].has_new_image:
                try:
                    for _ in range(len(ego.cameras)):
                        s_frame = sensor_queue.get(True, 1.0)
                        print("    Frame: %d   Sensor: %s" % (s_frame[0], s_frame[1]))
                except Empty:
                    print("    Some of the sensor information is missed")
                ego.cameras[0].has_new_image = False


            

    finally:
        world.apply_settings(original_settings)
        # These cameras are our camera objects so they need to destroy themselves
        for camera in ego.cameras:
            camera.destroy()

        print('destroying actors')
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_listw])
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles]) # Why is this separate from the actor list above?
        print('done.')


if __name__ == '__main__':

    main()
