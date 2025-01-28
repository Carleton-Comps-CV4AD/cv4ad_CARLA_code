import carla
import os
import numpy as np
from camera import Camera

# Regarding organization:
# This should be split into two files: one for the ego vehicle and one for the camera.
# also, as mentioned somewhere else, the organization of responsibilities re: camera creation and configuration
# between the ego vehicle and the camera class is not good. This should be fixed.

# This is the ego vehicle class. It is responsible for creating the vehicle and adding cameras to it.
# It also has functions to turn the lights on and off, and to configure the experiment.

class Ego_Vehicle():
    def __init__(self, world: carla.World, spawn_point = None, blueprint_name: str = 'vehicle.tesla.cybertruck', random_seed = None):
        self.world = world

        # TODO: let's make the vehicle model a parameter instead of randomly selecting one
        # BUG: related to the above, our hardcoded camera positions do not work for all vehicle models
        # blueprint = np.random.choice(world.get_blueprint_library().filter('vehicle.*.*'))
        blueprint = world.get_blueprint_library().find(blueprint_name)
        spawn_points = world.get_map().get_spawn_points()
        if spawn_point is None:
            if random_seed is not None:
                np.random.seed(random_seed)
            spawn_point = np.random.choice(spawn_points)
        else:
            spawn_point = spawn_points[spawn_point]
        
        self.cameras = []
        
        self.vehicle = world.spawn_actor(blueprint, spawn_point)
        self.vehicle.set_autopilot(True)
        print('created %s' % self.vehicle.type_id)

    def add_camera(self, camera: Camera):
        camera_actor = self.world.spawn_actor(camera.camera_blueprint, camera.transform, attach_to=self.vehicle)
        camera.set_actor(camera_actor)
        camera_actor.listen(lambda image: camera.listen(image))
        self.cameras.append(camera)
        print('created %s' % camera.blueprint)

    def configure_experiment(self, num_images_per_weather, weathers):
        for camera in self.cameras:
            camera.configure_experiment(num_images_per_weather, weathers)

    # Turning on and off the lights should be parameterized. Also, it's unclear if it's even necessary.
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