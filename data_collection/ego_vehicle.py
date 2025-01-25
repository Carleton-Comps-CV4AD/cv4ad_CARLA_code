import carla
import random
import os
from copy import deepcopy

class Ego_Vehicle():
    def __init__(self, world, spawn_point = None):
        self.world = world

        blueprint = random.choice(world.get_blueprint_library().filter('vehicle.*.*'))
        spawn_points = world.get_map().get_spawn_points()
        if spawn_point is None:
            spawn_point = random.choice(spawn_points)
        else:
            spawn_point = spawn_points[spawn_point]
        
        self.cameras = []
        
        # So let's tell the world to spawn the vehicle.
        self.vehicle = world.spawn_actor(blueprint, spawn_point)
        self.vehicle.set_autopilot(True)
        print('created %s' % self.vehicle.type_id)

    def add_camera(self, camera):
        camera_actor = self.world.spawn_actor(camera.camera_blueprint, camera.transform, attach_to=self.vehicle)
        camera.set_actor(camera_actor)
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

def get_vehicle_locations(world):
    # This is a tuple of the vehicle id, the bounding box, and the transform
    vehicle_locations = []
    for npc in world.get_actors().filter('*vehicle*'):
        id = npc.id
        bb = npc.bounding_box
        transform = npc.get_transform()
        vehicle_locations.append((id, bb, transform))
    return vehicle_locations

# This is the annoying thing: this camera just describes a blueprint with attributes and a listen function configured as we desire
# We make this type of camera, with some settings, and pass it to the ego vehicle,
# which actually uses the data in the object to spawn a camera actor. Thus, we only actually
# have a camera actor in here after the car creates a camera and puts it in the camera class we have made
# This is pretty bad organization and I should fix this. Sorry :(
class Camera():
    def __init__(self, world, sensor_queue, blueprint, transform, name, file_type, cc = None, out_dir = ".", seconds_per_tick = 0):
        self.blueprint = blueprint
        self.transform = transform
        self.counter = 0
        self.sensor_queue = sensor_queue

        blueprint_library = world.get_blueprint_library()
        self.camera_blueprint = blueprint_library.find(blueprint)
        self.transform = carla.Transform(carla.Location(x=1.5, z=2.4))
            
        self.camera_blueprint.set_attribute('sensor_tick', str(seconds_per_tick))

        self.out_dir = out_dir
        self.name = name
        self.file_type = file_type
        self.cc = cc

        self.has_new_image = False
        self.camera = None

        self.world = world
        self.world_vehicles_locations_at_last_image = None
        self.transform_at_last_image = None

    def set_actor(self, actor):
        self.camera = actor

    def configure_experiment(self, num_images_per_weather, weathers):
        self.num_images_per_weather = num_images_per_weather
        self.weathers = weathers

    def set_image_size(self, x = '1920', y = '1080'):
        self.camera_blueprint.set_attribute('image_size_x', x)
        self.camera_blueprint.set_attribute('image_size_y', y)

    def set_shutter_speed(self, speed = 60):
        self.camera_blueprint.set_attribute('shutter_speed', str(speed))
    
    def listen(self, image):
        print(f"{self.name}, {self.counter}")
        self.world_vehicles_locations_at_last_image = get_vehicle_locations(self.world)
        self.transform_at_last_image = self.camera.get_transform()
        self.sensor_queue.put((image, self.blueprint))
        weather_name = self.weathers[self.counter // self.num_images_per_weather]
        image_path = os.path.join(self.out_dir, weather_name, self.name, f'{self.counter}.{self.file_type}')
        if self.cc:
            image.save_to_disk(image_path, self.cc)
        else:
            image.save_to_disk(image_path)
        self.increment()
        self.has_new_image = True
    
    def increment(self):
        self.counter += 1

    def destroy(self):
        self.camera.destroy()