import carla
import random
import os

# Global configuration
SECONDS_PER_TICK = 5.0 # seconds per tick

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

    def set_image_size(self, x = '1920', y = '1080'):
        self.camera.set_attribute('image_size_x', x)
        self.camera.set_attribute('image_size_y', y)
    
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