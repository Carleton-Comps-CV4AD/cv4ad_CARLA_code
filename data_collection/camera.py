import carla
import os

def get_vehicle_locations(world):
    # This is a tuple of the vehicle id, the bounding box, and the transform
    vehicle_locations = []
    for npc in world.get_actors().filter('*vehicle*'):
        id = npc.id
        bb = npc.bounding_box
        transform = npc.get_transform()
        
        # ! Warning:
        # * So, the thought process behind doing all this is to guarantee that the bounding boxes we get are
        # * taken in the same frame as the image, since we had problems where the boxes were in front/behind (temporally)
        # * where they should have been. Somehow doing this seems to have improved the situation, though there is still some 
        # * misalignment. The weird thing is, as I see it, these tuples contain references to the actual objects, so they should
        # * be updated in real time, and I should not actually be saving any information -- I don't know why this appears to work.
        # * I tried using copy or deeopcopy, but the carla objects are not pickleable, and I don't feel like implementing a custom
        # * deepcopy function for them. I'm not sure what's going on here, but I'm not going to mess with it unless the misalignment
        # * turns out to be a problem.
        # * If someone feels like it, try taking this hack where we save the values out and using the commented out bounding box method in
        # * data_collection/bounding_boxes.py and see if it works. I'm not going to do it because I don't want to mess with it.
        vehicle_locations.append((id, bb, transform))
    return vehicle_locations

# This is the annoying thing: this camera just describes a blueprint with attributes and a listen function configured as we desire
# We make this type of camera, with some settings, and pass it to the ego vehicle,
# which actually uses the data in the object to spawn a camera actor. Thus, we only actually
# have a camera actor in here after the car creates a camera and puts it in the camera class we have made
# This is pretty bad organization and I should fix this. Sorry :(
class Camera():
    def __init__(self, world: carla.World, sensor_queue, blueprint: str, transform: carla.Transform, 
                 name, file_type, cc = None, out_dir = ".", seconds_per_tick = 0, video_mode_state = False, video_wait = 0, video_images_saved = 0):
        self.blueprint: str = blueprint
        self.transform = transform
        self.counter = 0
        self.sensor_queue = sensor_queue

        blueprint_library = world.get_blueprint_library()
        self.camera_blueprint: carla.ActorBlueprint = blueprint_library.find(blueprint)
        self.transform = carla.Transform(carla.Location(x=1.5, z=2.4))
            
        self.camera_blueprint.set_attribute('sensor_tick', str(seconds_per_tick))

        self.out_dir = out_dir
        self.name = name
        self.file_type = file_type
        self.cc = cc

        self.has_new_image = False
        self.camera: carla.Actor = None

        self.world = world
        self.world_vehicles_locations_at_last_image = None
        self.transform_at_last_image = None

        self.video_mode = video_mode_state
        self.video_images_saved = video_images_saved
        self.video_images_wait = video_wait

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
        if self.video_mode and (self.counter % (self.video_images_wait + self.video_images_saved)) >= self.video_images_saved:
            
            self.increment()
            return
        self.world_vehicles_locations_at_last_image = get_vehicle_locations(self.world)

        # ! Here too. See the comment above that says "Warning`"
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