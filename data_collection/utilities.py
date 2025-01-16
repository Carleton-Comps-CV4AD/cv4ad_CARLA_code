import carla
import yaml
import random
import logging

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

class World():
    def __init__(self, host: str, port: int, synchronous: bool = True, 
                 max_num_vehicles: int = 50, max_num_walkers: int = 100):
        # Create client and connect to server (the simulator)
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)

        # Retrieve the world that is running
        # world = client.load_world('Town10HD')
        self.world = self.client.get_world()
        
        self.original_settings = self.world.get_settings()
        self.settings = self.world.get_settings()
    
        # Set CARLA syncronous mode
        self.settings.fixed_delta_seconds = 0.1
        self.settings.synchronous_mode = True
        self.world.apply_settings(self.settings)

        # Traffic manager
        self.traffic_manager = self.client.get_trafficmanager()
        self.traffic_manager.set_synchronous_mode(True)

        self.walkers = []
        self.vehicles = []

        self.max_num_vehicles = max_num_vehicles
        self.max_num_walkers = max_num_walkers


    # Fix this organization -> decide whether Weather or world should hold the weather object
    def load_weathers(self, configs):
        self.weather = Weather(self.world.get_weather(), configs)

    def get_weather(self):
        return self.world.get_weather()
    
    def update_weather(self):
        self.world.set_weather(self.weather.weather)
    
    def get_spawn_points(self):
        return self.world.get_map().get_spawn_points()
    
    def get_blueprints(self, filter):
        return self.world.get_blueprint_library().filter(filter)
    
    def spawn_car(self, filter = 'vehicle.*.*', number = 1):
        if len(self.vehicles) > self.max_num_vehicles:
            # raise Exception('Max number of vehicles reached')
            print('Max number of vehicles reached')

        spawn_points = self.get_spawn_points()

        max_vehicles = 30
        max_vehicles = min([max_vehicles, len(spawn_points)])
        new_vehicles = []
        
        successfully_spawned = 0
        for i in range(number):
            blueprint = random.choice(self.world.get_blueprint_library().filter(filter))
            spawn_point = random.choice(spawn_points)
            vehicle = self.world.try_spawn_actor(blueprint, spawn_point)
            if vehicle is not None:
                new_vehicles.append(vehicle)
                successfully_spawned += 1
            else:
                print('Could not spawn vehicle')

        for v in new_vehicles:
            v.set_autopilot(True) 
            self.traffic_manager.random_left_lanechange_percentage(v, .1)
            self.traffic_manager.random_right_lanechange_percentage(v, .1)
            self.traffic_manager.auto_lane_change(v, True)  
            self.traffic_manager.set_synchronous_mode(True)
        
        self.vehicles.extend(new_vehicles)

        return successfully_spawned

        
    def spawn_walker(self, filter = 'walker.pedestrian.*', number = 1):
        if len(self.walkers) > self.max_num_walkers:
            raise Exception('Max number of walkers reached')
        
        spawn_points = []
        for i in range(number):
            spawn_point = carla.Transform()
            loc = self.world.get_random_location_from_navigation()
            if (loc != None):
                spawn_point.location = loc
                spawn_points.append(spawn_point)

        new_walkers = []
        
        # Select some models from the blueprint library
        blueprint = random.choice(self.world.get_blueprint_library().filter(filter))
        if blueprint.has_attribute('is_invincible'):
            blueprint.set_attribute('is_invincible', 'false')

        successfully_spawned = 0
        for i in range(number):
            blueprint = random.choice(self.world.get_blueprint_library().filter(filter))
            spawn_point = carla.Transform()
            loc = self.world.get_random_location_from_navigation()
            if (loc != None):
                spawn_point.location = loc
            else:
                print("Could not spawn walker: location is none")
                continue

            walker = self.world.try_spawn_actor(blueprint, spawn_point)
            if walker is not None:
                new_walkers.append(walker)
                successfully_spawned += 1
            else:
                print('Could not spawn walker')
        
        batch = []
        walker_controller_bp = self.world.get_blueprint_library().find('controller.ai.walker')
        for i in range(len(new_walkers)):
            batch.append(carla.command.SpawnActor(walker_controller_bp, carla.Transform(), new_walkers[i]))
        results = self.client.apply_batch_sync(batch, True)
        for i in range(len(results)):
            if results[i].error:
                logging.error(results[i].error)
            else:
                controller_id = results[i].actor_id
                controller = self.world.get_actor(controller_id)
                controller.start()
                controller.go_to_location(self.world.get_random_location_from_navigation())
                controller.set_max_speed(random.uniform(1.0, 3.0))  # Random speeds
                self.walkers.append((new_walkers[i], controller))

        return successfully_spawned

    def replace_dead_walkers(self):
        death_count = 0
        respawn_count = 0

        for walker, controller in self.walkers:
            if walker.is_active == False:
                death_count += 1
                self.client.apply_batch([carla.command.DestroyActor(walker)])
                self.client.apply_batch([carla.command.DestroyActor(controller)])
                self.walkers.remove((walker, controller))
                successfully_spawned = 0

                attempts = 0
                while successfully_spawned == 0 and attempts < 10:
                    successfully_spawned = self.spawn_walker()
                    attempts += 1
                respawn_count += 1
        
        print(f"{death_count} dead walkers replaced with {respawn_count} new walkers")
    
    def clean_up(self):
        self.world.apply_settings(self.original_settings)
        self.client.apply_batch([carla.command.DestroyActor(x) for x in self.vehicles])
        self.client.apply_batch([carla.command.DestroyActor(x[0]) for x in self.walkers])
        self.client.apply_batch([carla.command.DestroyActor(x[1]) for x in self.walkers])
        self.vehicles = []
        self.walkers = []
        print('Cleaned up')