import yaml

# I inherited this sun/weather organization from the carla examples, but I think it's a bit uneccessary
# I think I should just have a weather object that holds the weather and sun information. If someone feels 
# like it, they can refactor this.
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

        self.initial_state = self.states[0]
        self.set_weather(self.initial_state)

    def set_weather(self, state):
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

    def next(self):
        print("\n\n\n\n called next \n\n\n\n")
        try:
            state = self.states_iter.__next__()
            print(f"setting weather to", state['name'])
            self.set_weather(state)
            return 0
        except StopIteration:
            return -1

    def __str__(self):
        return '%s %s' % (self._sun)