Assuming CARLA 0.9.15 is installed

This folder contains our main scripts that helped us collect data.

data_collection.py is the main file to run which holds many arguements one can include to edit the dataset generated. 
It works with Town10HD CARLA Map, as we had to individually extract each possible spawnpoint to determinsitically spawn pedestriains. 
These spawn_points can be found in town_10_HD_walker_locations.txt.

Running: python3 data_collection.py --car_count 40 --num_images_per_weather 30     will give you a dataset of 30 images per weather
in the config file (which can also be modified) and within that Dataset, 40 cars will spawn around the map.

OUR CARLA SIMULATION:

We built our entire usage of CARLA program across multiple files:
- camera.py
- ego_vehicle.py
- utilities.py
- world.py
- weather.py
- bounding_box.py
Each of these files contribute to the overall function of data_collection.py. 


data_collection_driver.py uses our defined flags (carla_flags.py) to communicate with carla_driver.py in order to
automattically collect various deterministic datasets across multiple seeds. Both files need to be ran in different 
terminals with their respective virtual enviroments. 

Ideally, this would work for any time of dataset, however we struggled
to use the driver method to obtain non-video mode dataset. In order to obtain a dataset in non-video mode, we ran the drivers
in video mode, made a extremely large dataset with over 90 seeds, each containing various images, and paced out the first, middle, and
last image to create a regular dataset.
