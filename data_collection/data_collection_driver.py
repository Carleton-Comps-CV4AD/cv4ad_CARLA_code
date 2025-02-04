import subprocess
import sys
import os
import time
import signal
import random
from carla_flags import CARLA_DOWN, CARLA_RUNNING, COLLECTION_RUNNING, COLLECTION_COMPLETE, ALL_DONE

# TODO have data_collection write the arguments to a file so we know what ran/how data was made
# TODO Implement piping
# TODO DEBUG bc file is just saving 1 photo stopping
# TODO subprocess should stop elegantly
# TODO fix connector.txt if crashes



def run_collection(args) -> int:
    # Activate virtual environment
    command = ['python3', 'data_collection.py', *args]
    process = subprocess.Popen(command)

    # Wait for the process to complete
    process.wait()
    return process.returncode

# You do not need to activate the virtual environment as a subprocess because the subprocess will inherit the environment
def main():
    random.seed(234905)

    num_videos = 10
    tick_rate = .166666667 # 6fps
    num_images_per_video = 18
    num_images_between_videos = 30

    MAX_IMAGES_PER_RUN = 100
    num_seeds = num_videos * (num_images_per_video + num_images_between_videos) // MAX_IMAGES_PER_RUN + 1

    num_videos_per_seed = (num_videos // num_seeds) +1


    seeds = [random.randint(0, 10239584) for i in range(num_seeds)]
    weathers = ["configs/clear_night", "configs/rainy_day"] # a list of strings which are yamls

    while not os.path.exists("connector.txt"):
        time.sleep(1)

    # before starting collection, check if carla is running

    for weather in weathers:
        for seed in seeds:
            args = ['--weather_config', f'{weather}.yaml', '--random_seed', str(seed),
                     '--videos_wanted', str(num_videos_per_seed), '--video_images_wait', str(num_images_between_videos),
                     '--video_images_saved', str(num_images_per_video), '--seconds_per_tick', str(tick_rate), '--video_mode']

            status = -1
            while status != CARLA_RUNNING:
                with open("connector.txt", 'r') as f:
                    status = f.read()
                time.sleep(1)

            with open("connector.txt", 'w') as f:
                f.write(COLLECTION_RUNNING)
            return_code = run_collection(args)

            with open("connector.txt", 'w') as f:
                if weather == weathers[-1] and seed == seeds[-1]:
                    f.write(ALL_DONE)
                else:
                    f.write(COLLECTION_COMPLETE)

if __name__ == '__main__':
    main()