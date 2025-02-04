import subprocess
import sys
import os
import time
import signal
import random
from carla_flags import CARLA_DOWN, CARLA_RUNNING, COLLECTION_RUNNING, COLLECTION_COMPLETE, ALL_DONE


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

    num_videos = 650
    tick_rate = .166666667 # 6fps
    num_images_per_video = 18
    num_images_between_videos = 30

    MAX_IMAGES_PER_RUN = 400
    num_seeds = num_videos * (num_images_per_video + num_images_between_videos) // MAX_IMAGES_PER_RUN + 1


    seeds = [random.randint(0, 10239584) for i in range(num_seeds)]
    weathers = [] # a list of strings which are yamls

    while not os.path.exists("connector.txt"):
        time.sleep(1)

    # before starting collection, check if carla is running

    for weather in weathers:
        for seed in seeds:
            args = ['--config', f'{weather}.yaml', '--seed', str(seed)]

            status = -1
            while status != CARLA_RUNNING:
                with open("connector.txt", 'r') as f:
                    status = f.read()
                time.sleep(1)

            with open("connector.txt", 'w') as f:
                f.write(COLLECTION_RUNNING)
            return_code = run_collection(args)

            with open("connector.txt", 'w') as f:
                # TODO: if this is the last iteration write ALL_DONE instead!
                f.write(COLLECTION_COMPLETE)

if __name__ == '__main__':
    main()