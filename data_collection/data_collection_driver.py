import subprocess
import sys
import os
import time
import signal
import random
from carla_flags import CARLA_DOWN, CARLA_RUNNING, COLLECTION_RUNNING, COLLECTION_COMPLETE, ALL_DONE, random_seeds

# TODO have data_collection write the arguments to a file so we know what ran/how data was made -- DONE (I think)
# TODO Implement piping
# TODO DEBUG bc file is just saving 1 photo stopping
# TODO subprocess should stop elegantly
# TODO fix connector.txt if crashes

# im finna come in and attempt to fix the last 3

"""
current lead: go to the part where we progress to the next weather and print he logic.
 for some reason the thing crashes sometimes but not other times...? maybe its a crala thing and we need to restart t actually.

 implement ways to wrie to connector.txt immediately something happens like a subprocess is done or one of the files like crashes or something
 do this with the appropriate carla flag

 this is part of subprocess topping elegantly since it will help us ensure that the file is upated to notify everyone whenevr this happens


 also what if we did like a 
 
 """


# This is if someone hits Ctrl + C while this script is running
def signal_handler(sig, frame):
    print("\nReceived termination signal (Ctrl+C) for data_collection_driver")

    carla_down = False

    with open("connector.txt", 'r') as f:
        if f.read() == CARLA_DOWN:
            carla_down = True
    
    with open("connector.txt", 'w') as f:
        if carla_down:
            f.write(CARLA_DOWN)
        else:
            f.write(CARLA_RUNNING)
    
    sys.exit(0)  # Ensure the script exits cleanly

# Register the signal handler for SIGINT (Ctrl+C)
signal.signal(signal.SIGINT, signal_handler)



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
    num_images_between_videos = 40

    MAX_IMAGES_PER_RUN = 400
    num_seeds = num_videos * (num_images_per_video + num_images_between_videos) // MAX_IMAGES_PER_RUN + 1

    num_videos_per_seed = (num_videos // num_seeds) +1


    # seeds = [random.randint(0, 10239584) for i in range(num_seeds)]
    seeds = random_seeds

    # a list of strings which are yamls
    weathers = ["configs/foggy_day", "configs/rainy_day"] #["configs/clear_day", "configs/clear_night", "configs/clear_sunset", "configs/clear_wet_day", "configs/foggy_day", "configs/rainy_day"] 

    while not os.path.exists("connector.txt"):
        time.sleep(1)

    # before starting collection, check if carla is running

    for weather in weathers:
        print(f'\n\nStarting a new weather - {weather}\n\nThis is weather {weathers.index(weather) +1}/{len(weathers)}\n')

        for seed in seeds:
            print(f'\n\nStarting a new seed - {seed}\n\n This is seed {seeds.index(seed) + 1}/{len(seeds)} for weather {weathers.index(weather) +1}/{len(weathers)}\n')

            args = ['--weather_config', f'{weather}.yaml', '--random_seed', str(seed),
                     '--videos_wanted', str(num_videos_per_seed), '--video_images_wait', str(num_images_between_videos),
                     '--video_images_saved', str(num_images_per_video), '--seconds_per_tick', str(tick_rate), '--video_mode',
                     '--output_dir', f'/Data/video_data/{weather[8:]}-{seed}']
            

            status = -1
            while status != CARLA_RUNNING:
                with open("connector.txt", 'r') as f:
                    status = f.read()
                time.sleep(1)

            print("carla is running!!")
            with open("connector.txt", 'w') as f:
                f.write(COLLECTION_RUNNING)

            return_code = 1

            while return_code != 0:
                return_code = run_collection(args)

                with open("connector.txt", 'w') as f:
                    if return_code != 0: # do not write to file because we're still collectig data
                        print('Something happened and return code was not 0, so will collect again')
                        f.write(CARLA_RUNNING)
                    elif weather == weathers[-1] and seed == seeds[-1]:
                        f.write(ALL_DONE)
                    else:
                        f.write(COLLECTION_COMPLETE)


if __name__ == '__main__':
    main()