import subprocess

def run_commands(command_list):
    for command in command_list:
        print(f"Running: {command}")
        process = subprocess.run(command, shell=True)
        if process.returncode != 0:
            print(f"Error: Command '{command}' exited with code {process.returncode}")
        else:
            print(f"Finished: {command}\n")

def main():
    command_list = [
        "python3 -m data_collection --video_mode --videos_wanted 10 --video_images_saved  18 --video_images_wait 15 --random_seed 3683 --seconds_per_tick 0.167 --weather_config configs_yamls/clear_day.yaml",
    ]
    run_commands(command_list)

if __name__ == "__main__":
    main()

