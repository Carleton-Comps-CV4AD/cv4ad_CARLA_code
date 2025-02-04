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
        "python3 data_collection.py --random_seed 654321 --num_images_per_weather 1250 --weather_config rainy_day.yaml",
        "python3 data_collection.py --random_seed 654321 --num_images_per_weather 1250 --weather_config foggy_day.yaml"
    ]
    run_commands(command_list)

if __name__ == "__main__":
    main()

