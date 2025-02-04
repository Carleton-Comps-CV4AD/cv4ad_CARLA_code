import subprocess
import sys
import os
import time
import signal
from carla_flags import CARLA_DOWN, CARLA_RUNNING, COLLECTION_RUNNING, COLLECTION_COMPLETE, ALL_DONE

def run_carla(args) -> int:
    # Activate virtual environment
    command = ['./CarlaUE4.sh', *args]
    process = subprocess.Popen(command, cwd='/Carla/CARLA_0.9.15')
    pid = process.pid
    time.sleep(60) # Give Carla some time to start
    return pid

def kill_carla(pid: int) -> int:
    try:
        os.kill(pid, signal.SIGTERM)
        return 0
    except ProcessLookupError:
        print("Carla process not found")
        return 1
    except Exception as e:
        print(f"Error while killing Carla process: {e}")
        return 1

# You do not need to activate the virtual environment as a subprocess because the subprocess will inherit the environment
def main():
    if os.path.exists("connector.txt"):
        os.remove("connector.txt")
    with open("connector.txt", 'w') as f:
        f.write(CARLA_DOWN)

    while True:
        args = ['-prefernvidia', '-RenderOffScreen']
        pid = run_carla(args)
        time.sleep(20)

        with open("connector.txt", 'w') as f:
            f.write(CARLA_RUNNING)

        status = -1
        while (status != COLLECTION_COMPLETE) or (status != ALL_DONE):
            with open("connector.txt", 'r') as f:
                status = f.read()
            if status == ALL_DONE:
                kill_carla(pid)
                return
            time.sleep(1)

        kill_carla(pid)
        time.wait(15)

    # call run_carla, when we get the pid back, we write to a file that data_collection_driver can read
    # that tells it is is now okay to start collecting data
    # then, we will wait for a signal from the data_collection_driver to kill the carla process
    # then we will kill carla, start it again, and repeat the process

if __name__ == '__main__':
    main()