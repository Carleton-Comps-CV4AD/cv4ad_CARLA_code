import subprocess
import sys
import os
import time
import signal
import psutil
from carla_flags import CARLA_DOWN, CARLA_RUNNING, COLLECTION_RUNNING, COLLECTION_COMPLETE, ALL_DONE

# This is if someone hits ctrl + C or something

def signal_handler(sig, frame):
    print("\nReceived termination signal (Ctrl+C) for CARLA driver.")
    
    with open("connector.txt", 'w') as f:
        f.write(CARLA_DOWN)
    
    sys.exit(0)  # Ensure the script exits cleanly

# Register the signal handler for SIGINT (Ctrl+C)
signal.signal(signal.SIGINT, signal_handler)

def run_carla(args) -> int:
    print("\nStarting CARLA")
    # Activate virtual environment
    command = ['./CarlaUE4.sh', *args]
    process = subprocess.Popen(command, cwd='/Carla/CARLA_0.9.15')
    pid = process.pid
    time.sleep(60) # Give Carla some time to start
    print("\nCARLA is running")
    return process, pid

# helper function for killing carla
def __find_carla_pids():
    """Find CARLA process PIDs safely."""
    pids = []
    for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
        try:
            if proc.info['cmdline'] and any("Carla" in arg for arg in proc.info['cmdline']):
                pids.append(proc.info['pid'])
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            continue
    return pids

def kill_carla(pid: int) -> int:
    # I am going to comment out what we wrote and replace it with some gnarly stuff but it seems 
    # like we need to do more than just run kill to ensure that all running processes/used memory related 
    # to CARLA is killed/freed for a graceful stop+restart
    try:
        print("Killing CARLA")


        # os.kill(pid, signal.SIGTERM)

        # # Check if the process is still running
        # if os.path.exists(f"/proc/{pid}"):
        #     print(f"Process {pid} is still running, forcing kill...")
        #     os.kill(pid, signal.SIGKILL)  # Force kill if it's still alive


    # Find CARLA PIDs
        carla_pids = __find_carla_pids()
        if not carla_pids:
            print("\nNo CARLA process found.")
            return

        # First, send SIGTERM (graceful shutdown)
        for pid in carla_pids:
            print(f"\nSending SIGTERM to CARLA process {pid}...")
            os.kill(pid, signal.SIGTERM)

        # Wait a bit, then force kill if still running
        time.sleep(2)
        for pid in carla_pids:
            if psutil.pid_exists(pid):
                print(f"\nProcess {pid} is still running, sending SIGKILL...")
                os.kill(pid, signal.SIGKILL)

        print("\nDone killing CARLA")
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
        process, pid = run_carla(args)
        time.sleep(20)

        with open("connector.txt", 'w') as f:
            f.write(CARLA_RUNNING)

        status = -1
        while (status != COLLECTION_COMPLETE) or (status != ALL_DONE):
            with open("connector.txt", 'r') as f:
                status = f.read()

            # print(status)

            if status == ALL_DONE:
                print("Killing CARLA since data collection is done.")
                kill_carla(pid)
                return
            elif status == COLLECTION_COMPLETE:
                break
            elif process.poll() is not None: # if we're not done but the CARLA subprocess is no longer running then start it back up
                f.write(CARLA_DOWN)
                break
            time.sleep(1)

        kill_carla(pid)
        print("Restarting CARLA to continue data collection.")
        time.sleep(15)

    # call run_carla, when we get the pid back, we write to a file that data_collection_driver can read
    # that tells it is is now okay to start collecting data
    # then, we will wait for a signal from the data_collection_driver to kill the carla process
    # then we will kill carla, start it again, and repeat the process

if __name__ == '__main__':
    main()