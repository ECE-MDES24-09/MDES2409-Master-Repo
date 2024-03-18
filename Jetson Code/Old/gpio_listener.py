import Jetson.GPIO as GPIO
import time
import subprocess
import signal
import os

# Pin Definitions
start_pin = 13  # Pin to start the program
stop_pin = 24   # Pin to stop the program
program_process = None

def start_callback(channel):
    global program_process
    # Start the program only if it's not already running
    if program_process is None or program_process.poll() is not None:
        program_process = subprocess.Popen(['python3', '/home/jetson/GameObjDetector/objdetclass_async.py'])

def stop_callback(channel):
    global program_process
    # Attempt to stop the program gracefully
    if program_process is not None:
        program_process.terminate()
        try:
            # Wait for the process to terminate
            program_process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            # If the process does not terminate in time, force kill it
            os.kill(program_process.pid, signal.SIGKILL)

GPIO.setmode(GPIO.BOARD)
GPIO.setup(start_pin, GPIO.IN, initial=GPIO.HIGH)
GPIO.setup(stop_pin, GPIO.IN, initial=GPIO.HIGH)

GPIO.add_event_detect(start_pin, GPIO.FALLING, callback=start_callback, bouncetime=200)
GPIO.add_event_detect(stop_pin, GPIO.FALLING, callback=stop_callback, bouncetime=200)


try:
    while True:
        time.sleep(1)
finally:
    GPIO.cleanup()
