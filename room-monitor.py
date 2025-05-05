from sre_parse import State
import subprocess
import os
from abc import ABC, abstractmethod

ENV = os.environ.copy() # get current environment variables
ENV["PATH"] = f'{ENV["PATH"]}:{os.getcwd()}' # add pwd to PATH env so shell can find ardcom

class ArdcomError(Exception):
    def __init__(self, message):
        super().__init__(message)

class ArduinoError(Exception):
    def __init__(self, message):
        super().__init__(message)

def shell(command: str) -> subprocess.CompletedProcess[str] :
    output = subprocess.run(command, shell=True, capture_output=True, text=True, env=ENV)
    return output

def ardcom_start() -> None:
    output = shell("ardcom start")
    if output.returncode != 0:
        raise ArdcomError(output.stderr)
    
def ardcom_stop() -> None:
    output = shell("ardcom stop")
    if output.returncode != 0:
        raise ArdcomError(output.stderr)
    
def ardcom_log() -> tuple[int, int]:
    output_ardcom = shell("ardcom send -log")
    if output_ardcom.returncode != 0:
        raise ArdcomError(output_ardcom.stderr)
    
    output_arr = output_ardcom.stdout.splitlines()
    if len(output_arr) > 1:
        raise ArdcomError(f"Received unexpected output, ardcom out of sync? output: {output_ardcom.stdout}")
    
    output = output_arr[0]
    
    arduino_code = output[0]
    arduino_msg = output[1:]
    if arduino_code == ":":
        split_msg = arduino_msg.split(",")
        return (int(split_msg[0]), int(split_msg[1]))
    elif arduino_code == "-":
        raise ArduinoError(arduino_msg)
    else:
        raise Exception(f"Unknown error, ardcom received an invalid message from serial: {output_ardcom.stdout}")
    
def ardcom_move_servo() -> None:
    output_ardcom = shell(f"ardcom send -ms")
    if output_ardcom.returncode != 0:
        raise ArdcomError(output_ardcom.stderr)

def main():
    ardcom_start()


if "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        ardcom_stop()