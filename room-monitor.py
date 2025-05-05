import subprocess
import os
import sys
from time import sleep
from abc import ABC, abstractmethod

VERBOSE = True

VENV_PATH = "./.venv/"
PYTHON_PATH = f"{VENV_PATH}bin/python"
REQUIREMENTS = "./requirements.txt"

ENV = os.environ.copy() # get current environment variables
ENV["PATH"] = f'{ENV["PATH"]}:{os.getcwd()}' # add pwd to PATH env so shell can find ardcom

class ArdcomError(Exception):
    def __init__(self, message):
        super().__init__(message)

class ArduinoError(Exception):
    def __init__(self, message):
        super().__init__(message)

def shell(command: str) -> subprocess.CompletedProcess[str] :
    output = subprocess.run(command.split(" "), capture_output=True, text=True, env=ENV, timeout=5, start_new_session=True)

    if VERBOSE:
        if output.returncode != 0:
            print(f"Error {output.returncode}: {output.stderr}")
        else:
            print(f"{output.stdout}")

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

def shell_python(command: str) -> subprocess.CompletedProcess[str]:
    return shell(f"{PYTHON_PATH} {command}")

def generate_venv() -> None:
    # if venv doesnt exist
    if not os.path.isdir(VENV_PATH):
        shell(f"{sys.executable} -m venv {VENV_PATH}")

    # install required pip stuff
    if os.path.isfile(REQUIREMENTS):
        shell_python(f"-m pip install -r {REQUIREMENTS}")
    else:
        raise FileNotFoundError(f"{REQUIREMENTS} not found")

# interface for states (defined as abstract class)
class IState(ABC):
    @abstractmethod
    def run(self) -> None:
        pass

class FSM(IState):
    # abstract STATE class
    class State(IState, ABC):
        def __init__(self, fsm: "FSM"):
            self.fsm = fsm

        @abstractmethod
        def run(self) -> None:
            pass

    # child STATE classes

    # when room is still bright, periodically check room is dark to transition to StartState
    class AwaitState(State):
        def __init__(self, fsm: "FSM"):
            FSM.State.__init__(self, fsm)
            self.fsm.delay = 5000
        
        def run(self):
            sound_data, light_data = ardcom_log()
            self.fsm.add_light_data(light_data)

            # if room is dark
            success, data = self.fsm.get_light_data()
            if success and data < self.fsm.transition_light_value:
                self.fsm.set_state(FSM.StartState(self.fsm))
                

    # when room is dark, will check for sound and light to turn on light with servo
    class StartState(State):
        def __init__(self, fsm: "FSM"):
            FSM.State.__init__(self, fsm)
            self.fsm.delay = 100

        def run(self):
            sound_data, light_data = ardcom_log()

            self.fsm.add_light_data(light_data)
            self.fsm.add_sound_data(sound_data)

            light_success, light_value = self.fsm.get_light_data()

            # if room is bright
            if light_success and light_value > self.fsm.transition_light_value:
                self.fsm.set_state(FSM.AwaitState(self.fsm))
                return
            
            # if room is dark and sound is detected
            sound_success, sound_value = self.fsm.get_sound_data()
            if sound_success and sound_value > self.fsm.transition_sound_value:
                self.fsm.set_state(FSM.CheckState(self.fsm))

    # open ir camera to check room
    class CheckState(State):
        def __init__(self, fsm: "FSM"):
            FSM.State.__init__(self, fsm)
            self.fsm.delay = 100

        def run(self):
            ir_scan_file = "./ir_scan.py"
            output = None

            # run ir_scan file to check room for people detection
            if os.path.isfile(ir_scan_file):
                output = shell_python(ir_scan_file)
            else:
                raise FileNotFoundError(f"{ir_scan_file} not found")
            
            # if detected, turn on lights
            if output.returncode == 0:
                ardcom_move_servo()
            
            # always go back to Start state after scan
            self.fsm.set_state(FSM.StartState(self.fsm))

    def __init__(self, data_length: int, transition_light_value: int, transition_sound_value: int):
        self.state = None
        self.delay = 0
        self.light_data = [] # will only contain max of data length
        self.sound_data = [] # will only contain max of data length
        self.data_length = data_length
        self.transition_light_value = transition_light_value # value above this means room is bright, below means dark
        self.transition_sound_value = transition_sound_value # value above means sound is made, below means no sound (interference sound)

    def run(self) -> None:
        self.state.run()

    def reset(self) -> None:
        self.set_state(self.AwaitState(self))

    def reset_data(self) -> None:
        self.sound_data.clear();
        self.light_data.clear();

    # add light data to the list, making sure list doesnt exceed length
    def add_light_data(self, data: int) -> None:
        self.light_data.append(data)

        while len(self.light_data) > self.data_length:
            self.light_data.pop(0)

    # bool determines if the value is valid or not
    def get_light_data(self) -> tuple[bool, int]:
        if len(self.light_data) != self.data_length:
            return False, 0
        
        avg = sum(self.light_data) / self.data_length
        return True, avg
    
    # add sound data to the list, making sure list doesnt exceed length
    def add_sound_data(self, data: int) -> None:
        self.sound_data.append(data)

        while len(self.sound_data) > self.data_length:
            self.sound_data.pop(0)

    # bool determines if the value is valid or not
    def get_sound_data(self) -> tuple[bool, int]:
        if len(self.sound_data) != self.data_length:
            return False, 0
        
        avg = sum(self.sound_data) / self.data_length
        return True, avg
    
    # set new state and start data collection anew
    def set_state(self, state: State) -> None:
        self.reset_data()
        self.state = state

def main():
    ardcom_start()
    generate_venv()

    machine = FSM(5, 50, 50)
    machine.reset()

    while True:
        machine.run()
        sleep(machine.delay)


if "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        ardcom_stop()