import subprocess
import os
from ultralytics import YOLO
import cv2
from time import sleep
from time import time
from abc import ABC, abstractmethod

IR_CAMERA_NUM = 2 # IR camera device number
CAMERA_TIMEOUT = 10 # seconds to check room
CAMERA_THRESHOLD = 5 # number of person captures to consider success

DATA_LENGTH = 3 # number of last recorded values to calculate
TRANSITION_LIGHT_VALUE = 50 # the value for transition between considering dark and light
TRANSITION_SOUND_VALUE = 50 # the value for transition between considering sound and no sound 

VERBOSE = False # log shell outputs


ENV = os.environ.copy() # get current environment variables
ENV["PATH"] = f'{ENV["PATH"]}:{os.getcwd()}' # add pwd to PATH env so shell can find ardcom

# custom errors
class ArdcomError(Exception):
    def __init__(self, message):
        super().__init__(message)

class ArduinoError(Exception):
    def __init__(self, message):
        super().__init__(message)

# run commands in shell
def shell(command: str, timeout=5) -> subprocess.CompletedProcess[str] :

    output = subprocess.run(command, shell=True, capture_output=True, text=True, env=ENV, start_new_session=True, timeout=timeout)

    if VERBOSE:
        if output.returncode != 0:
            print(f"Error {output.returncode}: {output.stderr}")
        else:
            print(f"{output.stdout}")

    return output

# pre-written functions to run essential ardcom commands
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

# interface for states (defined as abstract class)
class IState(ABC):
    @abstractmethod
    def run(self) -> None:
        pass

# main finite state machine for program
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
            self.fsm.open_camera()

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
            # if detected, turn on lights
            if self.fsm.check_ir():
                ardcom_move_servo()
                sleep(5)
            
            # always go back to Start state after scan
            self.fsm.set_state(FSM.StartState(self.fsm))

    
    def __init__(self, data_length: int, transition_light_value: int, transition_sound_value: int, camera_scan_timeout: int, camera_scan_threshold: int):
        """
        data_length: number of last recorded values to calculate\n
        transition_value (light and sound): the value for transition between considering dark and light, or sound and no sound respectively\n
        camera_scan_timeout: time in seconds to scan before timeout and exiting scan\n
        camera_scan_threshold: number of successful scans required for success\n
        """
        self.state = None
        self.delay = 0
        self.light_data = [] # will only contain max of data length
        self.sound_data = [] # will only contain max of data length
        self.data_length = data_length
        self.transition_light_value = transition_light_value # value above this means room is bright, below means dark
        self.transition_sound_value = transition_sound_value # value above means sound is made, below means no sound (interference sound)

        self.camera_scan_threshold = camera_scan_threshold
        self.camera_scan_timeout = camera_scan_timeout
        self.camera = None
        self.model = YOLO(".model/yolo11n.pt")
        self.model_confidence = 50 # percent

    def open_camera(self) -> None:
        if not self.camera:
            self.camera = cv2.VideoCapture(IR_CAMERA_NUM)

        if not self.camera.isOpened():
            raise IOError("Cannot open camera")
    
    def take_camera(self) -> tuple[bool, cv2.typing.MatLike]:
        return self.camera.read()
    
    def close_camera(self) -> None:
        if self.camera:
            self.camera.release()

        self.camera = None

    def check_ir(self) -> bool:
            start_time = time()
            detection_num = 0

            while time() - start_time < self.camera_scan_timeout:
                is_captured, picture = self.take_camera() # capture an image

                if is_captured:
                    result = self.model(picture, verbose=False)[0] # only passed one image anyways

                    # finding index of "person" classification
                    person_index = 0 
                    for index, name in enumerate(result.names):
                        if name == "person":
                            person_index = index
                            break

                    # check if the result has any detection of person
                    for object in result.boxes:

                        # check if detection/object is a person
                        if not int(object.cls[0].item()) == person_index:
                            continue

                        # check if theres one person per image input only
                        if int(object.conf[0].item() * 100) > self.model_confidence:
                            detection_num += 1
                            break;

                # exit with 0, success
                if detection_num >= self.camera_scan_threshold:
                    self.close_camera()
                    return True
            
            # exit with 1, failed
            self.close_camera()
            return False

    # from IState
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
    sleep(1)

    machine = FSM(DATA_LENGTH, TRANSITION_LIGHT_VALUE, TRANSITION_SOUND_VALUE, CAMERA_TIMEOUT, CAMERA_THRESHOLD)
    machine.reset()

    while True:
        machine.run()
        sleep(machine.delay/1000)


if "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        ardcom_stop()