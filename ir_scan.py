from ultralytics import YOLO
import cv2
from time import time
import sys

IR_CAMERA_NUM = 2 # IR camera device number
CONFIDENCE = 50 # percentage
THRESHOLD = 5 # number of person captures to consider success
TIMEOUT = 10 # seconds to check room

def main():
    model = YOLO(".model/yolo11n.pt")

    camera = cv2.VideoCapture(IR_CAMERA_NUM)
    if not camera.isOpened():
        raise IOError("Cannot open camera")

    start_time = time()
    detection_num = 0

    while time() - start_time < TIMEOUT:
        is_captured, picture = camera.read() # capture an image

        if is_captured:
            result = model(picture, verbose=False)[0] # only passed one image anyways

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
                if int(object.conf[0].item() * 100) > CONFIDENCE:
                    detection_num += 1
                    break;

        # exit with 0, success
        if detection_num >= THRESHOLD:
            camera.release()
            sys.exit(0)
    
    # exit with 1, failed
    camera.release()
    sys.exit(1)


if "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Exiting...")
