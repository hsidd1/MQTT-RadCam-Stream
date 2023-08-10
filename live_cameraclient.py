import cv2
from processModule.serverConnect import connect_mqtt
import yaml 
import numpy as np

with open("config.yaml", "r") as f:
    config = yaml.safe_load(f)

def main():
    #cap = cv2.VideoCapture(0)
    cap = cv2.VideoCapture(1, cv2.CAP_DSHOW) # external camera
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, config["LiveData"]["camera"]["width"])
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config["LiveData"]["camera"]["height"])
    # cap.set(cv2.CAP_PROP_FPS, config["LiveData"]["camera"]["fps"])
    while True:
        ret, frame = cap.read()
        if ret:
            cv2.imshow("frame", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        else:
            print("No frame received.")
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()