import numpy as np
import yaml 
import cv2 

with open("config.yaml", "r") as f:
    config = yaml.safe_load(f)

cv2.namedWindow(f"Frame from {config['mqtt']['client_id2']}", cv2.WINDOW_NORMAL)
SAMPLE_IMG = "data/sample_frame.png"
data_array = cv2.imread(SAMPLE_IMG)

def process_frames(frame_payload: bytearray, ts_pub) -> None:
    # convert byte array to numpy array for cv2 to read
    frame = np.frombuffer(frame_payload, dtype=np.uint8)
    height, width, channels = data_array.shape
    frame = frame.reshape(height, width, channels)
    cv2.imshow(f"Frame from {config['mqtt']['client_id2']}", frame)
    if config["write_log"]: # write frame with timestamp as name to logs/frames
        filename = f"logs/frames/{ts_pub}.jpg"
        cv2.imwrite(filename, frame)
        print(f"Frame saved to {filename}")
    if config["CameraOutput"]["continuous_frame_mode"]:
        cv2.waitKey(1)
    else:
        cv2.waitKey(0)