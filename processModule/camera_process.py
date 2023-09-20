import numpy as np
import yaml 
import cv2 

with open("config.yaml", "r") as f:
    config = yaml.safe_load(f)

cv2.namedWindow(f"Frame from {config['mqtt']['client_id2']}", cv2.WINDOW_NORMAL)
SAMPLE_IMG = "data/sample_frame.png"
data_array = cv2.imread(SAMPLE_IMG)

def process_frames(frame_payload: bytearray) -> None:
    # convert byte array to numpy array for cv2 to read
    frame = np.frombuffer(frame_payload, dtype=np.uint8)
    height, width, channels = 720,1280,3#data_array.shape
    frame = frame.reshape(height, width, channels)
    cv2.imshow(f"Frame from {config['mqtt']['client_id2']}", frame)
    if config["CameraOutput"]["continuous_frame_mode"]:
        cv2.waitKey(1)
    else:
        cv2.waitKey(0)

def process_livecam(payload: bytearray) -> None:
    timestamp = payload[-26:].decode("utf-8")
    print("Timestamp: ", timestamp)
    frame_payload = payload[:-26] # remove timestamp

    # delimiter = b'|'
    # delimiter_index = payload.index(delimiter)
    # frame_payload = payload[:delimiter_index]
    # timestamp = payload[delimiter_index + len(delimiter):].decode("utf-8")

    # convert byte array to numpy array for cv2 to read
    frame = np.frombuffer(frame_payload, dtype=np.uint8)
    if not hasattr(process_livecam, 'window_created'):
        cv2.namedWindow("Live Camera Feed")
        process_livecam.window_created = True
    font = cv2.FONT_HERSHEY_SIMPLEX
    bottom_left_corner = (10, 30)
    font_scale = 1
    font_color = (255, 255, 255)
    line_type = 2
    frame = frame.reshape([720,1280,3])#data_array.shape)
    cv2.putText(frame, timestamp, bottom_left_corner, font, font_scale, font_color, line_type)
    cv2.imshow("Live Camera Feed", frame)
    if config["CameraOutput"]["continuous_frame_mode"]:
        cv2.waitKey(1)
    else:
        cv2.waitKey(0)