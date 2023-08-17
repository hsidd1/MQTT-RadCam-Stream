import cv2 
import numpy as np
import json 
import datetime as dt
from radar_process import readbuffer_process

"""
run as thread, writes live payload data: ./liveDataLog:
/camera_data: video frames as jpeg, filename=timestamp (publish)
/radcam_log.json: 
camera: {topic, sub_ts, pub_ts (matching filename in ./camera_data), frame_id}
 radar: {topic, sub_ts, pub_ts, radar_json (sensor id, x, y, z, tlv, tid, frame)} 
"""

IMG_PATH = "../liveDataLog/camera_data/"
RADCAM_PATH = "../liveDataLog/radcam_log"
SAMPLE_IMG = "../data/sample_frame.png"
data_array = cv2.imread(SAMPLE_IMG)
CAMERA_TOPIC = "data/livecamera"
RADAR_TOPIC = "data/liveradar"

def save_data(topic: str, payload: bytearray) -> None:
    if not hasattr(save_data, "frame_id"):
        save_data.frame_id = 0
    cam_object = radar_object = None
    # save cam data frames
    sub_ts = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
    if topic == CAMERA_TOPIC:
        save_data.frame_id += 1
        cam_ts = payload[-26:].decode("utf-8") # ts is 26 bytes extension
        frame_payload = payload[:-26] # remove timestamp
        frame = np.frombuffer(frame_payload, dtype=np.uint8)
        frame = frame.reshape(480, 640, 3)
        cv2.imwrite(IMG_PATH + str(cam_ts) + ".jpg", frame)
        # create cam object for json
        cam_object = {
            "topic": topic,
            "sub_ts": sub_ts,
            "pub_ts": cam_ts,
            "frame_id": save_data.frame_id
        }
        if topic == RADAR_TOPIC:
            radar_object = readbuffer_process(payload, "s1")
        with open(RADCAM_PATH + ".json", "a") as f:
            if cam_object:
                json.dump(cam_object, f)
                f.write("\n")
            if radar_object:
                json.dump(radar_object, f)
                f.write("\n")
