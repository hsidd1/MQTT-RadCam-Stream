import cv2 
import numpy as np
import json 
import datetime as dt

"""
run as thread, writes live payload data: ./liveDataLog:
/camera_data: video frames as jpeg, filename=timestamp (publish)
/radcam_log.json: 
camera: {topic, sub_ts, pub_ts (matching filename in ./camera_data), frame_id}
 radar: {topic, sub_ts, pub_ts, radar_json (sensor id, x, y, z, tlv, tid, frame)} 
"""
IMG_PATH = "./liveDataLog/camera_data/"
RADCAM_PATH = "./liveDataLog/radcam_log.json"
SAMPLE_IMG = "./data/sample_frame.png"
data_array = cv2.imread(SAMPLE_IMG)
# print(data_array.shape) # (480, 640, 3)
CAMERA_TOPIC = "data/livecamera"
RADAR_TOPIC = "data/liveradar"

def save_data(topic: str, payload: bytes | str) -> dict | None:
    """
    saves live camera frames and radar data to ./liveDataLog/camera_data and ./liveDataLog/radcam_log.json
    """
    # function attribute to keep track of frame_id
    if not hasattr(save_data, "frame_id"):
        save_data.frame_id = 0

    cam_object = radar_object = None
    # save camera data frames
    sub_ts = str(dt.datetime.now().isoformat())
    if topic == CAMERA_TOPIC:
        save_data.frame_id += 1
        cam_ts = payload[-26:].decode("utf-8") # ts is 26 bytes extension
        frame_payload = payload[:-26] # remove timestamp
        frame = np.frombuffer(frame_payload, dtype=np.uint8)
        frame = frame.reshape(data_array.shape)
        ts_filename = cam_ts.replace(":", "-").replace(".", "-")
        cv2.imwrite(IMG_PATH + ts_filename + ".jpg", frame)
        # create cam object for json
        cam_object = {
            "topic": topic,
            "sub_ts": sub_ts,
            "pub_ts": cam_ts,
            "frame_id": save_data.frame_id
        }
    # save radar data
    if topic == RADAR_TOPIC:
        radar_object1 = {
            "topic": topic,
            "sub_ts": sub_ts,
        }
        try:
            radar_object2 = {"radar_payload": json.loads(payload.decode("utf-8"))}
        except json.JSONDecodeError:
            radar_object2 = {"radar_payload": payload.decode("utf-8")}
        radar_object = {**radar_object1, **radar_object2}
    
    with open(RADCAM_PATH, "a") as f:
        if f.tell() == 0:
            f.write("[")
        else:
            f.write(",")
        if cam_object:
            json.dump(cam_object, f)
        if radar_object:
            json.dump(radar_object, f)

"""
class CamPayload:
    def __init__(self, sub_ts, pub_ts, frame_id) -> None:
        self.topic = CAMERA_TOPIC
        self.sub_ts = sub_ts
        self.pub_ts = pub_ts
        self.frame_id = frame_id
    
    def __repr__(self) -> str:
        return f"CamPayload(topic={self.topic}, sub_ts={self.sub_ts}, pub_ts={self.pub_ts}, frame_id={self.frame_id})"
    
    def to_json(self) -> dict:
        return {
            "topic": self.topic,
            "sub_ts": self.sub_ts,
            "pub_ts": self.pub_ts,
            "frame_id": self.frame_id
        }

class RadarPayload:
    def __init__(self, sub_ts, pub_ts, radar_json) -> None:
        self.topic = RADAR_TOPIC
        self.sub_ts = sub_ts
        self.pub_ts = pub_ts
        self.radar_json = radar_json
    
    def __repr__(self) -> str:
        return f"RadarPayload(topic={self.topic}, sub_ts={self.sub_ts}, pub_ts={self.pub_ts}, radar_json={self.radar_json})"
    
    def to_json(self) -> dict:
        return {
            "topic": self.topic,
            "sub_ts": self.sub_ts,
            "pub_ts": self.pub_ts,
            "radar_json": self.radar_json
        }
"""