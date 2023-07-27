from processModule.serverConnect import connect_mqtt
import cv2
import time
import yaml
import subprocess

"""
PC client as subscriber of both device clients for logging received published data.
"""

with open("config.yaml", "r") as f:
    config = yaml.safe_load(f)

def process_frames(video_src) -> None:
    cap = cv2.VideoCapture(video_src)
    frame_id = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        frame_id += 1
        cv2.imshow(f"frame: {frame_id} (ESC to exit)", frame)
        if not config["CameraOutput"]["continuous_frame_mode"]:
            # wait until the user closes the window or presses ESC
            key = cv2.waitKey(0) & 0xFF
            if key == 27:
                break
            fps = config["CameraOutput"]["fps"]
            frame_time = 1/fps
            time.sleep(frame_time)
    cv2.destroyAllWindows()

def subscribe(client, topic):
    def on_message(client, userdata, msg):
        if msg.topic == "data/camera/frame":
            print(f"Received {len(msg.payload)} bytes from topic {msg.topic}")
            process_frames(config["Files"]["video_file"])
        else:
            print(f"Received {msg.payload.decode()} from topic {msg.topic}")
    client.subscribe(topic)
    client.on_message = on_message

def main():
    subprocess.Popen(["python", "cam_client.py"])
    subprocess.Popen(["python", "radar_client.py"])
    client = connect_mqtt("PC")
    subscribe(client, topic = "data/radar")
    subscribe(client, topic = "data/camera/frame")
    subscribe(client, topic = "data/camera/ts")
    try:
        client.loop_forever()
    except KeyboardInterrupt:
        print("Exiting...")
        client.disconnect()

if __name__ == "__main__":
    main()