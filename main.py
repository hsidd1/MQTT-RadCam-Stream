from processModule.serverConnect import connect_mqtt
import cv2
import os
import sys
import datetime as dt
import yaml
import subprocess
from processModule.cam_process import process_frames
import traceback

"""
Receiver client as subscriber of both device clients for logging and processing
received published data. Runs subprocesses in parallel automatically.
"""

with open("config.yaml", "r") as f:
    config = yaml.safe_load(f)

LOG_FILE = sys.argv[1] if len(sys.argv) > 1 else "log.txt"

if os.path.exists(LOG_FILE):
    # if the file already exists, create a separate file with a unique name - avoid overwriting
    base, ext = os.path.splitext(LOG_FILE)
    idx = 1
    while os.path.exists(f"{base}_{idx}{ext}"):
        idx += 1
    log_file_path = f"{base}_{idx}{ext}"

frame_payload = "None"
ts_pub = "None"
def subscribe(client, topic):
    def on_message(client, userdata, msg):
        global frame_payload, ts_pub
        if msg.topic == "data/camera/frame":
            print(f"Received {len(msg.payload)} bytes from topic {msg.topic}")
            frame_payload = msg.payload
            # process_frames(msg.payload) # display frames in cv2 window
            # msg_decode = str(bytearray(msg.payload))
        else:
            print(f"Received {msg.payload.decode()} from topic {msg.topic}")
            # msg_decode = msg.payload.decode()
            ts_pub = msg.payload.decode()
        if frame_payload != "None" and ts_pub != "None":
            process_frames(frame_payload, ts_pub)
        # if config["write_log"] and msg_decode != "None":
        #     with open(LOG_FILE, "a") as f:
        #         f.write(f"{dt.datetime.now().isoformat()}: Received {msg_decode} from {msg.topic}\n")

    client.subscribe(topic)
    client.on_message = on_message

def main():
    camera_process = subprocess.Popen(["python", "cam_client.py"])
    radar_process = subprocess.Popen(["python", "radar_client.py"])
    client = connect_mqtt("PC")
    subscribe(client, topic = "data/radar")
    subscribe(client, topic = "data/camera/frame")
    #subscribe(client, topic = "data/camera/ts")

    def exit_handler(client):
        camera_process.kill()
        print("Camera process killed.") 
        radar_process.kill()
        print("Radar process killed.") # RIP my friends
        client.disconnect()
        cv2.destroyAllWindows()
    try:
        client.loop_forever()
    except KeyboardInterrupt:
        print("Process Terminated. Exiting Receiver...")
        exit_handler(client)
    except Exception as e:
        print("Something went wrong. Exiting Receiver...")
        #print(e)
        traceback.print_exc()
        exit_handler(client)

if __name__ == "__main__":
    main()