from processModule.serverConnect import connect_mqtt
from visualizationModule.visualization_main import run_visualization
from threading import Thread
import yaml
import json
import traceback
import subprocess
from processModule.camera_process import process_livecam
from processModule.save_data import save_data
"""
Receiver client for live radar and camera data. Requires radar to be connected. 
This program receives and processes live radar and camera data from the device clients.
Note: Runs subprocess for radar and camera automatically. UART COM ports config
must be set in config.yaml for Windows. Requires external camera connection.
"""

with open("config.yaml", "r") as f:
    config = yaml.safe_load(f)

RADAR_ID = config["LiveData"]["radar"]["client_id"]
RADAR_TOPIC = config["LiveData"]["radar"]["topic"]

CAMERA_ID = "LIVE CAMERA"
CAMERA_TOPIC = "data/livecamera"

def subscribe(client, topic):
    def on_message(client, userdata, msg):
        t = Thread(target=save_data, args=(msg.topic, msg.payload))
        t.start()
        cam_payload = radar_payload = None
        if msg.topic == RADAR_TOPIC:
            #process_radar(msg.payload)
            radar_payload = msg.payload
            print(f"Received {len(msg.payload)} bytes from topic {msg.topic}\n\n")
        if msg.topic == CAMERA_TOPIC:
            cam_payload = msg.payload
            print(f"Received {len(msg.payload)} bytes from topic {msg.topic}\n\n")
            # process_livecam(msg.payload) # display frames in cv2 window w/ timestamp
        try:
            #run_visualization(cam_payload, radar_payload)
            pass
        # do nothing if payload is None
        except TypeError:
            pass
    client.subscribe(topic)
    client.on_message = on_message

def main():
    live_radar_process = subprocess.Popen(["python", "live_radarclient.py"])
    live_cam_process = subprocess.Popen(["python", "live_cameraclient.py"])
    client = connect_mqtt("PC")
    subscribe(client, topic = RADAR_TOPIC)
    subscribe(client, topic = CAMERA_TOPIC)

    def exit_handler(client):
        live_radar_process.kill()
        print("Live radar process killed.") 
        live_cam_process.kill()
        print("Live camera process killed.")
        client.disconnect()
        with open("liveDataLog/radcam_log.json", "r") as f:
            data = f.read()
        try:
            json_data = json.loads(data)
        except json.JSONDecodeError:
            data = data.rstrip()
            if data.startswith("[") and not data.endswith("]"):
                data += "]"
            data = data.replace("],{", ",{")
            with open("liveDataLog/radcam_log.json", "w") as f:
                f.write(data)

    try:
        client.loop_forever()
    except KeyboardInterrupt:
        print("RECEIVER: Process Terminated. Exiting Receiver...")
        exit_handler(client)
    except Exception as e:
        print("RECEIVER: Something went wrong. Exiting Receiver...")
        print(traceback.format_exc())
        exit_handler(client)

if __name__ == "__main__":
    main()