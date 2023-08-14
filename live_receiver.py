from processModule.serverConnect import connect_mqtt
import yaml
import subprocess
from processModule.camera_process import process_livecam
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
        if msg.topic == RADAR_TOPIC:
            print(f"Received {len(msg.payload)} bytes from topic {msg.topic}")
        if msg.topic == CAMERA_TOPIC:
            print(f"Received {len(msg.payload)} bytes from topic {msg.topic}")
            process_livecam(msg.payload) # display frames in cv2 window w/ timestamp
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
    try:
        client.loop_forever()
    except KeyboardInterrupt:
        print("Process Terminated. Exiting Receiver...")
        exit_handler(client)
    except Exception as e:
        print("Something went wrong. Exiting Receiver...")
        print(e)
        exit_handler(client)

if __name__ == "__main__":
    main()