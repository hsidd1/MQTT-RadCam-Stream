from processModule.serverConnect import connect_mqtt
import cv2
import yaml
import subprocess
from processModule.cam_process import process_frames

"""
Receiver client as subscriber of both device clients for logging and processing
received published data. Runs subprocesses in parallel automatically.
"""

with open("config.yaml", "r") as f:
    config = yaml.safe_load(f)

def subscribe(client, topic):
    def on_message(client, userdata, msg):
        if msg.topic == "data/camera/frame":
            print(f"Received {len(msg.payload)} bytes from topic {msg.topic}")
            process_frames(msg.payload) # display frames in cv2 window
        else:
            print(f"Received {msg.payload.decode()} from topic {msg.topic}")
    client.subscribe(topic)
    client.on_message = on_message

def main():
    camera_process = subprocess.Popen(["python", "cam_client.py"])
    radar_process = subprocess.Popen(["python", "radar_client.py"])
    client = connect_mqtt("PC")
    subscribe(client, topic = "data/radar")
    subscribe(client, topic = "data/camera/frame")
    subscribe(client, topic = "data/camera/ts")

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
        print(e)
        exit_handler(client)

if __name__ == "__main__":
    main()