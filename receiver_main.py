from processModule.serverConnect import connect_mqtt
import cv2
import numpy as np
import yaml
import subprocess

"""
PC client as subscriber of both device clients for logging received published data.
"""

with open("config.yaml", "r") as f:
    config = yaml.safe_load(f)

cv2.namedWindow(f"Frame from {config['mqtt']['client_id2']}", cv2.WINDOW_NORMAL)

def process_frames(frame_payload: bytearray) -> None:
    # convert byte array to numpy array for cv2 to read
    #frame = cv2.imdecode(np.frombuffer(frame_payload, np.uint8), -1)
    frame = np.frombuffer(frame_payload, dtype=np.uint8)
    sample_img = "data/sample_frame.png"
    data_array = cv2.imread(sample_img)
    height, width, channels = data_array.shape
    frame = frame.reshape(height, width, channels)
    cv2.imshow(f"Frame from {config['mqtt']['client_id2']}", frame)
    cv2.waitKey(0)

def subscribe(client, topic):
    def on_message(client, userdata, msg):
        if msg.topic == "data/camera/frame":
            print(f"Received {len(msg.payload)} bytes from topic {msg.topic}")
            process_frames(msg.payload)
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