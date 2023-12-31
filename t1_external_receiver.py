from processModule.serverConnect import connect_mqtt
import cv2
import yaml
from processModule.camera_process import process_frames

"""
Receiver client as subscriber of both device clients for logging received published data.
Strictly runs receiver client only, so device clients can be run externally or separately.
"""

with open("config.yaml", "r") as f:
    config = yaml.safe_load(f)

cv2.namedWindow(f"Frame from {config['mqtt']['client_id2']}", cv2.WINDOW_NORMAL)
SAMPLE_IMG = "data/sample_frame.png"
data_array = cv2.imread(SAMPLE_IMG)


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
    client = connect_mqtt("PC")
    subscribe(client, topic="data/radar")
    subscribe(client, topic="data/camera/frame")
    subscribe(client, topic="data/camera/ts")

    def exit_handler(client):
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
