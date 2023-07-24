import paho.mqtt.client as mqtt
import time
import yaml
import subprocess
from processModule.serverConnect import connect_mqtt
"""
PC client as subscriber of both device clients for logging received published data.
"""

def on_message(client, userdata, message):
    print("Received message: ", str(message.payload.decode("utf-8")))

with open("config.yaml", "r") as f:
    config = yaml.safe_load(f)

#client = mqtt.Client("PC")
#client.connect(config["mqtt"]["broker"])

def subscribe(client, topic):
    def on_message(client, userdata, msg):
        print(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")

    client.subscribe(topic)
    client.on_message = on_message

def main():
    # subprocess.run(["python", "radar_client.py"])
    # subprocess.run(["python", "cam_client.py"])
    #client.subscribe("data") # subscribes to both radar and cam

    client = connect_mqtt("PC")
    subscribe(client, topic = "data/radar")
    subscribe(client, topic = "data/camera/frame")
    subscribe(client, topic = "data/camera/ts")
    client.loop_forever()
    #time.sleep(1)

if __name__ == "__main__":
    main()