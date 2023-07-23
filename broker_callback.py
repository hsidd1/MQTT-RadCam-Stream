import paho.mqtt.client as mqtt
import time
import yaml
import subprocess

def on_message(client, userdata, message):
    print("Received message: ", str(message.payload.decode("utf-8")))

with open("config.yaml", "r") as f:
    config = yaml.safe_load(f)

client = mqtt.Client("PC")
client.connect(config["mqtt"]["broker"])

def main():
    client.on_message = on_message
    client.loop_start()
    subprocess.call(["python", "radar_client.py"])
    subprocess.call(["python", "cam_client.py"])
    #client.subscribe("data") # subscribes to both radar and cam
    client.subscribe("data/radar")
    client.subscribe("data/camera")
    time.sleep(30)
    client.loop_stop()

if __name__ == "__main__":
    main()