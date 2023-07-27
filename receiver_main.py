from processModule.serverConnect import connect_mqtt
import yaml
import subprocess

"""
PC client as subscriber of both device clients for logging received published data.
"""

def on_message(client, userdata, message):
    print("Received message: ", str(message.payload.decode("utf-8")))

with open("config.yaml", "r") as f:
    config = yaml.safe_load(f)

def subscribe(client, topic):
    def on_message(client, userdata, msg):
        print(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")

    client.subscribe(topic)
    client.on_message = on_message

def main():
    subprocess.Popen(["python", "cam_client.py"])
    subprocess.Popen(["python", "radar_client.py"])
    client = connect_mqtt("PC")
    subscribe(client, topic = "data/radar")
    subscribe(client, topic = "data/camera/frame")
    subscribe(client, topic = "data/camera/ts")
    client.loop_forever()

if __name__ == "__main__":
    main()