import paho.mqtt.client as mqtt
import yaml
from processModule.rd_process import data
import time
from processModule.serverConnect import connect_mqtt

with open("config.yaml", "r") as f:
    config = yaml.safe_load(f)

#client = mqtt.Client(config["mqtt"]["client_id1"]) # radar
#client.connect(config["mqtt"]["broker"])
client = connect_mqtt(config["mqtt"]["client_id1"]) # emqx web server

# client.subscribe("data/camera") # receive frame data from cam topic

def on_log(client, userdata, level, buf):
    print("log: ",buf)
# client.on_log=on_log
def on_message(client, userdata, message):
    print("message received " ,str(message.payload.decode("utf-8")))
    print("message topic=",message.topic)
    print("message qos=",message.qos)
    print("message retain flag=",message.retain)
if config["mqtt"]["show_log"]:
    client.on_message=on_message

client.loop_start()
while data:
    for i in range(0, len(data), 10):
        msg = data[i:i+10]  # modify to publish based on timestamps for intervals
        data = data[i+10:]
        client.publish(topic="data/radar", payload=str(msg), qos=0)
        time.sleep(3)
client.loop_stop()