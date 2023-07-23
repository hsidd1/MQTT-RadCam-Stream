import paho.mqtt.client as mqtt
import cv2
import yaml
import datetime as dt

with open("config.yaml", "r") as f:
    config = yaml.safe_load(f)

client = mqtt.Client(config["mqtt"]["client_id2"]) # camera
client.connect(config["mqtt"]["broker"]) # web server 
# client.subscribe("data/radar") 

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

cap = cv2.VideoCapture(config["Files"]["video_file"])

client.loop_start()
while True:
    ret, frame = cap.read()
    if not ret:
        break
    msg = frame.tobytes()
    client.publish(topic="data/camera/frame", payload=msg, qos=0) # QoS 0 for frames
    timestamp = dt.datetime.now().isoformat()
    client.publish(topic="data/camera/ts", payload=timestamp, qos=1) # QoS 1 for timestamps
client.loop_stop()
cap.release()
cv2.destroyAllWindows()