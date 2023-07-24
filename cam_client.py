import paho.mqtt.client as mqtt
import cv2
import yaml
import datetime as dt
import time 
from processModule.serverConnect import connect_mqtt

with open("config.yaml", "r") as f:
    config = yaml.safe_load(f)

#client = mqtt.Client(config["mqtt"]["client_id2"]) # camera
#client.connect(config["mqtt"]["broker"]) 
#client = connect_mqtt(config["mqtt"]["client_id2"]) # emqx web server
#client.subscribe("data/radar") 

def on_log(client, userdata, level, buf):
    print("log: ",buf)
# client.on_log=on_log
def on_message(client, userdata, message):
    print("message received " ,str(message.payload.decode("utf-8")))
    print("message topic=",message.topic)
    print("message qos=",message.qos)
    print("message retain flag=",message.retain)

def publish(client):
    while True:
        ret, frame = cap.read()
        time.sleep(1)
        if not ret:
            break
        msg = frame.tobytes()
        msg = str(msg)
        msg = msg[0:10] # gibberish rn so just send 10 bytes
        topic="data/camera/frame"
        res = client.publish(topic, payload=msg, qos=0) # QoS 0 for frames
        status = res[0]
        if status == 0:
            print(f"Send `{msg}` to topic `{topic}`")
        else:
            print(f"Failed to send frame message to topic {topic}")
        timestamp = dt.datetime.now().isoformat()
        topic="data/camera/ts"
        res = client.publish(topic, payload=timestamp, qos=1) # QoS 1 for timestamps
        status = res[0]
        if status == 0:
            print(f"Send `{timestamp}` to topic `{topic}`")
        else:
            print(f"Failed to send timestamp message to topic {topic}")

cap = cv2.VideoCapture(config["Files"]["video_file"])

def run():
    client = connect_mqtt(config["mqtt"]["client_id2"]) 
    if config["mqtt"]["show_log"]:
        client.on_message=on_message
    client.loop_start()
    publish(client)
    client.loop_stop()
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    run()