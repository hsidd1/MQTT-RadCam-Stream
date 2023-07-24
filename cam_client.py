import paho.mqtt.client as mqtt
import cv2
import yaml
import datetime as dt
import time 
from processModule.serverConnect import connect_mqtt

with open("config.yaml", "r") as f:
    config = yaml.safe_load(f)
CLIENT_ID = config["mqtt"]["client_id2"]

def on_log(client, userdata, level, buf):
    print("log: ",buf)
# client.on_log=on_log
def on_message(client, userdata, message):
    print("message received " ,str(message.payload.decode("utf-8")))
    print("message topic=",message.topic)
    print("message qos=",message.qos)
    print("message retain flag=",message.retain)

def publish(client):
    frame_id = 0
    while True:
        ret, frame = cap.read()
        time.sleep(3)
        if not ret:
            break
        frame_id += 1
        # for demo/debug: only send first 2x2 pixels of frame, else take actual array
        if config["mqtt"]["compressed_output"]:
            frame = frame[:2, :2]
        msg = f"f_id {frame_id}: {frame}"
        topic="data/camera/frame"
        res = client.publish(topic, payload=msg, qos=0) # QoS 0 for frames
        status = res[0]
        if status == 0:
            print(f"{CLIENT_ID}: Send `{msg}` to topic `{topic}`\n")
        else:
            print(f"{CLIENT_ID}: Failed to send frame message to topic {topic}")
        timestamp = dt.datetime.now().isoformat()
        topic="data/camera/ts"
        res = client.publish(topic, payload=timestamp, qos=1) # QoS 1 for timestamps
        status = res[0]
        if status == 0:
            print(f"{CLIENT_ID}: Send `{timestamp}` to topic `{topic}`\n")
        else:
            print(f"{CLIENT_ID}: Failed to send timestamp message to topic {topic}")

cap = cv2.VideoCapture(config["Files"]["video_file"])

def run():
    client = connect_mqtt(CLIENT_ID) 
    if config["mqtt"]["show_log"]:
        client.on_message=on_message
    client.loop_start()
    publish(client)
    client.loop_stop()
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    run()