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

def on_message(client, userdata, message):
    if message.topic == "data/camera/frame":
        print(f"Received {len(message.payload)} bytes from topic {message.topic}")
    else:
     print("message received " ,str(message.payload.decode("utf-8")))
    print("message topic=",message.topic)
    print("message qos=",message.qos)
    print("message retain flag=",message.retain)

def publish(client): 
    frame_id = 0
    i = 0
    frame_start = config["CameraOutput"]["frame_start"]
    frame_skip = config["CameraOutput"]["frame_skip"]
    while True:
        ret, frame = cap.read()
        # first couple of 'frames' are not even frames based on my observations, so skip
        if i < frame_start:
            i += 1
            continue
        frame_id += 1
        if frame_id % frame_skip != 0:
            i+=1 
            continue
    
        if not ret:
            break
        
        msg_str = f"f_id {frame_id}: {frame}"
        topic="data/camera/frame"
        msg = bytearray(frame)
        res = client.publish(topic, payload=msg, qos=0) # QoS 0 for frames
        status = res[0]
        if status == 0:
            if config["mqtt"]["compressed_output"]:
                print(f"{CLIENT_ID}: Send `{msg[:10]}` to topic `{topic} (fid={frame_id})`\n")
            else:
                print(f"{CLIENT_ID}: Send `{msg}` to topic `{topic} (fid={frame_id})`\n")    
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
        time.sleep(3)
        i+=1

cap = cv2.VideoCapture(config["Files"]["video_file"])

def run():
    try:
        client = connect_mqtt(CLIENT_ID) 
        def exit_handler(client):
            cap.release()
            client.disconnect()
            cv2.destroyAllWindows()

        if config["mqtt"]["show_log"]:
            client.on_message=on_message
            # client.on_log=on_log

        client.loop_start()
        publish(client)
        exit_handler(client)
    except KeyboardInterrupt:
        print("Camera Process Terminated. Exiting Camera Client...")
        exit_handler(client)
    except Exception as e:
        print("Something went wrong. Exiting Camera Client...")
        print(e)
        exit_handler(client)

if __name__ == '__main__':
    run()