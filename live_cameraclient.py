import cv2
from processModule.serverConnect import connect_mqtt
import yaml
import datetime as dt
import traceback
import time

"""
Live client for camera device. Publishes frames and corresponding timestamps
to topic data/livecamera.
Note: This program runs as a subprocess of live_receiver.py.
"""
with open("config.yaml", "r") as f:
    config = yaml.safe_load(f)
cap = None


def publish(client):
    def on_message(client, userdata, message):
        print("message received ", str(message.payload.decode("utf-8")))
        print("message topic=", message.topic)
        print("message qos=", message.qos)
        print("message retain flag=", message.retain)

    def on_log(client, userdata, level, buf):
        print("log: ", buf)

    if config["mqtt"]["show_log"]:
        client.on_message = on_message

    global cap
    cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)  # external camera
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, config["LiveData"]["camera"]["width"])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config["LiveData"]["camera"]["height"])
    cap.set(cv2.CAP_PROP_FPS, config["LiveData"]["camera"]["fps"])
    client = connect_mqtt("Camera")
    # num_frames = 120
    # start = time.perf_counter()
    # i = 0
    while True:
        # if i == num_frames:
        #    break
        try:
            ret, frame = cap.read()
            if ret:
                """
                Send frame and timestamp within same payload to topic data/livecamera
                """
                payload = bytearray(frame)
                # extend payload with timestamp: 26 bytes
                payload.extend(bytearray(str(dt.datetime.now().isoformat()), "utf-8"))

                # print(f"Timestamp: {str(dt.datetime.now().isoformat())}")
                # print(len(bytearray(str(dt.datetime.now().isoformat()), "utf-8"))) # 26 bytes
                res = client.publish("data/livecamera", payload=payload, qos=0)
                status = res[0]
                if status == 0:
                    print(f"Send {len(payload)} bytes to topic data/livecamera")
                    #i += 1
            else:
                print("No frame received.")
        except KeyboardInterrupt:
            print("LIVE CAMERA: Process Terminated. Exiting Camera...")
            break
    #print(f"FPS is {num_frames/(time.perf_counter()-start)}")

def main():
    try:
        client = connect_mqtt("LIVE CAMERA")
        client.loop_start()
        publish(client)
        client.loop_stop()
    except KeyboardInterrupt:
        print("LIVE CAMERA: Process Terminated. Exiting Camera...")
        cap.release()
        cv2.destroyAllWindows()
        client.disconnect()
    except Exception as e:
        print("LIVE CAMERA: Error Occured. Exiting Camera...")
        cap.release()
        cv2.destroyAllWindows()
        client.disconnect()
        print(traceback.format_exc())
    if cap.isOpened():
        cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
