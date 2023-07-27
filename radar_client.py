from processModule.rd_process import data
import yaml
import time
from processModule.serverConnect import connect_mqtt

with open("config.yaml", "r") as f:
    config = yaml.safe_load(f)
CLIENT_ID = config["mqtt"]["client_id1"]

def on_log(client, userdata, level, buf):
    print("log: ",buf)
# client.on_log=on_log
def on_message(client, userdata, message):
    print("message received " ,str(message.payload.decode("utf-8")))
    print("message topic=",message.topic)
    print("message qos=",message.qos)
    print("message retain flag=",message.retain)

def publish(client, data=data):
    def on_message(client, userdata, message):
        print("message received " ,str(message.payload.decode("utf-8")))
        print("message topic=",message.topic)
        print("message qos=",message.qos)
        print("message retain flag=",message.retain)
    if config["mqtt"]["show_log"]:
        client.on_message=on_message
    while True:
        for i in range(0, len(data), 10):
            time.sleep(4)
            msg = data[i:i+10]  # modify to publish based on timestamps for intervals
            data = data[i+10:]
            if not msg:
                break # stop once msg becomes empty list 
            # for demo/debug: only send 2 objects from the data slice
            if config["mqtt"]["compressed_output"]:
                msg = msg[:2]
            res = client.publish(topic="data/radar", payload=str(msg), qos=0)
            status = res[0]
            if status == 0:
                print(f"{CLIENT_ID}: Send `{msg}` to topic `data/radar`\n")
            else:
                print(f"{CLIENT_ID}: Failed to send radar message to topic data/radar")
        if not data:
            break

def run():
    try:
        client = connect_mqtt(CLIENT_ID) 
        client.loop_start()
        publish(client)
        client.loop_stop()
    except KeyboardInterrupt:     
        print("Exiting Radar Client...")
        client.disconnect()

if __name__ == "__main__":
    run()