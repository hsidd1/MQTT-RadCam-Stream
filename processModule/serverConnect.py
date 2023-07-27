from paho.mqtt import client as mqtt_client


broker = 'broker.emqx.io'
port = 1883

def connect_mqtt(client_id) -> mqtt_client:
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print(f"{client_id}: Connected to MQTT Broker!")
        else:
            print(client_id, "failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id)
    # client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.connect(broker, port)
    return client

def on_disconnect(client, userdata, rc):
    if rc != 0:
        print(f"Unexpected MQTT disconnection. Will auto-reconnect (rc={rc})")