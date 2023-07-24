from paho.mqtt import client as mqtt_client


broker = 'broker.emqx.io'
port = 1883
# username = 'emqx'
# password = 'public'

def connect_mqtt(client_id) -> mqtt_client:
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id)
    # client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client