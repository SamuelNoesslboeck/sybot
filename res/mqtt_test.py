# python3.6

import random
import struct

from paho.mqtt import client as mqtt_client


broker = 'syhub'
port = 1883
topic = "pos/phis"
# generate client ID with pub prefix randomly
client_id = f'python-mqtt-{random.randint(0, 100)}'
username = "test"
password = "test"


def connect_mqtt() -> mqtt_client:
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print(f"Failed to connect, return code {rc}\n")

    client = mqtt_client.Client(client_id)
    client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)

    return client


def subscribe(client: mqtt_client):
    def on_message(client, userdata, msg):
        print(f"Received `{struct.unpack('4f', msg.payload)}` from `{msg.topic}` topic")

    client.subscribe(topic)
    client.on_message = on_message


def run():
    print("Trying to connect ... ")
    client = connect_mqtt()
    subscribe(client)
    client.loop_forever()


if __name__ == '__main__':
    run()
