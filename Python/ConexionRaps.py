import network
import json
from time import sleep
from umqtt.robust import MQTTClient

SSID = "PipeG"
PWD  = "12345678"

def wifi_connect():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(SSID, PWD)
    while not wlan.isconnected():
        sleep(1)
    print ("conexion a wifi")

def message(topic, msg):
    print(msg.decode())

client = MQTTClient("DEF", "192.168.130.108")
client.set_callback(message)

wifi_connect()
client.connect()
client.subscribe(b"test/topic")

while True:
    client.wait_msg()
    data = json.dumps({"distance":30})
    client.publish(b"rpi/topic", data)
