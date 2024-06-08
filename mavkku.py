
# Disable "Bare exception" warning
# pylint: disable=W0702

import time
import serial
import struct

from paho.mqtt import client as mqtt_client
import random

# Import mavutil
from pymavlink import mavutil


FIRST_RECONNECT_DELAY = 1
RECONNECT_RATE = 2
MAX_RECONNECT_COUNT = 12
MAX_RECONNECT_DELAY = 60
#broker = 'mqtt.kkiot.tech'
broker = 'www.kkiot.tech'
#61.91.50.18
port = 1884
topic = "aris/car"
client_id = f'python-mqtt-{random.randint(0, 1000)}'
username = 'tobb'
password = 'tb9918t'

def connect_mqtt():
    #def on_connect(client, userdata, flags, rc):
    # For paho-mqtt 2.0.0, you need to add the properties parameter.
    def on_connect(client, userdata, flags, rc, properties):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)
    # Set Connecting Client ID
    #client = mqtt_client.Client(client_id)

    # For paho-mqtt 2.0.0, you need to set callback_api_version.
    client = mqtt_client.Client(client_id=client_id, callback_api_version=mqtt_client.CallbackAPIVersion.VERSION2)
    #client = mqtt_client.Client(mqtt_client.CallbackAPIVersion.VERSION1, client_id)
    client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client

def on_disconnect(client, userdata, rc):
    logging.info("Disconnected with result code: %s", rc)
    reconnect_count, reconnect_delay = 0, FIRST_RECONNECT_DELAY
    while reconnect_count < MAX_RECONNECT_COUNT:
        logging.info("Reconnecting in %d seconds...", reconnect_delay)
        time.sleep(reconnect_delay)

        try:
            client.reconnect()
            logging.info("Reconnected successfully!")
            return
        except Exception as err:
            logging.error("%s. Reconnect failed. Retrying...", err)

        reconnect_delay *= RECONNECT_RATE
        reconnect_delay = min(reconnect_delay, MAX_RECONNECT_DELAY)
        reconnect_count += 1
    logging.info("Reconnect failed after %s attempts. Exiting...", reconnect_count)

def publish(client):
    msg_count = 1
    while True:
        time.sleep(1)
        msg = f"messages: {msg_count}"
        result = client.publish(topic, msg)
        # result: [0, 1]
        status = result[0]
        if status == 0:
            print(f"Send `{msg}` to topic `{topic}`")
        else:
            print(f"Failed to send message to topic {topic}")
        msg_count += 1
        if msg_count > 5:
            break


def run():
    client = connect_mqtt()
    client.loop_start()
    publish(client)
    client.loop_stop()


if __name__ == '__main__':
    while True:
        run()


SERIALPORT = "/dev/ttyUSB0"
BAUDRATE = 115200

ser = serial.Serial(SERIALPORT, BAUDRATE)

# Create the connection
#  If using a companion computer
#  the default connection is available
#  at ip 192.168.2.1 and the port 14550
# Note: The connection is done with 'udpin' and not 'udpout'.
#  You can check in http:192.168.2.2:2770/mavproxy that the communication made for 14550
#  uses a 'udpbcast' (client) and not 'udpin' (server).
#  If you want to use QGroundControl in parallel with your python script,
#  it's possible to add a new output port in http:192.168.2.2:2770/mavproxy as a new line.
#  E.g: --out udpbcast:192.168.2.255:yourport
# mavproxy.py --master=/dev/ttyACM1 --out 127.0.0.1:14550
master = mavutil.mavlink_connection('udpin:127.0.0.1:14570')





# Make sure the connection is valid SERVO_OUTPUT_RAW
master.wait_heartbeat()

while True:
    msg = master.recv_match()
    if not msg:
        continue
    if msg.get_type() == 'SERVO_OUTPUT_RAW':
        ddata = msg.to_dict()
        out_throttle = int(ddata.get('servo3_raw'))
        out_stearing = int(ddata.get('servo1_raw'))
        #if out_throttle > 1855:
        #    out_throttle=1855

        print("Timeus: %d Throttle : %d  Stearing: %d " % (ddata.get('time_usec'),out_throttle,out_stearing))
        ser.write(bytes([0xFF]))
        ser.write(bytes([0xFE]))
        ser.write(bytes([0x08]))
        ser.write(bytes([0x00]))
        ser.write(bytes([0x00]))#CHKSUM
        ser.write(bytes([0x01]))
        ser.write(bytes([0x00]))#TOPIC ID
        ser.write(struct.pack('<h',out_throttle))
        ser.write(struct.pack('<h',out_stearing))
        ser.write(struct.pack('<h',1234))
        ser.write(struct.pack('<h',5678))
        #print(struct.pack('<h',out_throttle)) 
        #print(struct.pack('<h',out_stearing)) 
        ser.write(bytes([0x08]))#CHKSUM
        






        #print("\n\n*****Got message: %s*****" % msg.get_type())
        #print("Message: %s" % msg)
        #print("\nAs dictionary: %s" % msg.to_dict())
        # Armed = MAV_STATE_STANDBY (4), Disarmed = MAV_STATE_ACTIVE (3)




# Get some information !
while True:
    try:
        print(master.recv_match().to_dict())
    except:
        pass
    time.sleep(0.1)