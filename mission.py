import math
from pymavlink import mavutil
import sys

import time
import serial
import struct


from paho.mqtt import client as mqtt_client
import random


#CONFIG MQTT
FIRST_RECONNECT_DELAY = 1
RECONNECT_RATE = 2
MAX_RECONNECT_COUNT = 12
MAX_RECONNECT_DELAY = 60

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

#CONFIG SERIAL LINT TO LOW LEVEL
SERIALPORT = "/dev/ttyUSB0"
BAUDRATE = 115200

ser = serial.Serial(SERIALPORT, BAUDRATE)


class mission_item:
    def __init__(self, l, current, x,y,z):
        self.seq = l
        self.frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        self.command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
        self.current = current
        self.auto = 1
        self.param1 = 0.0
        self.param2 = 2.00
        self.param3 = 20.00
        self.param4 = math.nan
        self.param5 = x
        self.param6 = y
        self.param7 = z
        self.mission_type = 0

def arm(the_connection):
    print("*-- Arming*")

    the_connection.mav.command_long_send(
            the_connection.target_system, 
            the_connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,1,0,0,0,0,0,0
            )
    
    ack(the_connection, "COMMAND_ACK")

def takeoff(the_connection, height):
    print("*-- Takeoff Initiated*")

    the_connection.mav.command_long_send(
            the_connection.target_system, 
            the_connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,0,0,0,math.nan,0,0,height)
    
    ack(the_connection, "COMMAND_ACK")

def upload_mission(the_connection, mission_items):
    n = len(mission_items)
    print("*-- Sending Message Out*")

    the_connection.mav.mission_count_send(
            the_connection.target_system, 
            the_connection.target_component,
            n,0)
    
    ack(the_connection, "MISSION_REQUEST")

    for waypoint in mission_items:
        print("*-- Creating a waypoint*")

        the_connection.mav.mission_item_send(
            the_connection.target_system, 
            the_connection.target_component,
            waypoint.seq,
            waypoint.frame,
            waypoint.command,
            waypoint.current,
            waypoint.auto,
            waypoint.param1,
            waypoint.param2,
            waypoint.param3,
            waypoint.param4,
            waypoint.param5,
            waypoint.param6,
            waypoint.param7,
            waypoint.mission_type
        )

    if waypoint != mission_items[n-1]:
        ack(the_connection, "MISSION_REQUEST")

    ack(the_connection, "MISSION_ACK")

def set_return(the_connection):
    print("*-- Set Return To Launch*")

    the_connection.mav.command_long_send(
            the_connection.target_system, 
            the_connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0,0,0,0,0,0,0,0)
    
    ack(the_connection, "COMMAND_ACK")

def start_mission(the_connection):
    print("*-- Mission Start*")

    the_connection.mav.command_long_send(
            the_connection.target_system, 
            the_connection.target_component,
            mavutil.mavlink.MAV_CMD_MISSION_START,
            0,0,0,0,0,0,0,0)
    
    ack(the_connection, "COMMAND_ACK")

def set_flight_mode(the_connection, fmode):
    mode = fmode

    # Check if mode is available
    if mode not in the_connection.mode_mapping():
        print('Unknown mode : {}'.format(mode))
        print('Try:', list(the_connection.mode_mapping().keys()))
        sys.exit(1)

    # Get mode ID
    mode_id = the_connection.mode_mapping()[mode]
    the_connection.mav.set_mode_send(
        the_connection.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)

def ack(the_connection, keyword):
    print("-- Message Read " + \
          str(the_connection.recv_match(type=keyword, blocking=True))
          )
    
if __name__ == "__main__":
    print("-- Program Started")

    # the_connection = mavutil.mavlink_connection('COM7', 57600) #telemetry
    # the_connection = mavutil.mavlink_connection('udpin:172.26.64.1:14550') #simulator
    the_connection = mavutil.mavlink_connection('udpin:127.0.0.1:14570')
    #the_connection = mavutil.mavlink_connection('COM7', 57600)
    takeoff_height = 15

    # Wait for the first heartbeat 
    while the_connection.target_system == 0:
        print("-- Checking for heartbeat")
        the_connection.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" % \
            (the_connection.target_system,
             the_connection.target_component))
    
    mission_waypoints = []

    mission_waypoints.append(mission_item(0, 0, 16.48267675, 102.81907061, 0)) #Current Home
    mission_waypoints.append(mission_item(1, 0, 16.48262578, 102.81902505, 0))
    mission_waypoints.append(mission_item(2, 0, 16.4825651, 102.81897949, 0))
    #mission_waypoints.append(mission_item(3, 0, 16.48254292, 102.81892569, 0))
    #mission_waypoints.append(mission_item(2, 0, 16.48251744, 102.81893328, 0))
    #mission_waypoints.append(mission_item(3, 0, 16.4824956, 102.81897377, 0))
    # mission_waypoints.append(mission_item(2, 0, 7.5183386, 4.5264412, 5))

    upload_mission(the_connection, mission_waypoints)

    arm(the_connection)
    

    the_connection.motors_armed_wait()

    set_flight_mode(the_connection, 'GUIDED')

    while True:
        msg = the_connection.recv_match(type ='HEARTBEAT', blocking = False)
        if msg:
            mode = mavutil.mode_string_v10(msg)
            print(mode)

            if "GUIDED" in mode:
                print("Taking off")
                takeoff(the_connection, 0)
                break

# *** START MISSION ****

    #Establish MQTT CONNECTION
    try:
        client = connect_mqtt()
        client.loop_start()
        mqttok = 1
    except ValueError:
        mqttok = 0

    start_mission(the_connection)

    # for mission_item in mission_waypoints:
    #     print("-- Message Read " + str(the_connection.recv_match(blocking=True)))
    while True:
        msg = the_connection.recv_match()
        
        if not msg:
            continue
        if msg:
            # print(msg)
            if msg.get_type() == 'MISSION_CURRENT':
                if msg.mission_state == 5:  #state==5 mission complete
                    #set_flight_mode(the_connection, "RTL")
                    break

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
            if msg.get_type() == 'GPS_RAW_INT':
                gpdata = msg.to_dict()
                gp_lat = int(gpdata.get('lat'))
                gp_lon = int(gpdata.get('lon'))
                if mqttok==1:
                    gpsmsg = "lat," + str(gp_lat) + "lon" + str(gp_lon)
                    #update to MQTT
                    result = client.publish(topic, gpsmsg)
                    # result: [0, 1]
                    status = result[0]
                    if status == 0:
                        print(f"Send `{gpsmsg}` to topic `{topic}`")
                    else:
                        print(f"Failed to send message to topic {topic}")
            if msg.get_type() == 'MISSION_ITEM_REACHED':
                    missionre = msg.to_dict()
                    #timestamp = master.time_since('GPS_RAW_INT')
                    print("mission reach" + str(missionre.get('seq')))

    the_connection.motors_disarmed_wait()

    set_flight_mode(the_connection, "MANUAL")

    print("Reset to Stabilize")



        