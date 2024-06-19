import math
from pymavlink import mavutil, mavwp
import sys

import time
import serial
import struct
import time

from paho.mqtt import client as mqtt_client
import random

    # the_connection = mavutil.mavlink_connection('COM7', 57600) #telemetry
    # the_connection = mavutil.mavlink_connection('udpin:172.26.64.1:14550') #simulator
the_connection = mavutil.mavlink_connection('udpin:127.0.0.1:14570')
    #the_connection = mavutil.mavlink_connection('COM7', 57600)
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
        self.param2 = 2.0   #2.0
        self.param3 = 1.0  #1.0
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
    
def save_mission_to_file(filename):
    
    the_connection.mav.send('wp save %s\n' % filename)
    the_connection.mav.expect('Saved ([0-9]+) waypoints')
    num_wp = int(the_connection.mav.match.group(1))
    print("num_wp: %d" % num_wp)
    return True

if __name__ == "__main__":
    print("-- Program Started")


    takeoff_height = 0

    # Wait for the first heartbeat 
    while the_connection.target_system == 0:
        print("-- Checking for heartbeat")
        the_connection.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" % \
            (the_connection.target_system,
             the_connection.target_component))
    

    #save_mission_to_file("missions/{}.txt".format(mission))

    #Establish MQTT CONNECTION
    try:
        client = connect_mqtt()
        client.loop_start()
        mqttok = 1
    except ValueError:
        mqttok = 0


    mission_waypoints_ELEC = []
    mission_waypoints_ENVI = []
    mission_waypoints_CHEM = []
    mission_waypoints_INDE = []
    mission_waypoints_EN50 = []
    #route1 aris to ee ok
    mission_waypoints_ELEC.append(mission_item(0, 0, 16.47222011224263, 102.82388331297517, 0))
    mission_waypoints_ELEC.append(mission_item(1, 0, 16.472290069553516, 102.82388314023564, 0))
    mission_waypoints_ELEC.append(mission_item(2, 0, 16.47240432360487, 102.82388793261464, 0))
    mission_waypoints_ELEC.append(mission_item(3, 0, 16.472450021922622, 102.82394955561062, 0))
    mission_waypoints_ELEC.append(mission_item(4, 0, 16.472449101388673, 102.82426425593468, 0))
    mission_waypoints_ELEC.append(mission_item(5, 0, 16.472468783416055, 102.82455144321699, 0))
    mission_waypoints_ELEC.append(mission_item(6, 0, 16.472467087638595, 102.82480503131131, 0))
    mission_waypoints_ELEC.append(mission_item(7, 0, 16.472470924330477, 102.82506669779451, 0))
    mission_waypoints_ELEC.append(mission_item(8, 0, 16.472474088308513, 102.82522503425662, 0))
    mission_waypoints_ELEC.append(mission_item(9, 0, 16.472390476658244, 102.82528883727792, 0))
    mission_waypoints_ELEC.append(mission_item(10, 0, 16.472112470440585, 102.82530042973525, 0))
    mission_waypoints_ELEC.append(mission_item(11, 0, 16.471824947503936, 102.82529468411383, 0))

    mission_waypoints_ENVI.append(mission_item(0, 0, 16.471643820137576, 102.82527999073929, 0)) #12-16
    mission_waypoints_ENVI.append(mission_item(1, 0, 16.47146269277019, 102.8252652973922, 0))
    mission_waypoints_ENVI.append(mission_item(2, 0, 16.471396150104795, 102.82521823310611, 0))
    mission_waypoints_ENVI.append(mission_item(3, 0, 16.47136934955409, 102.82492203860397, 0))
    mission_waypoints_ENVI.append(mission_item(4, 0, 16.471361400549764, 102.82478303430099, 0))

    mission_waypoints_CHEM.append(mission_item(0, 0, 16.471347224112623, 102.82445275, 0)) #17-20
    mission_waypoints_CHEM.append(mission_item(1, 0, 16.47134185, 102.82407133, 0))
    mission_waypoints_CHEM.append(mission_item(2, 0, 16.471351045885537, 102.82377867428988, 0))

    mission_waypoints_INDE.append(mission_item(0, 0, 16.47138141954493, 102.82356037429952, 0))#21-27
    mission_waypoints_INDE.append(mission_item(1, 0, 16.47145458857858, 102.82338145394681, 0))
    mission_waypoints_INDE.append(mission_item(2, 0, 16.471557077976584, 102.82326861380943, 0))
    mission_waypoints_INDE.append(mission_item(3, 0, 16.471709781634743, 102.82316287680254, 0))
    mission_waypoints_INDE.append(mission_item(4, 0, 16.4718686573784, 102.82310359142605, 0))
    mission_waypoints_INDE.append(mission_item(5, 0, 16.4719927044229, 102.82308607672536, 0))
    mission_waypoints_INDE.append(mission_item(6, 0, 16.472204972317364, 102.82308943853127, 0))
    mission_waypoints_INDE.append(mission_item(7, 0, 16.472610013583548, 102.82307678768387, 0))

    mission_waypoints_EN50.append(mission_item(0, 0, 16.47288230294113, 102.82307097941276, 0))  #28-30
    mission_waypoints_EN50.append(mission_item(1, 0, 16.47309859469423, 102.82307789412681, 0))
    mission_waypoints_EN50.append(mission_item(2, 0, 16.47337209742737, 102.823072539414, 0))
    
    currentEndPoint=1
    while True:
    #LOAD MISSION ***********

        if currentEndPoint==1 :
            upload_mission(the_connection, mission_waypoints_ELEC)
            print("MISSION TO ELECTRICAL EN")
        elif currentEndPoint==2 :
            upload_mission(the_connection, mission_waypoints_ENVI)
            print("MISSION TO ENVIRONMENT EN")
        elif currentEndPoint==3 :
            upload_mission(the_connection, mission_waypoints_CHEM)
            print("MISSION TO CHEMICAL EN")
        elif currentEndPoint==4 :
            upload_mission(the_connection, mission_waypoints_INDE)
            print("MISSION TO INDUSTRAIL EN")
        elif currentEndPoint==5 :
            upload_mission(the_connection, mission_waypoints_EN50)
            print("MISSION TO EN50YEAR")

            

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

        currentmission=0
        start_mission(the_connection) 

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

                    print("Timeus: %d Throttle : %d  Stearing: %d mission: %d " % (ddata.get('time_usec'),out_throttle,out_stearing,currentmission))
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
                        gpsmsg = "lat," + str(gp_lat) + ",lon," + str(gp_lon)
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
                        currentmission=missionre.get('seq')
                        print("mission reach" + str(missionre.get('seq')))
                        #if missionre.get('seq') == endmissionseq:
                        #    break  #End sequence mission

        print("MISSION END WAIT FOR NEXT COMMAND")
        #the_connection.motors_disarmed_wait()
        set_flight_mode(the_connection, "MANUAL")
        #the_connection.waypoint_clear_all_send()
        print("Reset to Stabilize")
        currentEndPoint = currentEndPoint + 1
        if currentEndPoint >= 6:
            break
        
        time.sleep(60)
        #END MISSION  WAIT HERE
        
        








    while True:
        msg = the_connection.recv_match()
        
        if not msg:
            continue
        if msg:
            # print(msg)

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
        