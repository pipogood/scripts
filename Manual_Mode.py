#!/usr/bin/env python
# -*- coding: utf-8 -*-
import json
import os
import numpy
import math
import time
import rospy
from std_msgs.msg import String
import paho.mqtt.client as mqtt

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *  # Uses Dynamixel SDK library

####################################################################################################################################################


# RAM REGISTER ADDRESSES - resets after shut down
ADDR_MX_TORQUE_ENABLE          = 24
ADDR_MX_LED                    = 25
ADDR_MX_D_GAIN                 = 26
ADDR_MX_I_GAIN                 = 27
ADDR_MX_P_GAIN                 = 28
ADDR_MX_GOAL_POSITION          = 30
ADDR_MX_MOVING_SPEED           = 32
ADDR_MX_TORQUE_LIMIT           = 34
ADDR_MX_PRESENT_POSITION       = 36
ADDR_MX_PRESENT_SPEED          = 38
ADDR_MX_PRESENT_LOAD           = 40
ADDR_MX_PRESENT_VOLTAGE        = 42
ADDR_MX_PRESENT_TEMPERATURE    = 43
ADDR_MX_REGISTERED             = 44
ADDR_MX_MOVING                 = 46
ADDR_MX_LOCK                   = 47
ADDR_MX_PUNCH_L                = 48
ADDR_REALTIME_TICK             = 50
ADDR_CURRENT                   = 68
ADDR_TORQUE_CTRL_MODE_ENABLE   = 70
ADDR_GOAL_TORQUE               = 71
ADDR_GOAL_ACCELERATION         = 73

# Protocol version
PROTOCOL_VERSION               = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL1_ID                        = 1                 # Dynamixel#1 ID : 1
DXL2_ID                        = 2                 # Dynamixel#2 ID : 2
DXL3_ID                        = 3                 # Dynamixel#3 ID : 3
DXL4_ID                        = 4                 # Dynamixel#4 ID : 4
BAUDRATE                       = 57600             # Dynamixel default baudrate : 57600
DEVICENAME                     = '/dev/ttyUSB1'    # Port connected to controller
TORQUE_ENABLE                  = 1                 # Value for enabling the torque
TORQUE_DISABLE                 = 0                 # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD    = 20                # Dynamixel moving status threshold

# Defined Parameter
DEBUG                          = True
DXL_MINIMUM_SPEED_VALUE        = 0           # Dynamixel will rotate between this value
DXL_MAXIMUM_SPEED_VALUE        = 1023            # and this value (note that the Dynamixel would not
DXL_GOAL_SPEED                 = [DXL_MINIMUM_SPEED_VALUE, DXL_MAXIMUM_SPEED_VALUE]

# Data Byte Length
LEN_MX_MOVING_SPEED            = 4
LEN_MX_PRESENT_SPEED           = 4

DXL_CW_ANGLE_TO_Z              = 0
DXL_CCW_ANGLETO_Z              = 0

q1 = ''
q2 = ''
q3 = ''
q4 = ''
q5 = ''
q6 = ''
q7 = ''
q8 = ''
mes = ''
Wheel_stop_left = 0
Wheel_stop_right = 0
Wheel_stop_up = 0
Wheel_stop_down = 0
Time = 0
dt = 0.1
Flag_OK = 1
Flag_CH = 1
count = 0

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_MX_MOVING_SPEED, LEN_MX_MOVING_SPEED )
MQTT_topic = [("unity/mobot/mobility",0),("Mobot/stop",0),("Mobot/shutdown",0)]

###################################################################################################################################################

if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

for id in range(1, 5):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d torque has been enable" % id)
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, id, ADDR_MX_CW_ANGLE_LIMIT, DXL_CW_ANGLE_TO_Z)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, id, ADDR_MX_CCW_ANGLE_LIMIT, DXL_CCW_ANGLETO_Z)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully set to wheel mode" % id)

def Feedback():
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

def WriteDXL_Feedback(W1,W2,W3,W4):
    client.publish("mobility/debug","Moving",2)
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 1, ADDR_MX_MOVING_SPEED, W1)
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 2, ADDR_MX_MOVING_SPEED, W2)
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 3, ADDR_MX_MOVING_SPEED, W3)
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 4, ADDR_MX_MOVING_SPEED, W4)

####################################################################################################################################################

####################################################################################################################################################

# The callback for when the client receives a CONNACK response from the server.

class DemoNode(): #Timer
  def __init__(self):
    self.timer = rospy.Timer(rospy.Duration(dt), self.demo_callback)

  def demo_callback(self, timer):
    global Time
    global count
    global Timestamp
    global Flag_CH
    Time += dt
    #print(Time)

    if(count == 1):
        if(Time-Timestamp >= 2.0):
	        #print("YESSSS")
            Flag_CH = 1
	    count = 0

####################################################################################################################################################



def callback_ridar(data):
    global Wheel_stop_up
    global Wheel_stop_left
    global Wheel_stop_down
    global Wheel_stop_right
    global ridar_mes
    global Flag_CH
    ridar_mes = data.data
    if(Flag_CH == 1):
        if ridar_mes == "Q2_STOP" or ridar_mes == "Q3_STOP":
	        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
            Wheel_stop_up = 1
            Flag_CH = 0
	        WriteDXL_Feedback(0,0,0,0)

        if  ridar_mes == "Q4_STOP" or ridar_mes == "Q5_STOP":
	        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
            Wheel_stop_left = 1
            Flag_CH = 0
	        WriteDXL_Feedback(0,0,0,0)

        if  ridar_mes == "Q6_STOP" or ridar_mes == "Q7_STOP" :
	        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
            Wheel_stop_down = 1
            Flag_CH = 0
	        WriteDXL_Feedback(0,0,0,0)

        if  ridar_mes == "Q8_STOP" or ridar_mes == "Q1_STOP":
	        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
            Wheel_stop_right = 1
            Flag_CH = 0
	        WriteDXL_Feedback(0,0,0,0)



def listener():
    rospy.Subscriber("chatter", String, callback_ridar)
    DemoNode()

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    listener()
    client.subscribe(MQTT_topic)

def myhook():
	client.publish("mobility/debug","Shutdown and Reinitialize",2)
	print("Shutdown and Reintialize")

####################################################################################################################################################

# The callback for when a PUBLISH message is received from the server.

def on_message(client, userdata, msg):
    global mes
    global Flag_OK
    global Flag_CH
    global Wheel_stop_up
    global Wheel_stop_left
    global Wheel_stop_down
    global Wheel_stop_right
    global ridar_mes
    global count
    global Timestamp
    #print(msg.topic+" "+str(msg.payload))
    topic = msg.payload
    mes = msg.payload

    # Wheel Parameter
    Lx = 0.45
    Ly = 0.3
    R = 0.04
    K = 40.92
    eqm = numpy.array([[1, 1, -(Lx+Ly)], [1, -1, (Lx+Ly)], [1, -1, -(Lx+Ly)], [1, 1, (Lx+Ly)]])

    st = json.loads(mes)
    if(topic == 'unity/mobot/mobility'):
        Vx = (st["mobi_vx"])
        Vy = (st["mobi_vy"])
        Wz = (st["mobi_w"])

    elif(topic == 'Mobot/stop'):
        Vx = 0
        Vy = 0
        Wz = 0

    elif(topic == 'Mobot/shutdown'):
        os.system("rosnode kill Mobility_listener")
        rospy.on_shutdown(myhook)
        client.loop_stop()
        client.disconnect()

    if Wheel_stop_up == 1:
        if(Vx < 0):
            Flag_OK = 1
	    Timestamp = Time
	    count = 1
	    Wheel_stop_up = 0
        else:
            Flag_OK = 0

    if Wheel_stop_left == 1:
        if(Vy < 0):
            Flag_OK = 1
	    Timestamp = Time
	    count = 1
	    Wheel_stop_left = 0
        else:
            Flag_OK = 0

    if Wheel_stop_down == 1:
        if(Vx < 0):
            Flag_OK = 1
	    Timestamp = Time
	    count = 1
	    Wheel_stop_down = 0
        else:
            Flag_OK = 0

    if Wheel_stop_right == 1:
        if(Vy < 0):
            Flag_OK = 1
	    Timestamp = Time
	    count = 1
	    Wheel_stop_right = 0
        else:
            Flag_OK = 0

    if(Flag_OK == 1):
        robot_vel = numpy.array([[Vx], [Vy], [Wz]])
        wheel_vel = ((1/R)*eqm).dot((robot_vel))*K

        W1 = int(math.floor(wheel_vel[0])) #0-1023
        W2 = int(math.floor(wheel_vel[1])) #1024-2046
        W3 = int(math.floor(wheel_vel[2])) #0-1023
        W4 = int(math.floor(wheel_vel[3])) #1024-2046

        if(W1 < 0):
            W1 = abs(W1)+1023

        if(W2 < 0):
            W2 = abs(W2)
        else:
            W2 = W2+1023

        if(W3 < 0):
            W3 = abs(W3) +1023

        if(W4 < 0):
            W4 = abs(W4)
        else:
            W4 = W4+1023

        print(W1,W2,W3,W4)

	if Vx != 0 or Vy != 0 or Wz != 0:
            WriteDXL_Feedback(W1,W2,W3,W4)

    if Vx == 0 and Vy == 0 and Wz == 0:
	print("Stop")
        W1 = 0
        W2 = 0
        W3 = 0
        W4 = 0
        WriteDXL_Feedback(W1,W2,W3,W4)


client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

rospy.init_node('Mobility_listener')

#client.connect("service.hcilab.net", 2580, 60)
client.connect("broker.hivemq.com")
# Blocking call that processes network traffic, dispatches callbacks and
# handles reconnecting.
# Other loop*() functions are available that give a threaded interface and a
# manual interface.

client.loop_forever()
