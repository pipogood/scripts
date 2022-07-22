#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import numpy
import math
import time
import rospy
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
from sensor_msgs.msg import LaserScan

####################################################################################################################################################

# Control table ADDRess for MX-106
# EEPROM REGISTER ADDRESSES - Permanently stored in memory once changed
ADDR_MX_MODEL_NUMBER           = 0
ADDR_MX_FIRMWARE_VERSION       = 2
ADDR_MX_ID                     = 3
ADDR_MX_BAUD_RATE              = 4
ADDR_MX_RETURN_DELAY_TIME      = 5
ADDR_MX_CW_ANGLE_LIMIT         = 6
ADDR_MX_CCW_ANGLE_LIMIT        = 8
ADDR_MX_DRIVE_MODE             = 10
ADDR_MX_LIMIT_TEMPERATURE      = 11
ADDR_MX_MIN_VOLTAGE_LIMIT      = 12
ADDR_MX_MAX_VOLTAGE_LIMIT      = 13
ADDR_MX_MAX_TORQUE             = 14
ADDR_MX_STATUS_RETURN_LEVEL    = 16
ADDR_MX_ALARM_LED              = 17
ADDR_MX_SHUTDOWN               = 18
ADDR_MX_MULTI_TURN_OFFSET      = 20
ADDR_MX_RESOLUTION_DIVIDER     = 22

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
DEVICENAME                     = '/dev/ttyUSB0'    # Port connected to controller
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

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_MX_MOVING_SPEED, LEN_MX_MOVING_SPEED )


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
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 1, ADDR_MX_MOVING_SPEED, W1)
    Feedback()
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 2, ADDR_MX_MOVING_SPEED, W2)
    Feedback()
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 3, ADDR_MX_MOVING_SPEED, W3)
    Feedback()
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 4, ADDR_MX_MOVING_SPEED, W4)
    Feedback()


def Set_robot_safe():
    Time = 0

    if Wheel_stop_left == 1:
        #move right
        Vx = 0
        Vy = 0.1
        Wz = 0
        robot_vel = numpy.array([[Vx], [Vy], [Wz]])
        wheel_vel = ((1/R)*eqm).dot((robot_vel))*K

        W1 = int(math.floor(wheel_vel[0])+1023)
        W2 = -(int(math.floor(wheel_vel[1]))-1023)
        W3 = -(int(math.floor(wheel_vel[2])))
        W4 = (int(math.floor(wheel_vel[3])))
        WriteDXL_Feedback(W1,W2,W3,W4)

        if(Time == 1.5):
            W1 = 0
            W2 = 0
            W3 = 0
            W4 = 0
            WriteDXL_Feedback(W1,W2,W3,W4)
            Wheel_stop_left = 0

    if Wheel_stop_up == 1:
        #move down
        Vx = -0.1
        Vy = 0
        Wz = 0
        robot_vel = numpy.array([[Vx], [Vy], [Wz]])
        wheel_vel = ((1/R)*eqm).dot((robot_vel))*K

        W1 = -(int(math.floor(wheel_vel[0]))-1023)
        W2 = -(int(math.floor(wheel_vel[0])))
        W3 = -(int(math.floor(wheel_vel[0]))-1023)
        W4 = -(int(math.floor(wheel_vel[0])))
        WriteDXL_Feedback(W1,W2,W3,W4)

        if(Time == 1.5):
            W1 = 0
            W2 = 0
            W3 = 0
            W4 = 0
            WriteDXL_Feedback(W1,W2,W3,W4)
            Wheel_stop_up = 0

####################################################################################################################################################

####################################################################################################################################################

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("UnityToRobot/mobility")

class DemoNode(): #Timer
  def __init__(self):
    self.timer = rospy.Timer(rospy.Duration(dt), self.demo_callback)

  def demo_callback(self, timer):
    global Time
    Time += dt
    #print(Time)

####################################################################################################################################################



def callback_ridar(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    ridar_mes = data.data
    if ridar_mes == "STOP":
        client.publish("UnityToRobot/mobility","0 0 0")

def listener():
    rospy.Subscriber("chatter", String, callback_ridar)
    DemoNode()


# def Mobility_listener():
#     rospy.init_node('Mobility_listener',anonymous=True)
#     rospy.Subscriber('/scan', LaserScan, callback_ridar)
#     DemoNode()

####################################################################################################################################################

# The callback for when a PUBLISH message is received from the server.

def on_message(client, userdata, msg):
    global mes
    print(msg.topic+" "+str(msg.payload))
    mes = msg.payload
    #listener()

    Lx = 0.45
    Ly = 0.3

    # Wheel Parameter

    R = 0.04
    K = 40.92
    eqm = numpy.array([[1, 1, -(Lx+Ly)], [1, -1, (Lx+Ly)], [1, -1, -(Lx+Ly)], [1, 1, (Lx+Ly)]])
    st = mes.split()

    Vx = float(st[0])
    Vy = float(st[1])
    Wz = float(st[2])

    robot_vel = numpy.array([[Vx], [Vy], [Wz]])
    wheel_vel = ((1/R)*eqm).dot((robot_vel))*K


    # "UP"

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
    WriteDXL_Feedback(W1,W2,W3,W4)

    # if Vx > 0 and Vy == 0 and Wz == 0:
	#     print("Up")
    #     W1 = int(math.floor(wheel_vel[0]))
    #     W2 = int(math.floor(wheel_vel[1]))+1023
    #     W3 = int(math.floor(wheel_vel[2]))
    #     W4 = int(math.floor(wheel_vel[3]))+1023
	#     print(W1,W2,W3,W4)
    #     WriteDXL_Feedback(W1,W2,W3,W4)
    #
    #
    #
    # # "DOWN"
    #
    # if Vx < 0 and Vy == 0 and Wz == 0:
	#     print("Down")
    #     W1 = -(int(math.floor(wheel_vel[0]))-1023)
    #     W2 = -(int(math.floor(wheel_vel[0])))
    #     W3 = -(int(math.floor(wheel_vel[0]))-1023)
    #     W4 = -(int(math.floor(wheel_vel[0])))
	#     print(W1,W2,W3,W4)
    #     WriteDXL_Feedback(W1,W2,W3,W4)
    #
    #
    # # "RIGHT"
    #
    # if Vx == 0 and Vy > 0 and Wz ==0:
	#     print("Right")
    #     W1 = int(math.floor(wheel_vel[0])+1023)
    #     W2 = -(int(math.floor(wheel_vel[1]))-1023)
    #     W3 = -(int(math.floor(wheel_vel[2])))
    #     W4 = (int(math.floor(wheel_vel[3])))
	#     print(W1,W2,W3,W4)
    #     WriteDXL_Feedback(W1,W2,W3,W4)
    #
    #
    # # "LEFT"
    #
    # if Vx == 0 and Vy < 0 and Wz == 0:
    #     print("Left")
    #     W1 = -(int(math.floor(wheel_vel[0])))
    #     W2 = int(math.floor(wheel_vel[1]))
    #     W3 = int(math.floor(wheel_vel[2])+1023)
    #     W4 = -(int(math.floor(wheel_vel[3]))-1023)
    #     print(W1,W2,W3,W4)
    #     WriteDXL_Feedback(W1,W2,W3,W4)
    #
    #
    # # "TURN-RIGHT"
    #
    # if Vx == 0 and Vy == 0 and Wz > 0:
	#     print("Turn-right")
    #     W1 = -(int(math.floor(wheel_vel[0]))-1023)
    #     W2 = (int(math.floor(wheel_vel[1]))+1023)
    #     W3 = -(int(math.floor(wheel_vel[2]))-1023)
    #     W4 = (int(math.floor(wheel_vel[3]))+1023)
	#     print(W1,W2,W3,W4)
    #     WriteDXL_Feedback(W1,W2,W3,W4)
    #
    #
    # # "TURN-LEFT"
    #
    # if Vx == 0 and Vy == 0 and Wz < 0:
	#     print("Turn-left")
    #     W1 = (int(math.floor(wheel_vel[0])))
    #     W2 = -(int(math.floor(wheel_vel[1])))
    #     W3 = int(math.floor(wheel_vel[2]))
    #     W4 = -(int(math.floor(wheel_vel[3])))
	#     print(W1,W2,W3,W4)
    #     WriteDXL_Feedback(W1,W2,W3,W4)


    # "STOP"

    if Vx == 0 and Vy == 0 and Wz == 0:
	    print("Stop")
        W1 = 0
        W2 = 0
        W3 = 0
        W4 = 0
        WriteDXL_Feedback(W1,W2,W3,W4)

    if Wheel_stop_left == 1:
        print("lidar_stop_left")
        W1 = 0
        W2 = 0
        W3 = 0
        W4 = 0
        WriteDXL_Feedback(W1,W2,W3,W4)
        #Set_robot_safe()


    if Wheel_stop_up == 1:
	    print("lidar_stop_up")
        W1 = 0
        W2 = 0
        W3 = 0
        W4 = 0
        WriteDXL_Feedback(W1,W2,W3,W4)
        #Set_robot_safe()

    # # "0-180 Degree"
    # if Vx > 0 and Vy > 0 and Wz == 0:
	#     print("0-180")
    #     W1 = int(math.floor(wheel_vel[0]))
    #     W2 = int(math.floor(wheel_vel[1]))+1023
    #     W3 = int(math.floor(wheel_vel[2]))
    #     W4 = int(math.floor(wheel_vel[3]))+1023
	#     print(W1,W2,W3,W4)
    #     WriteDXL_Feedback(W1,W2,W3,W4)
    #
    # # "181-359 Degree"    if mes == "SHUTDOWN":
    #
    # if Vx < 0 and Vy < 0 and Wz == 0:
	#     print("181-359")
    #     W1 = -(int(math.floor(wheel_vel[0]))-1023)
    #     W2 = -(int(math.floor(wheel_vel[0])))
    #     W3 = -(int(math.floor(wheel_vel[0]))-1023)
    #     W4 = -(int(math.floor(wheel_vel[0])))
	#     print(W1,W2,W3,W4)
    #     WriteDXL_Feedback(W1,W2,W3,W4)

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

#client.connect("service.hcilab.net", 2580, 60)
client.connect("broker.hivemq.com")
# Blocking call that processes network traffic, dispatches callbacks and
# handles reconnecting.
# Other loop*() functions are available that give a threaded interface and a
# manual interface.

client.loop_forever()
