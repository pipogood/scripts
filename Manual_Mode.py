#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
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


####################################################################################################################################################

####################################################################################################################################################

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("telemm/mob/manual")

####################################################################################################################################################

# The callback for when a PUBLISH message is received from the server.
# "UP"

def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))
    st = split(msg.payload)
    Direction = str(st[0])
    DXL_MAXIMUM_SPEED_VALUE = int(st[1])

    if Direction == "UP":

        for id_u in range(1, 5):
            if id_u == 1:
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, id_u, ADDR_MX_MOVING_SPEED, DXL_MAXIMUM_SPEED_VALUE+1023)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % packetHandler.getRxPacketError(dxl_error))
                else:
                    print("ID[%03d] Speed: %03d " % (id_u, DXL_MAXIMUM_SPEED_VALUE+1023))

            elif id_u == 4:
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, id_u, ADDR_MX_MOVING_SPEED, DXL_MAXIMUM_SPEED_VALUE+1023)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % packetHandler.getRxPacketError(dxl_error))
                else:
                    print("ID[%03d] Speed: %03d " % (id_u, DXL_MAXIMUM_SPEED_VALUE+1023))
            else:
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, id_u, ADDR_MX_MOVING_SPEED, DXL_MAXIMUM_SPEED_VALUE)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % packetHandler.getRxPacketError(dxl_error))
                else:
                    print("ID[%03d] Speed: %03d " % (id_u, DXL_MAXIMUM_SPEED_VALUE))


####################################################################################################################################################

####################################################################################################################################################

# "DOWN"

    if Direction == "DOWN":

        for id_u in range(1, 5):
            if id_u == 2:
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, id_u, ADDR_MX_MOVING_SPEED, DXL_MAXIMUM_SPEED_VALUE+1023)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % packetHandler.getRxPacketError(dxl_error))
                else:
                    print("ID[%03d] Speed: %03d " % (id_u, DXL_MAXIMUM_SPEED_VALUE+1023))

            elif id_u == 3:
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, id_u, ADDR_MX_MOVING_SPEED, DXL_MAXIMUM_SPEED_VALUE+1023)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % packetHandler.getRxPacketError(dxl_error))
                else:
                    print("ID[%03d] Speed: %03d " % (id_u, DXL_MAXIMUM_SPEED_VALUE+1023))
            else:
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, id_u, ADDR_MX_MOVING_SPEED, DXL_MAXIMUM_SPEED_VALUE)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % packetHandler.getRxPacketError(dxl_error))
                else:
                    print("ID[%03d] Speed: %03d " % (id_u, DXL_MAXIMUM_SPEED_VALUE))


####################################################################################################################################################

####################################################################################################################################################

# "LEFT"

    if Direction == "LEFT":

        for id_u in range(1, 5):
            if id_u == 1:
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, id_u, ADDR_MX_MOVING_SPEED, DXL_MAXIMUM_SPEED_VALUE+1023)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % packetHandler.getRxPacketError(dxl_error))
                else:
                    print("ID[%03d] Speed: %03d " % (id_u, DXL_MAXIMUM_SPEED_VALUE+1023))

            elif id_u == 2:
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, id_u, ADDR_MX_MOVING_SPEED, DXL_MAXIMUM_SPEED_VALUE+1023)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % packetHandler.getRxPacketError(dxl_error))
                else:
                    print("ID[%03d] Speed: %03d " % (id_u, DXL_MAXIMUM_SPEED_VALUE+1023))
            else:
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, id_u, ADDR_MX_MOVING_SPEED, DXL_MAXIMUM_SPEED_VALUE)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % packetHandler.getRxPacketError(dxl_error))
                else:
                    print("ID[%03d] Speed: %03d " % (id_u, DXL_MAXIMUM_SPEED_VALUE))


####################################################################################################################################################

####################################################################################################################################################

# "RIGHT"

    if Direction == "RIGHT":

        for id_u in range(1, 5):
            if id_u == 3:
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, id_u, ADDR_MX_MOVING_SPEED, DXL_MAXIMUM_SPEED_VALUE+1023)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % packetHandler.getRxPacketError(dxl_error))
                else:
                    print("ID[%03d] Speed: %03d " % (id_u, DXL_MAXIMUM_SPEED_VALUE+1023))

            elif id_u == 4:
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, id_u, ADDR_MX_MOVING_SPEED, DXL_MAXIMUM_SPEED_VALUE+1023)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % packetHandler.getRxPacketError(dxl_error))
                else:
                    print("ID[%03d] Speed: %03d " % (id_u, DXL_MAXIMUM_SPEED_VALUE+1023))
            else:
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, id_u, ADDR_MX_MOVING_SPEED, DXL_MAXIMUM_SPEED_VALUE)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % packetHandler.getRxPacketError(dxl_error))
                else:
                    print("ID[%03d] Speed: %03d " % (id_u, DXL_MAXIMUM_SPEED_VALUE))


####################################################################################################################################################

####################################################################################################################################################

# "STOP"

    if Direction == "STOP":
        for id_s in range(1, 5):
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, id_s, ADDR_MX_MOVING_SPEED, 0)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            else:
                print("ID[%03d] Speed: %03d " % (id_s, 0))

####################################################################################################################################################

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect("service.hcilab.net", 2580, 60)

# Blocking call that processes network traffic, dispatches callbacks and
# handles reconnecting.
# Other loop*() functions are available that give a threaded interface and a
# manual interface.
client.loop_forever()
