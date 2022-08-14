#!/usr/bin/env python
import rospy
import os
import math
import time
import json
from math import sin,cos
# from open_manipulator_msgs.msg import KinematicsPose
# from sensor_msgs.msg import JointState
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import paho.mqtt.client as mqtt

q1_1 = ''
q1_2 = ''
q1_3 = ''
q1_4 = ''
q1_5 = ''
q1_6 = ''
q1_7 = ''
q1_8 = ''
q2_1 = ''
q2_2 = ''
q2_3 = ''
q2_4 = ''
q2_5 = ''
q2_6 = ''
q2_7 = ''
q2_8 = ''
Q1 = ''
Q2 = ''
Q3 = ''
Q4 = ''
Q5 = ''
Q6 = ''
Q7 = ''
Q8 = ''
Time = 0
dt = 0.1
send_to_unity = {}

class DemoNode():
  def __init__(self):
    self.timer = rospy.Timer(rospy.Duration(dt), self.demo_callback)

  def demo_callback(self, timer):
    # global Time
    # Time += dt
    send_to_unity = {'lidar_q1':Q1, 'lidar_q2':Q2, 'lidar_q3':Q3, 'lidar_q4':Q4, 'lidar_q5':Q5,'lidar_q6':Q6, 'lidar_q7':Q7, 'lidar_q8':Q8}
    #print(send_to_unity)
    client.publish("mobot/unity/lidar",json.dumps(send_to_unity,sort_keys=True))
    #print(Time)

def callback_ridar1(data):
    global q1_1
    global q1_2
    global q1_3
    global q1_4
    global q1_5
    global q1_6
    global q1_7
    global q1_8
    global Q2
    global Q3
    global Q4
    global Q5

    count = int(math.floor(data.scan_time/data.time_increment))

    for i in range (0,count):
        degree = (data.angle_min + data.angle_increment * i)*57.2958

        if(degree >= 90 and degree <= 180):
            degree = degree - 90
        elif(degree >= -180 and degree < 0):
            degree = (180-abs(degree)) + 90
        else:
            degree =  270 + degree

        lidar_x = abs((data.ranges[i])*sin(degree/57.2958))
        lidar_y = abs((data.ranges[i])*cos(degree/57.2958))

        #print("ridar1",count,degree)

        if (degree >= 1 and degree < 45):
            if (lidar_x < 0.4 and lidar_y < 0.4):
                q1_1 = "BEWARE"
            if (lidar_x < 0.2 and lidar_y < 0.4):
                q1_1 = "STOP"
            if (lidar_x >= 0.4):
                q1_1 = "OK"

        if (degree >= 45 and degree < 90):
            if (lidar_x < 0.4 and lidar_y < 0.4):
                q1_2 = "BEWARE"
            if (lidar_x < 0.2 and lidar_y < 0.4):
                q1_2 = "STOP"
            if (lidar_x >= 0.4 and lidar_y < 0.4):
                q1_2 = "OK"

        if (degree >= 90 and degree < 135):
            if (data.ranges[i] < 0.4):
                q1_3 = "BEWARE"
            if (data.ranges[i] < 0.2):
                q1_3 = "STOP"
            if (data.ranges[i] >= 0.4):
                q1_3 = "OK"

        if (degree >= 135 and degree < 180):
            if (data.ranges[i] < 0.4):
                q1_4 = "BEWARE"
            if (data.ranges[i] < 0.2):
                q1_4 = "STOP"
            if (data.ranges[i] >= 0.4):
                q1_4 = "OK"

        if (degree >= 180 and degree < 225):
            if (lidar_y < 0.4):
                q1_5 = "BEWARE"
            if (lidar_y < 0.2):
                q1_5 = "STOP"
            if (lidar_y >= 0.4):
                q1_5 = "OK"

        if (degree >= 225 and degree < 270):
            if (lidar_y < 0.4):
                q1_6 = "BEWARE"
            if (lidar_y < 0.2):
                q1_6 = "STOP"
            if (lidar_y >= 0.4):
                q1_6 = "OK"
        if (degree >= 270 and degree < 315):
            if (lidar_x < 0.4):
                q1_7 = "BEWARE"
            if (lidar_x < 0.2):
                q1_7 = "STOP"
            if (lidar_x >= 0.4):
                q1_7 = "OK"

        if (degree >= 315 and degree < 360):
            if (lidar_y < 0.4):
                q1_8 = "BEWARE"
            if (lidar_y < 0.2):
                q1_8 = "STOP"
            if (lidar_y >= 0.4):
                q1_8 = "OK"

        if  q1_8 == "STOP":
            Q2 = "STOP"
        elif q1_8 == "BEWARE":
            Q2 = "BEWARE"
        elif q1_8 == "OK":
            Q2 = "OK"

        if  q1_1 == "STOP":
            Q3 = "STOP"
        elif q1_1 == "BEWARE":
            Q3 = "BEWARE"
        elif q1_1 == "OK":
            Q3 = "OK"

        if q1_2 == "STOP" or q1_3 == "STOP":
            Q4 = "STOP"
        elif q1_2 == "BEWARE" or q1_3 == "BEWARE":
            Q4 = "BEWARE"
        elif q1_2 == "OK" and q1_3 == "OK":
            Q4 = "OK"

        if q1_4 == "STOP" or q1_5 == "STOP":
            Q5 = "STOP"
        elif q1_4 == "BEWARE" or q1_5 == "BEWARE":
            Q5 = "BEWARE"
        elif q1_4 == "OK" and q1_5 == "OK":
            Q5 = "OK"

def callback_ridar2(data):
    global q2_1
    global q2_2
    global q2_3
    global q2_4
    global q2_5
    global q2_6
    global q2_7
    global q2_8
    global Q1
    global Q6
    global Q7
    global Q8

    count = int(math.floor(data.scan_time/data.time_increment))

    for i in range (0,count):
        degree = (data.angle_min + data.angle_increment * i)*57.2958

        if(degree >= 90 and degree <= 180):
            degree = degree - 90
        elif(degree >= -180 and degree < 0):
            degree = (180-abs(degree)) + 90
        else:
            degree =  270 + degree

        lidar_x = abs((data.ranges[i])*sin(degree/57.2958))
        lidar_y = abs((data.ranges[i])*cos(degree/57.2958))

        #print("ridar2",count,degree)

        if (degree >= 1 and degree < 45):
            if (lidar_x < 0.4 and lidar_y < 0.4):
                q2_1 = "WARNING"
            if (lidar_x < 0.2 and lidar_y < 0.4):
                q2_1 = "STOP"
            if (lidar_x >= 0.4):
                q2_1 = "OK"

        if (degree >= 45 and degree < 90):
            if (lidar_x < 0.4 and lidar_y < 0.4):
                q2_2 = "WARNING"
            if (lidar_x < 0.2 and lidar_y < 0.4):
                q2_2 = "STOP"
            if (lidar_x >= 0.4 and lidar_y < 0.4):
                q2_2 = "OK"

        if (degree >= 90 and degree < 135):
            if (data.ranges[i] < 0.4):
                q2_3 = "WARNING"
            if (data.ranges[i] < 0.2):
                q2_3 = "STOP"
            if (data.ranges[i] >= 0.4):
                q2_3 = "OK"

        if (degree >= 135 and degree < 180):
            if (data.ranges[i] < 0.4):
                q2_4 = "WARNING"
            if (data.ranges[i] < 0.2):
                q2_4 = "STOP"
            if (data.ranges[i] >= 0.4):
                q2_4 = "OK"

        if (degree >= 180 and degree < 225):
            if (lidar_y < 0.4):
                q2_5 = "WARNING"
            if (lidar_y < 0.2):
                q2_5 = "STOP"
            if (lidar_y >= 0.4):
                q2_5 = "OK"

        if (degree >= 225 and degree < 270):
            if (lidar_y < 0.4):
                q2_6 = "WARNING"
            if (lidar_y < 0.2):
                q2_6 = "STOP"
            if (lidar_y >= 0.4):
                q2_6 = "OK"

        if (degree >= 270 and degree < 315):
            if (lidar_x < 0.4):
                q2_7 = "WARNING"
            if (lidar_x < 0.2):
                q2_7 = "STOP"
            if (lidar_x >= 0.4):
                q2_7 = "OK"

        if (degree >= 315 and degree < 360):
            if (lidar_y < 0.4):
                q2_8 = "WARNING"
            if (lidar_y < 0.2):
                q2_8 = "STOP"
            if (lidar_y >= 0.4):
                q2_8 = "OK"

        if  q2_1 == "STOP":
            Q7 = "STOP"
        elif q2_1 == "WARNING":
            Q7 = "BEWARE"
        elif q2_1 == "OK":
            Q7 = "OK"

        if  q2_8 == "STOP":
            Q6 = "STOP"
        elif q2_8 == "WARNING":
            Q6 = "BEWARE"
        elif q2_8 == "OK":
            Q6 = "OK"

        if q2_2 == "STOP" or q2_3 == "STOP":
            Q8 = "STOP"
        elif q2_2 == "WARNING" or q2_3 == "WARNING":
            Q8 = "BEWARE"
        elif q2_2 == "OK" and q2_3 == "OK":
            Q8 = "OK"

        if q2_4 == "STOP" or q2_5 == "STOP":
            Q1 = "STOP"
        elif q2_4 == "WARNING" or q2_5 == "WARNING":
            Q1 = "BEWARE"
        elif q2_4 == "OK" and q2_5 == "OK":
            Q1 = "OK"

def talker():
    global send_str
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():

        if Q1 == "STOP":
            send_str = "Q1_STOP"
            rospy.loginfo(send_str)
            pub.publish(send_str)

        if Q2 == "STOP":
            send_str = "Q2_STOP"
            rospy.loginfo(send_str)
            pub.publish(send_str)

        if Q3 == "STOP":
            send_str = "Q3_STOP"
            rospy.loginfo(send_str)
            pub.publish(send_str)

        if Q4 == "STOP":
            send_str = "Q4_STOP"
            rospy.loginfo(send_str)
            pub.publish(send_str)

        if Q5 == "STOP":
            send_str = "Q5_STOP"
            rospy.loginfo(send_str)
            pub.publish(send_str)

        if Q6 == "STOP":
            send_str = "Q6_STOP"
            rospy.loginfo(send_str)
            pub.publish(send_str)

        if Q7 == "STOP":
            send_str = "Q7_STOP"
            rospy.loginfo(send_str)
            pub.publish(send_str)

        if Q8 == "STOP":
            send_str = "Q8_STOP"
            rospy.loginfo(send_str)
            pub.publish(send_str)

        rate.sleep()

def listener_ridar():
    #rospy.Subscriber('/lidar0/scan', LaserScan, callback_ridar1)
    #rospy.Subscriber('/lidar1/scan', LaserScan, callback_ridar2)
    DemoNode()
    talker()
    rospy.spin()

def myhook():
    print("Shutdown and Reintialize")

client = mqtt.Client()
client.connect("broker.hivemq.com")

if __name__ == '__main__':
    rospy.init_node('listener_test')
    try:
        listener_ridar()
    except rospy.ROSInterruptException:
        print("exception thrown")
        pass
