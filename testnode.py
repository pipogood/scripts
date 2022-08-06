#!/usr/bin/env python
import rospy
import os
import math
import time
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
Time = 0
dt = 0.1


class DemoNode():
  def __init__(self):
    self.timer = rospy.Timer(rospy.Duration(dt), self.demo_callback)

  def demo_callback(self, timer):
    global Time
    Time += dt
    send_to_unity = q1_1 + ' ' + q1_2 + ' ' + q1_3 + ' '+ q1_4 + ' ' + q2_5 + ' ' + q2_4 + ' ' +q2_3 +' ' +q2_3
    #if(q2 != "OK"):
    client.publish("RobotFeedback/mobility",send_to_unity)
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
                q1_1 = "WARNING"
                # rospy.loginfo(': [%f %f %f]',degree,lidar_x,lidar_y)
                # rospy.loginfo('Q1 :[%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)
            if (lidar_x < 0.2 and lidar_y < 0.4):
                q1_1 = "STOP"
                # rospy.loginfo(': [%f %f %f]',degree,lidar_x,lidar_y)
                rospy.loginfo('Q1_1 :[%s %s %s %s %s %s %s %s]',q1_1,q1_2,q1_3,q1_4,q1_5,q1_6,q1_7,q1_8)
            if (lidar_x >= 0.4):
                q1_1 = "OK"
                # rospy.loginfo(': [%f %f %f]',degree,lidar_x,lidar_y)
                # rospy.loginfo('Q1 :[%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)

        if (degree >= 45 and degree < 90):
            if (lidar_x < 0.4 and lidar_y < 0.4):
                q1_2 = "WARNING"
                # rospy.loginfo(': [%f %f %f]',degree,lidar_x,lidar_y)
                # rospy.loginfo('Q2 :[%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)
            if (lidar_x < 0.2 and lidar_y < 0.4):
                q1_2 = "STOP"
                # rospy.loginfo(': [%f %f %f]',degree,lidar_x,lidar_y)
                rospy.loginfo('Q1_2 :[%s %s %s %s %s %s %s %s]',q1_1,q1_2,q1_3,q1_4,q1_5,q1_6,q1_7,q1_8)
            if (lidar_x >= 0.4 and lidar_y < 0.4):
                q1_2 = "OK"
            # rospy.loginfo(': [%f %f]',degree,data.ranges[i])
            # rospy.loginfo('Q1-Q8 : [%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)

        if (degree >= 90 and degree < 135):
            if (data.ranges[i] < 0.4):
                q1_3 = "WARNING"
                # rospy.loginfo(': [%f %f %f]',degree,lidar_x,lidar_y)
                # rospy.loginfo('Q3 :[%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)
            if (data.ranges[i] < 0.2):
                q1_3 = "STOP"
                #rospy.loginfo(': [%f %f]',degree,data.ranges[i])
                rospy.loginfo('Q1_3 :[%s %s %s %s %s %s %s %s]',q1_1,q1_2,q1_3,q1_4,q1_5,q1_6,q1_7,q1_8)
            if (data.ranges[i] >= 0.4):
                q1_3 = "OK"

        if (degree >= 135 and degree < 180):
            if (data.ranges[i] < 0.4):
                q1_4 = "WARNING"
                # rospy.loginfo(': [%f %fand degree <= 180): %f]',degree,lidar_x,lidar_y)
                # rospy.loginfo('Q4 :[%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)
            if (data.ranges[i] < 0.2):
                q1_4 = "STOP"
                #rospy.loginfo(': [%f %f]',degree,data.ranges[i])
                rospy.loginfo('Q1_4 :[%s %s %s %s %s %s %s %s]',q1_1,q1_2,q1_3,q1_4,q1_5,q1_6,q1_7,q1_8)
            if (data.ranges[i] >= 0.4):
                q1_4 = "OK"
            # rospy.loginfo(': [%f %f]',degree,data.ranges[i])
            # rospy.loginfo('Q1-Q8 : [%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)

        if (degree >= 180 and degree < 225):
            if (lidar_y < 0.4):
                q1_5 = "WARNING"
                # rospy.loginfo(': [%f %f %f]',degree,lidar_x,lidar_y)
                # rospy.loginfo('Q5 :[%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)
            if (lidar_y < 0.2):
                q1_5 = "STOP"
                #rospy.loginfo(': [%f %f %f]',degree,lidar_x,lidar_y)
                rospy.loginfo('Q1_5 :[%s %s %s %s %s %s %s %s]',q1_1,q1_2,q1_3,q1_4,q1_5,q1_6,q1_7,q1_8)
            if (lidar_y >= 0.4):
                q1_5 = "OK"
            # rospy.loginfo(': [%f %f]',degree,data.ranges[i])
            # rospy.loginfo('Q1-Q8 : [%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)

        if (degree >= 225 and degree < 270):
            if (lidar_y < 0.4):
                q1_6 = "WARNING"
                #rospy.loginfo(': [%f %f %f]',degree,lidar_x,lidar_y)
                #rospy.loginfo('Q6 :[%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)
            if (lidar_y < 0.2):
                q1_6 = "STOP"
                #rospy.loginfo(': [%f %f %f]',degree,lidar_x,lidar_y)
                rospy.loginfo('Q1_6 :[%s %s %s %s %s %s %s %s]',q1_1,q1_2,q1_3,q1_4,q1_5,q1_6,q1_7,q1_8)
            if (lidar_y >= 0.4):
                q1_6 = "OK"
            # rospy.loginfo(': [%f %f]',degree,data.ranges[i])
            # rospy.loginfo('Q1-Q8 : [%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)
            #rospy.loginfo('Q5: [%s]',q5)
        if (degree >= 270 and degree < 315):
            if (lidar_x < 0.4):
                q1_7 = "WARNING"
                #rospy.loginfo(': [%f %f %f]',degree,lidar_x,lidar_y)
                #rospy.loginfo('Q7 :[%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)
            if (lidar_x < 0.2):
                q1_7 = "STOP"
                #rospy.loginfo(': [%f %f %f]',degree,lidar_x,lidar_y)
                rospy.loginfo('Q1_7 :[%s %s %s %s %s %s %s %s]',q1_1,q1_2,q1_3,q1_4,q1_5,q1_6,q1_7,q1_8)
            if (lidar_x >= 0.4):
                q1_7 = "OK"
            # rospy.loginfo(': [%f %f]',degree,data.ranges[i])
            # rospy.loginfo('Q1-Q8 : [%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)

        if (degree >= 315 and degree < 360):
            if (lidar_y < 0.4):
                q1_8 = "WARNING"
                #rospy.loginfo(': [%f %f %f]',degree,lidar_x,lidar_y)
                #rospy.loginfo('Q8 :[%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)
            if (lidar_y < 0.2):
                q1_8 = "STOP"
                #rospy.loginfo(': [%f %f %f]',degree,lidar_x,lidar_y)
                rospy.loginfo('Q1_8 :[%s %s %s %s %s %s %s %s]',q1_1,q1_2,q1_3,q1_4,q1_5,q1_6,q1_7,q1_8)
            if (lidar_y >= 0.4):
                q1_8 = "OK"
            # rospy.loginfo(': [%f %f]',degree,data.ranges[i])
            # rospy.loginfo('Q1-Q8 : [%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)

def callback_ridar2(data):
    global q2_1
    global q2_2
    global q2_3
    global q2_4
    global q2_5
    global q2_6
    global q2_7
    global q2_8

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
                # rospy.loginfo(': [%f %f %f]',degree,lidar_x,lidar_y)
                # rospy.loginfo('Q1 :[%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)
            if (lidar_x < 0.2 and lidar_y < 0.4):
                q2_1 = "STOP"
                # rospy.loginfo(': [%f %f %f]',degree,lidar_x,lidar_y)
                rospy.loginfo('Q2_1 :[%s %s %s %s %s %s %s %s]',q2_1,q2_2,q2_3,q2_4,q2_5,q2_6,q2_7,q2_8)
            if (lidar_x >= 0.4):
                q2_1 = "OK"
                # rospy.loginfo(': [%f %f %f]',degree,lidar_x,lidar_y)
                # rospy.loginfo('Q1 :[%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)

        if (degree >= 45 and degree < 90):
            if (lidar_x < 0.4 and lidar_y < 0.4):
                q2_2 = "WARNING"
                # rospy.loginfo(': [%f %f %f]',degree,lidar_x,lidar_y)
                # rospy.loginfo('Q2 :[%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)
            if (lidar_x < 0.2 and lidar_y < 0.4):
                q2_2 = "STOP"
                # rospy.loginfo(': [%f %f %f]',degree,lidar_x,lidar_y)
                rospy.loginfo('Q2_2 :[%s %s %s %s %s %s %s %s]',q2_1,q2_2,q2_3,q2_4,q2_5,q2_6,q2_7,q2_8)
            if (lidar_x >= 0.4 and lidar_y < 0.4):
                q2_2 = "OK"
            # rospy.loginfo(': [%f %f]',degree,data.ranges[i])
            # rospy.loginfo('Q1-Q8 : [%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)

        if (degree >= 90 and degree < 135):
            if (data.ranges[i] < 0.4):
                q2_3 = "WARNING"
                # rospy.loginfo(': [%f %f %f]',degree,lidar_x,lidar_y)
                # rospy.loginfo('Q3 :[%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)
            if (data.ranges[i] < 0.2):
                q2_3 = "STOP"
                #rospy.loginfo(': [%f %f]',degree,data.ranges[i])
                rospy.loginfo('Q2_3 :[%s %s %s %s %s %s %s %s]',q2_1,q2_2,q2_3,q2_4,q2_5,q2_6,q2_7,q2_8)
            if (data.ranges[i] >= 0.4):
                q2_3 = "OK"

        if (degree >= 135 and degree < 180):
            if (data.ranges[i] < 0.4):
                q2_4 = "WARNING"
                # rospy.loginfo(': [%f %fand degree <= 180): %f]',degree,lidar_x,lidar_y)
                # rospy.loginfo('Q4 :[%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)
            if (data.ranges[i] < 0.2):
                q2_4 = "STOP"
                #rospy.loginfo(': [%f %f]',degree,data.ranges[i])
                rospy.loginfo('Q2_4 :[%s %s %s %s %s %s %s %s]',q2_1,q2_2,q2_3,q2_4,q2_5,q2_6,q2_7,q2_8)
            if (data.ranges[i] >= 0.4):
                q2_4 = "OK"
            # rospy.loginfo(': [%f %f]',degree,data.ranges[i])
            # rospy.loginfo('Q1-Q8 : [%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)

        if (degree >= 180 and degree < 225):
            if (lidar_y < 0.4):
                q2_5 = "WARNING"
                # rospy.loginfo(': [%f %f %f]',degree,lidar_x,lidar_y)
                # rospy.loginfo('Q5 :[%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)
            if (lidar_y < 0.2):
                q2_5 = "STOP"
                #rospy.loginfo(': [%f %f %f]',degree,lidar_x,lidar_y)
                rospy.loginfo('Q2_5 :[%s %s %s %s %s %s %s %s]',q2_1,q2_2,q2_3,q2_4,q2_5,q2_6,q2_7,q2_8)
            if (lidar_y >= 0.4):
                q2_5 = "OK"
            # rospy.loginfo(': [%f %f]',degree,data.ranges[i])
            # rospy.loginfo('Q1-Q8 : [%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)

        if (degree >= 225 and degree < 270):
            if (lidar_y < 0.4):
                q2_6 = "WARNING"
                #rospy.loginfo(': [%f %f %f]',degree,lidar_x,lidar_y)
                #rospy.loginfo('Q6 :[%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)
            if (lidar_y < 0.2):
                q2_6 = "STOP"
                #rospy.loginfo(': [%f %f %f]',degree,lidar_x,lidar_y)
                rospy.loginfo('Q2_6 :[%s %s %s %s %s %s %s %s]',q2_1,q2_2,q2_3,q2_4,q2_5,q2_6,q2_7,q2_8)
            if (lidar_y >= 0.4):
                q2_6 = "OK"
            # rospy.loginfo(': [%f %f]',degree,data.ranges[i])
            # rospy.loginfo('Q1-Q8 : [%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)
            #rospy.loginfo('Q5: [%s]',q5)
        if (degree >= 270 and degree < 315):
            if (lidar_x < 0.4):
                q2_7 = "WARNING"
                #rospy.loginfo(': [%f %f %f]',degree,lidar_x,lidar_y)
                #rospy.loginfo('Q7 :[%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)
            if (lidar_x < 0.2):
                q2_7 = "STOP"
                #rospy.loginfo(': [%f %f %f]',degree,lidar_x,lidar_y)
                rospy.loginfo('Q2_7 :[%s %s %s %s %s %s %s %s]',q2_1,q2_2,q2_3,q2_4,q2_5,q2_6,q2_7,q2_8)
            if (lidar_x >= 0.4):
                q2_7 = "OK"
            # rospy.loginfo(': [%f %f]',degree,data.ranges[i])
            # rospy.loginfo('Q1-Q8 : [%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)

        if (degree >= 315 and degree < 360):
            if (lidar_y < 0.4):
                q2_8 = "WARNING"
                #rospy.loginfo(': [%f %f %f]',degree,lidar_x,lidar_y)
                #rospy.loginfo('Q8 :[%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)
            if (lidar_y < 0.2):
                q2_8 = "STOP"
                #rospy.loginfo(': [%f %f %f]',degree,lidar_x,lidar_y)
                rospy.loginfo('Q2_8 :[%s %s %s %s %s %s %s %s]',q2_1,q2_2,q2_3,q2_4,q2_5,q2_6,q2_7,q2_8)
            if (lidar_y >= 0.4):
                q2_8 = "OK"
            # rospy.loginfo(': [%f %f]',degree,data.ranges[i])
            # rospy.loginfo('Q1-Q8 : [%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)

def talker():
    global send_str
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        if q1_1 == "STOP":
            send_str = "Q1_STOP"
            # rospy.loginfo(send_str)
            # pub.publish(send_str)

        if q1_2 == "STOP":
            send_str = "Q2_STOP"
            # rospy.loginfo(send_str)
            # pub.publish(send_str)

        if q1_3 == "STOP":
            send_str = "Q3_STOP"
            # rospy.loginfo(send_str)
            # pub.publish(send_str)

        if q1_4 == "STOP":
            send_str = "Q4_STOP"
            # rospy.loginfo(send_str)
            # pub.publish(send_str)

        if q1_5 == "STOP":
            send_str = "Q5_STOP"
            # rospy.loginfo(send_str)
            # pub.publish(send_str)

        if q1_6 == "STOP":
            send_str = "Q6_STOP"
            # rospy.loginfo(send_str)
            # pub.publish(send_str)

        if q1_7 == "STOP":
            send_str = "Q7_STOP"
            # rospy.loginfo(send_str)
            # pub.publish(send_str)

        if q1_8 == "STOP":
            send_str = "Q8_STOP"
            # rospy.loginfo(send_str)
            # pub.publish(send_str)

        rate.sleep()

def listener_ridar():
    rospy.Subscriber('/lidar0/scan', LaserScan, callback_ridar1)
    rospy.Subscriber('/lidar1/scan', LaserScan, callback_ridar2)
    DemoNode()
    talker()
    rospy.spin()

def myhook():
    print("Shutdown and Reintialize")

client = mqtt.Client()
#client.on_connect = on_connect
client.connect("broker.hivemq.com")

if __name__ == '__main__':
    rospy.init_node('listener_test')
    try:
        # os.system("rosnode kill listener_test")
        # rospy.on_shutdown(myhook)
        listener_ridar()
    except rospy.ROSInterruptException:
        print("exception thrown")
        pass

    #rospy.sleep(1)
    # os.system("rosnode kill listener_test")
    # rospy.on_shutdown(myhook)
    #listener_ridar()

    # joint1 = data_j[0]
    # joint2 = data_j[1]
    # joint3 = data_j[2]
    # joint4 = data_j[3]
    # joint5 = data_j[4]
    # joint6 = data_j[5]

    #print("Data of Joint is:", joint1, joint2, joint3, joint4, joint5, joint6)
    #os.system("rosnode kill listener_joint")
    #rospy.on_shutdown(myhook)
    #listener_joint()
