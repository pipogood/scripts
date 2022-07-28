#!/usr/bin/env python
import rospy
import os
import math
import time
from math import sin,cos
from open_manipulator_msgs.msg import KinematicsPose
from sensor_msgs.msg import JointState
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import paho.mqtt.client as mqtt

q1 = ''
q2 = ''
q3 = ''
q4 = ''
q5 = ''
q6 = ''
q7 = ''
q8 = ''
Time = 0
dt = 0.1
stop = 0

def callback(data):
    global data_x
    global data_y
    global data_z
    global data_ox
    global data_oy
    global data_oz
    global data_ow
    data_x = data.pose.position.x
    data_y = data.pose.position.y
    data_z = data.pose.position.z
    data_ox = data.pose.orientation.x
    data_oy = data.pose.orientation.y
    data_oz = data.pose.orientation.z
    data_ow = data.pose.orientation.w
    rospy.loginfo("Data of kinematics_pose is: %.3f %.3f %.3f %.3f %.3f %.3f %.3f", data_x, data_y, data_z, data_ox, data_oy, data_oz, data_ow)
    #rospy.loginfo("Data of kinematics_pose is: %.3f %.3f %.3f", data_x, data_y, data_z)

def listener():
    #rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/gripper/kinematics_pose', KinematicsPose, callback)
    #rospy.sleep(0.01)
    rospy.spin()

def callback_joint(data):
    global data_j
    data_j = data.position

class DemoNode():
  def __init__(self):
    self.timer = rospy.Timer(rospy.Duration(dt), self.demo_callback)

  def demo_callback(self, timer):
    global Time
    Time += dt
    send_to_unity = q1 + ' ' + q2 + ' ' + q3 + ' '+ q4 + ' ' + q5 + ' ' + q6 + ' ' +q7 +' ' +q8
    #if(q2 != "OK"):
    client.publish("RobotFeedback/mobility",send_to_unity)
    #print(Time)

def callback_ridar(data):
    global degree
    global q1
    global q2
    global q3
    global q4
    global q5
    global q6
    global q7
    global q8
    global lidar_x
    global lidar_y
    global stop
    count = 759

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

        if (degree >= 0 and degree < 45):
            if (lidar_x < 0.4 and lidar_y < 0.4):
                q1 = "WARNING"
                # rospy.loginfo(': [%f %f %f]',degree,lidar_x,lidar_y)
                # rospy.loginfo('Q1 :[%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)
            if (lidar_x < 0.2 and lidar_y < 0.4):
                q1 = "STOP"
                #rospy.loginfo(': [%f %f %f]',degree,lidar_x,lidar_y)
                #rospy.loginfo('Q1 :[%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)
            if (lidar_x >= 0.4 and lidar_y < 0.4):
                q1 = "OK"

        if (degree >= 45 and degree < 90):
            if (lidar_x < 0.4 and lidar_y < 0.4):
                q2 = "WARNING"
                # rospy.loginfo(': [%f %f %f]',degree,lidar_x,lidar_y)
                # rospy.loginfo('Q2 :[%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)
            if (lidar_x < 0.2 and lidar_y < 0.4):
                q2 = "STOP"
                #rospy.loginfo(': [%f %f %f]',degree,lidar_x,lidar_y)
                #rospy.loginfo('Q2 :[%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)
            if (lidar_x >= 0.4 and lidar_y < 0.4):
                q2 = "OK"
            # rospy.loginfo(': [%f %f]',degree,data.ranges[i])
            # rospy.loginfo('Q1-Q8 : [%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)

        if (degree >= 90 and degree < 135):
            if (data.ranges[i] < 0.4):
                q3 = "WARNING"
                # rospy.loginfo(': [%f %f %f]',degree,lidar_x,lidar_y)
                # rospy.loginfo('Q3 :[%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)
            if (data.ranges[i] < 0.2):
                q3 = "STOP"
                #rospy.loginfo(': [%f %f]',degree,data.ranges[i])
                #rospy.loginfo('Q3 :[%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)
            if (data.ranges[i] >= 0.4):
                q3 = "OK"

        if (degree >= 135 and degree < 180):
            if (data.ranges[i] < 0.4):
                q4 = "WARNING"
                # rospy.loginfo(': [%f %f %f]',degree,lidar_x,lidar_y)
                # rospy.loginfo('Q4 :[%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)
            if (data.ranges[i] < 0.2):
                q4 = "STOP"
                #rospy.loginfo(': [%f %f]',degree,data.ranges[i])
                #rospy.loginfo('Q4 :[%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)
            if (data.ranges[i] >= 0.4):
                q4 = "OK"
            # rospy.loginfo(': [%f %f]',degree,data.ranges[i])
            # rospy.loginfo('Q1-Q8 : [%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)

        if (degree >= 180 and degree < 225):
            if (lidar_y < 0.4):
                q5 = "WARNING"
                # rospy.loginfo(': [%f %f %f]',degree,lidar_x,lidar_y)
                # rospy.loginfo('Q5 :[%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)
            if (lidar_y < 0.2):
                q5 = "STOP"
                #rospy.loginfo(': [%f %f %f]',degree,lidar_x,lidar_y)
                #rospy.loginfo('Q5 :[%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)
            if (lidar_y >= 0.4):
                q5 = "OK"
            # rospy.loginfo(': [%f %f]',degree,data.ranges[i])
            # rospy.loginfo('Q1-Q8 : [%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)

        if (degree >= 225 and degree < 270):
            if (lidar_y < 0.4):
                q6 = "WARNING"
                #rospy.loginfo(': [%f %f %f]',degree,lidar_x,lidar_y)
                #rospy.loginfo('Q6 :[%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)
            if (lidar_y < 0.2):
                q6 = "STOP"
                #rospy.loginfo(': [%f %f %f]',degree,lidar_x,lidar_y)
                #rospy.loginfo('Q6 :[%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)
            if (lidar_y >= 0.4):
                q6 = "OK"
            # rospy.loginfo(': [%f %f]',degree,data.ranges[i])
            # rospy.loginfo('Q1-Q8 : [%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)
            #rospy.loginfo('Q5: [%s]',q5)
        if (degree >= 270 and degree < 315):
            if (lidar_x < 0.4):
                q7 = "WARNING"
                #rospy.loginfo(': [%f %f %f]',degree,lidar_x,lidar_y)
                #rospy.loginfo('Q7 :[%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)
            if (lidar_x < 0.2):
                q7 = "STOP"
                #rospy.loginfo(': [%f %f %f]',degree,lidar_x,lidar_y)
                #rospy.loginfo('Q7 :[%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)
            if (lidar_x >= 0.4):
                q7 = "OK"
            # rospy.loginfo(': [%f %f]',degree,data.ranges[i])
            # rospy.loginfo('Q1-Q8 : [%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)

        if (degree >= 315 and degree < 360):
            if (lidar_y < 0.4):
                q8 = "WARNING"
                #rospy.loginfo(': [%f %f %f]',degree,lidar_x,lidar_y)
                #rospy.loginfo('Q8 :[%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)
            if (lidar_y < 0.2):
                q8 = "STOP"
                #rospy.loginfo(': [%f %f %f]',degree,lidar_x,lidar_y)
                #rospy.loginfo('Q8 :[%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)
            if (lidar_y >= 0.4):
                q8 = "OK"
            # rospy.loginfo(': [%f %f]',degree,data.ranges[i])
            # rospy.loginfo('Q1-Q8 : [%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)

        #rospy.loginfo(': [%f %f %f]',degree,lidar_x,lidar_y)
        #rospy.loginfo(': [%s %s %s %s %s %s %s %s]',q1,q2,q3,q4,q5,q6,q7,q8)

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        if q1 == "STOP":
            send_str = "Q1_STOP"

        if q2 == "STOP":
            send_str = "Q2_STOP"

        if q3 == "STOP":
            send_str = "Q3_STOP"

        if q4 == "STOP":
            send_str = "Q4_STOP"

        if q5 == "STOP":
            send_str = "Q5_STOP"

        if q6 == "STOP":
            send_str = "Q6_STOP"

        if q7 == "STOP":
            send_str = "Q7_STOP"

        if q8 == "STOP":
            send_str = "Q8_STOP"

        rospy.loginfo(send_str)
        pub.publish(send_str)
        rate.sleep()

def listener_joint():
    rospy.Subscriber('joint_states', JointState, callback_joint)
    rospy.Subscriber('/gripper/kinematics_pose', KinematicsPose, callback)
    # a = os.system("rosnode ping listener_joint")
    # print(a)
    #rospy.spin()
    rospy.sleep(2)

def listener_ridar():
    rospy.Subscriber('/scan', LaserScan, callback_ridar)
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
        os.system("rosnode kill listener_test")
        rospy.on_shutdown(myhook)
        #listener_ridar()
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
