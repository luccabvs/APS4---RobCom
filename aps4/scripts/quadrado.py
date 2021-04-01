#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
from geometry_msgs.msg import Twist, Vector3
from math import pi

v = 0.1  # Velocidade linear
w = -(90*pi/180)  # Velocidade angular

if __name__ == "__main__":
    rospy.init_node("roda_exemplo")
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=3)

    try:
        while not rospy.is_shutdown():
            vel = Twist(Vector3(v,0,0), Vector3(0,0,0))
            pub.publish(vel)
            rospy.sleep(3.0)
            vel = Twist(Vector3(v/2,0,0), Vector3(0,0,0))
            pub.publish(vel)
            rospy.sleep(2.0)
            vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
            pub.publish(vel)
            rospy.sleep(3.0)
            vel = Twist(Vector3(0,0,0), Vector3(0,0,w/7))
            pub.publish(vel)
            rospy.sleep(7.0)
    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")
