#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy

import numpy as np

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan


distancia = 20.0

def scaneou(dado):
    global distancia
    ranges = np.array(dado.ranges).round(decimals=2)
    distancia = ranges[0]


if __name__=="__main__":

    rospy.init_node("le_scan")

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)

    anda_re = Twist(Vector3(-0.2, 0, 0), Vector3(0, 0, 0))
    frente = Twist(Vector3(0.2, 0, 0), Vector3(0, 0, 0))
    parado = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
    

    while not rospy.is_shutdown():
        velocidade_saida.publish(frente)

        if distancia < 0.95:
            velocidade_saida.publish(parado)
            rospy.sleep(2)
            velocidade_saida.publish(anda_re)
        if distancia > 1.05:
            velocidade_saida.publish(parado)
            rospy.sleep(2)
            velocidade_saida.publish(frente)

           
        rospy.sleep(0.5)


