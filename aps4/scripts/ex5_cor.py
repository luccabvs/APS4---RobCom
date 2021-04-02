#! /usr/bin/env python3
# -*- coding:utf-8 -*-

__author__ = ["Rachel P. B. Moraes", "Igor Montagner", "Fabio Miranda"]


import rospy
import numpy as np
import tf
import math
import cv2
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from cv_bridge import CvBridge, CvBridgeError
import cormodule


bridge = CvBridge()

cv_image = None
media = []
centro = []
atraso = 1.5E9

area = 0.0

check_delay = False 

def roda_todo_frame(imagem):
	print("frame")
	global cv_image
	global media
	global centro

	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime # calcula o lag
	delay = lag.nsecs
	if delay > atraso and check_delay==True:
		return 
	try:
		antes = rospy.Time.now()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		# cv_image = cv2.flip(cv_image, -1) # Descomente se for robo real
		media, centro, maior_area =  cormodule.identifica_cor(cv_image)
		depois = rospy.Time.now()
		cv2.imshow("Camera", cv_image)
	except CvBridgeError as e:
		print('ex', e)

distancia = 1.0

def scaneou(dado):
    global distancia
    ranges = np.array(dado.ranges).round(decimals=2)
    distancia = ranges[0]
        
if __name__=="__main__":
	rospy.init_node("cor")

	topico_imagem = "camera/image/compressed"
    
	recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)
    
	frente = Twist(Vector3(0.1, 0, 0), Vector3(0, 0, 0))
	parado = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
    
    
	try:

		while not rospy.is_shutdown():
			vel = frente
			if len(media) != 0 and len(centro) != 0:
				if (media[0] > centro[0]):
					vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.1))
				if (media[0] < centro[0]):
					vel = Twist(Vector3(0,0,0), Vector3(0,0,0.1))
			velocidade_saida.publish(vel)
			rospy.sleep(0.05)

			if distancia > 0.30:
				vel = frente
			if distancia <= 0.30:
				vel = parado
			velocidade_saida.publish(vel)
			rospy.sleep(0.05)
            

	except rospy.ROSInterruptException:
	    print("Ocorreu uma exceção com o rospy")
