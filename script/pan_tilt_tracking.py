#!/usr/bin/env python

#Title: 
#Author: Khairul Izwan Bin Kamsani - [23-01-2020]
#Description: 

from __future__ import print_function
from __future__ import division

# import the necessary packages
from imutils import face_utils
import imutils
import time
import cv2
import os
import rospkg
import sys
import rospy
import numpy as np

# import the necessary ROS messages
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from common_face_application.msg import objCenter as objCoord
from common_pid_controller.pid import PID

from geometry_msgs.msg import Twist

class PanTiltTracking:

	def __init__(self):

		rospy.logwarn("PanTiltTracking (ROI) node [ONLINE]")

		# set PID values for panning
		self.panP = 0.1
		self.panI = 0.0008
		self.panD = 0.01

		# set PID values for tilting
		self.tiltP = 0.1
		self.tiltI = 0.0008
		self.tiltD = 0.01

		# create a PID and initialize it
		self.panPID = PID(self.panP, self.panI, self.panD)
		self.tiltPID = PID(self.tiltP, self.tiltI, self.tiltD)

		self.panPID.initialize()
		self.tiltPID.initialize()

#		self.image_recieved = False

		# rospy shutdown
		rospy.on_shutdown(self.cbShutdown)

		# Subscribe to CameraInfo msg
		cameraInfo_topic = "/cv_camera/camera_info"
		self.cameraInfo_sub = rospy.Subscriber(cameraInfo_topic, CameraInfo, 
			self.cbCameraInfo)

#		# Publish to objCenter msg
#		objCoord_topic = "/objCoord"
#		self.objCoord_pub = rospy.Subscriber(objCoord_topic, objCoord, queue_size=10)
		
		# Subscribe to objCenter msg
		objCoord_topic = "/objCoord"
		self.objCoord_sub = rospy.Subscriber(objCoord_topic, objCoord, 
			self.cbObjCenter)

		# Allow up to one second to connection
		rospy.sleep(1)

	# Get CameraInfo
	def cbCameraInfo(self, msg):

		self.imgWidth = msg.width
		self.imgHeight = msg.height

		# calculate the center of the frame as this is where we will
		# try to keep the object
		self.centerX = self.imgWidth // 2
#		self.centerY = self.imgHeight // 2
		self.centerY = np.int0(0.9 * self.imgHeight)

	# Get objCenter coordinates
	def cbObjCenter(self, msg):

		self.objX = msg.centerX
		self.objY = msg.centerY

	# show information callback
	def cbInfo(self):

		self.panErr, self.panOut = self.cbPIDprocess(self.panPID, self.objX, self.centerX)
		self.tiltErr, self.tiltOut = self.cbPIDprocess(self.tiltPID, self.objY, self.centerY)
		
		rospy.loginfo([self.tiltErr, self.tiltOut])

	def cbPIDprocess(self, pid, objCoord, centerCoord):

		# calculate the error
		error = centerCoord - objCoord

		# update the value
		output = pid.update(error)

		return error, output

#	def cbPIDtracking(self):

#		twist = Twist()
#		self.cbInfo()
#		if self.

	# rospy shutdown callback
	def cbShutdown(self):
		try:
			rospy.logwarn("PanTiltTracking (ROI) node [OFFLINE]")
		finally:
			cv2.destroyAllWindows()
			
#			twist = Twist()
#			twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
#			twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
#			pub.publish(twist)

if __name__ == '__main__':

	# Initializing your ROS Node
	rospy.init_node('pant_tilt_tracking', anonymous=False)
	pt = PanTiltTracking()

	# Camera preview
	while not rospy.is_shutdown():
		pt.cbInfo()
