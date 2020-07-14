#!/usr/bin/env python

#Title: Python Subscriber for Tank Navigation
#Author: Khairul Izwan Bin Kamsani - [23-01-2020]
#Description: Tank Navigation Subcriber Nodes (Python)

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

# import the necessary ROS messages
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

#from cv_bridge import CvBridge
#from cv_bridge import CvBridgeError

#from sensor_msgs.msg import RegionOfInterest

#from common_face_application.objcenter import ObjCenter
from common_face_application.msg import objCenter
from common_pid_controller.pid import PID

class PanTiltTracking:

	def __init__(self):

		rospy.logwarn("PanTiltTracking (ROI) node [ONLINE]")

#		self.bridge = CvBridge()
#		self.rospack = rospkg.RosPack()
#		self.roi = RegionOfInterest()
#		self.centerRoi = objCenter()

		# set PID values for panning
		self.panP = 0.1
		self.panI = 0.0008
		self.panD = 0.01

		# set PID values for tilting
		self.tiltP = 1
		self.tiltI = 0.10
		self.tiltD = 0.002

		# create a PID and initialize it
		self.panPID = PID(self.panP, self.panI, self.panD)
		self.tiltPID = PID(self.tiltP, self.tiltI, self.tiltD)

		self.panPID.initialize()
		self.tiltPID.initialize()

#		self.image_recieved = False

		# rospy shutdown
		rospy.on_shutdown(self.cbShutdown)

#		# Import haarCascade files
#		self.p = os.path.sep.join([self.rospack.get_path('common_face_application')])
#		self.libraryDir = os.path.join(self.p, "model")

#		self.haar_filename = self.libraryDir + "/haarcascade_frontalface_default.xml"

#		# Path to input Haar cascade for face detection
#		self.faceCascade = ObjCenter(self.haar_filename)

#		# Subscribe to Image msg
#		image_topic = "/cv_camera/image_raw"
#		self.image_sub = rospy.Subscriber(image_topic, Image, self.cbImage)

		# Subscribe to CameraInfo msg
		cameraInfo_topic = "/cv_camera/camera_info"
		self.cameraInfo_sub = rospy.Subscriber(cameraInfo_topic, CameraInfo, 
			self.cbCameraInfo)

		# Subscribe to objCenter msg
		objCenter_topic = "/centerROI"
		self.objCenter_sub = rospy.Subscriber(objCenter_topic, objCenter, 
			self.cbObjCenter)

#		# Publish to RegionOfInterest msg
#		roi_topic = "/faceROI"
#		self.roi_pub = rospy.Publisher(roi_topic, RegionOfInterest, queue_size=10)

#		# Publish to objCenter msg
#		objCenter_topic = "/centerROI"
#		self.objCenter_pub = rospy.Publisher(objCenter_topic, objCenter, queue_size=10)

		# Allow up to one second to connection
		rospy.sleep(1)

#	# Convert image to OpenCV format
#	def cbImage(self, msg):

#		try:
#			self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
#		except CvBridgeError as e:
#			print(e)

#		if self.cv_image is not None:
#			self.image_received = True
#		else:
#			self.image_received = False

	# Get CameraInfo
	def cbCameraInfo(self, msg):

		self.imgWidth = msg.width
		self.imgHeight = msg.height

		# calculate the center of the frame as this is where we will
		# try to keep the object
		self.centerX = self.imgWidth // 2
		self.centerY = self.imgHeight // 2

	# Get objCenter coordinates
	def cbObjCenter(self, msg):

		self.objX = msg.centerX
		self.objY = msg.centerY

#	# Show the output frame
#	def cbShowImage(self):

#		cv2.imshow("Haar Face Detector (ROI)", self.cv_image)
#		cv2.waitKey(1)

	# Image information callback
	def cbInfo(self):

#		fontFace = cv2.FONT_HERSHEY_DUPLEX
#		fontScale = 0.5
#		color = (255, 255, 255)
#		thickness = 1
#		lineType = cv2.LINE_AA
#		bottomLeftOrigin = False # if True (text upside down)

#		self.timestr = time.strftime("%Y%m%d-%H:%M:%S")

#		cv2.putText(self.cv_image, "{}".format(self.timestr), (10, 20), 
#			fontFace, fontScale, color, thickness, lineType, 
#			bottomLeftOrigin)
#		cv2.putText(self.cv_image, "(%d, %d)" % (self.objX, self.objY), 
#			(self.imgWidth-100, 20), fontFace, fontScale, 
#			color, thickness, lineType, bottomLeftOrigin)
#		cv2.putText(self.cv_image, "Sample", (10, self.imgHeight-10), 
#			fontFace, fontScale, color, thickness, lineType, 
#			bottomLeftOrigin)
#		cv2.putText(self.cv_image, "(%d, %d)" % (self.imgWidth, self.imgHeight), 
#			(self.imgWidth-100, self.imgHeight-10), fontFace, fontScale, 
#			color, thickness, lineType, bottomLeftOrigin)

		panErr, panOut = self.cbPIDprocess(self.panPID, self.objX, self.centerX)
		tiltErr, tiltOut = self.cbPIDprocess(self.tiltPID, self.objY, self.centerY)

		rospy.loginfo([panErr, panOut])

	def cbPIDprocess(self, pid, objCoord, centerCoord):

		# calculate the error
		error = centerCoord - objCoord

		# update the value
		output = pid.update(error)

		return error, output

#	# Detect the face(s)
#	def cbFace(self):
#		if self.image_received:
#			# find the object's location
#			objectLoc = self.faceCascade.update(self.cv_image, 
#				(self.centerX, self.centerY))
#			((self.objX, self.objY), rect) = objectLoc

#			self.pubObjCenter()

#			# extract the bounding box and draw it
#			if rect is not None:
#				(self.x, self.y, self.w, self.h) = rect
#				cv2.rectangle(self.cv_image, (self.x, self.y), 
#					(self.x + self.w, self.y + self.h), 
#					(0, 255, 0), 2)

#				self.pubRegionofInterest()

#			self.cbInfo()
#			self.cbShowImage()
#		else:
#			rospy.logerr("No images recieved")

#	# Publish to RegionOfInterest msg
#	def pubRegionofInterest(self):

#		self.roi.x_offset = self.x
#		self.roi.y_offset = self.y
#		self.roi.width = self.x + self.w
#		self.roi.height = self.y + self.h

#		self.roi_pub.publish(self.roi)

#	# Publish to objCenter msg
#	def pubObjCenter(self):

#		self.centerRoi.centerX = self.objX
#		self.centerRoi.centerY = self.objY

#		self.objCenter_pub.publish(self.centerRoi)

	# rospy shutdown callback
	def cbShutdown(self):
		try:
			rospy.logwarn("PanTiltTracking (ROI) node [OFFLINE]")
		finally:
			cv2.destroyAllWindows()

if __name__ == '__main__':

	# Initializing your ROS Node
	rospy.init_node('pant_tilt_tracking', anonymous=False)
	pt = PanTiltTracking()

	# Camera preview
	while not rospy.is_shutdown():
		pt.cbInfo()
