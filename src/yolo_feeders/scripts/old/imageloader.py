#!/usr/bin/env python

# Simple image listener that looks at ../../../input_images 
# Loads images found, and publishes them to /camera/rgb/image_raw
# for YOLO_v3 to process
# Can supply 'filter' argument to apply filters before YOLO processing
# Options: filter (optional),

# Created: 22/09/19 - Dustin Haines
# Last Modified: 19/10/19 - Robin Phoeng

import sys
import os
import cv2
import rospkg
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

# Constant directory locations
rospack = rospkg.RosPack()
fp = rospack.get_path('yolo_feeders')
IMAGE_DIR = fp + '/../../input_images/'
PROCESSED_DIR = fp + '/../../processed_images/'

# Configuration
VAL_EXTENSIONS = ['png', 'jpg']
OUTPUT_TOPIC = 'camera/rgb/image_raw'
NODE_NAME = 'image_loader'
QUERY_RATE = 5.0

def findImage():
	# Create publishers
	pub = rospy.Publisher(OUTPUT_TOPIC, Image, queue_size=1)
	# Create an anonymous node to avoid any potential conflict
	rospy.init_node(NODE_NAME, anonymous=True)
	# Set the rate at which the query is run (in Hz)
	# 1 query a second currently, YOLO should process quicker than this
	# but varies based on hardware
	rate = rospy.Rate(1)
	# Create a bridge instance which will convert out image into the right format
	bridge = CvBridge()

	# Launch the listener loop
	while not rospy.is_shutdown():
		print '\nQuerying for images'
		# Query the directory
		dir_contents = os.listdir(IMAGE_DIR)
		# Only do processing if there is an image to process
		for item in dir_contents:
			dir_contents.remove(item) if item.split('.')[-1] not in VAL_EXTENSIONS else None
		if len(dir_contents) > 0:
			# Open the first file
			first_file_name = dir_contents[0]
			first_file = cv2.imread((IMAGE_DIR + first_file_name), -1)
			print 'Image found (' + first_file_name + '), processing'
			try:
				# Transform into the message
				yolo_image = bridge.cv2_to_imgmsg(first_file)
				yolo_image.encoding = 'bgr8'
				yolo_image.header.frame_id = first_file_name
				# Publish the message
				pub.publish(yolo_image)
			except CvBridgeError as err:
				print(err)
			# Move the processed image
			os.rename((IMAGE_DIR + first_file_name), (PROCESSED_DIR + first_file_name))
			print(first_file_name + ' moved to ' + PROCESSED_DIR)
		else:
			print 'No images found, sleeping...'
		# Sleep for next iteration
		rospy.sleep(QUERY_RATE)

if __name__ == '__main__':
	print 'Image Loader launching with following values'
	print 'IMAGE_DIR: ' + IMAGE_DIR
	print 'PROCESSED_DIR: ' + PROCESSED_DIR
	print 'VAL_EXTENSIONS: ' + str(VAL_EXTENSIONS)
	print 'OUTPUT_TOPIC: ' + OUTPUT_TOPIC
	print 'NODE_NAME: ' + NODE_NAME
	print 'QUERY_RATE: ' + str(QUERY_RATE)
	print '\n'
    
	try:
		findImage()
	except rospy.ROSInterruptException:
		print '\nShutting down Image Loader'
