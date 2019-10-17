#!/usr/bin/env python

# Receives images from YOLO_V3 and stores the results in a directory
# Output is augmented images with bounding boxes and a text file defining
# the bounds

# Created: 22/09/19 - Dustin Haines
# Last Modified: 22/09/19 - Dustin Haines

import os
import cv2
import rospkg
import rospy
import time
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
#from darknet_ros_msgs import BoundingBox, BoundingBoxes

# Constant directory locations
rospack = rospkg.RosPack()
fp = rospack.get_path('yolo_feeders')
OUTPUT_IMAGE_DIR = fp + '/../../output_images/'
OUTPUT_BOUND_DIR = fp + '/../../output_bounds/'

# Configuration
INPUT_IMAGE_TOPIC = 'darknet_ros/detection_image'
#INPUT_IMAGE_TOPIC = 'camera/rgb/image_raw'
INPUT_BOUND_TOPIC = '/'
NODE_NAME = 'image_response'

# Variables
# Don't want to write out the same original ROSBAG message twice, keep track of the last
last_frame = -1

def images_callback(data):
    global last_frame
    # Obtain the sequence number
    frame_num = data.image_header.seq
    if frame_num > last_frame:
        last_frame = frame_num
        print '\n New Image received'
        # Create a CvBridge instance to convert
        bridge = CvBridge()
        # Take the sensor_msgs Image and convert back to cv2
        cv_image = bridge.imgmsg_to_cv2(data)
        # Save the file to the output directory. Give system time as filename
        filename = str(int(time.time())) + '.png'
        cv2.imwrite(OUTPUT_IMAGE_DIR + filename, cv_image)
        print 'Image saved as ' + filename

def bounds_callback(data):
    return

def response():
    # Initialise the node
    rospy.init_node(NODE_NAME, anonymous=True)

    # Subscribe to topics, specifying the data type and callback function
    rospy.Subscriber(INPUT_IMAGE_TOPIC, Image, images_callback)
    print 'Listener to /' + INPUT_IMAGE_TOPIC + ' created'
    #rospy.Subscriber(INPUT_BOUND_TOPIC, BoundingBoxes, bounds_callback)
    #print 'Listener to /' + INPUT_BOUND_TOPIC + ' created'
    
    # Keeps the listener alive
    rospy.spin()

if __name__ == '__main__':
    response()
