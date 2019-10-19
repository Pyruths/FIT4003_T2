#!/usr/bin/env python

"""
Feed images to YOLO and listen to bounding box updates. This feeds as many images as the queue can take



"""
# Receives bounding box information from yolo and exports it
# out as csv

# Created: 19/10/19 - Robin Phoeng
# Last Modified: 19/10/19 - Robin Phoeng

import os
import cv2
import rospkg
import rospy
import time
import argparse
import csv
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes

# Constants
rospack = rospkg.RosPack()
fp = rospack.get_path('yolo_feeders')
OUTPUT_BOUND_DIR = fp + '/../../output_bounds/'
IMAGE_DIR = fp + '/../../input_images/'
PROCESSED_DIR = fp + '/../../processed_images/'
START_TIME = str(int(time.time()))
QUERY_RATE = 5.0




# Configuration
VAL_EXTENSIONS = ['png', 'jpg']
OUTPUT_TOPIC = 'camera/rgb/image_raw'
INPUT_BOX_TOPIC = 'darknet_ros/bounding_boxes'
OUTPUT_FILENAME_DET = START_TIME + '_Detailed.csv'
OUTPUT_FILE_DET = None
NODE_NAME_READER = 'box_reader'
NODE_NAME_WRITER = 'image_loader'
queue_size = 1
queue_counter = 0

# Variables
# Don't want to write out the same original ROS message twice, keep track of the last
last_frame = ''

def setup_publisher():
	# Create publishers
	pub = rospy.Publisher(OUTPUT_TOPIC, Image, queue_size=queue_size)
	# Create an anonymous node to avoid any potential conflict
	rospy.init_node(NODE_NAME_WRITER, anonymous=True)
	# Set the rate at which the query is run (in Hz)
	# 1 query a second currently, YOLO should process quicker than this
	# but varies based on hardware
	rate = rospy.Rate(1)
	# Create a bridge instance which will convert out image into the right format
	bridge = CvBridge()

	# Launch the listener loop
	while not rospy.is_shutdown():
        # if there is a free buffer slot
        if queue_counter >= 0:
            print '\nQuerying for images'
            # Query the directory
            dir_contents = os.listdir(IMAGE_DIR)
            # Only do processing if there is an image to process
            for item in dir_contents:
                dir_contents.remove(item) if item.split('.')[-1] not in VAL_EXTENSIONS else None
            if len(dir_contents) > 0:
                # Open the first file
                file_name = dir_contents[0]
                file = cv2.imread((IMAGE_DIR + file_name), -1)
                print 'Image found (' + file_name + '), processing'
                try:
                    # Transform into the message
                    yolo_image = bridge.cv2_to_imgmsg(file)
                    yolo_image.encoding = 'bgr8'
                    yolo_image.header.frame_id = file_name
                    # Publish the message
                    pub.publish(yolo_image)
                except CvBridgeError as err:
                    print(err)
                # Move the processed image
                os.rename((IMAGE_DIR + file_name), (PROCESSED_DIR + file_name))
                print(file_name + ' moved to ' + PROCESSED_DIR)
            else:
                print 'No images found, sleeping...'
		# Sleep for next iteration
        rospy.sleep(QUERY_RATE)

def bounds_callback(data):
    global last_frame
    # Obtain the sequence number
    frame_num = data.image_header.frame_id

    # Verify we haven't already recorded the interpretation of the frame
    if frame_num != last_frame:
        print
        '\n New Frame Recieved :' + frame_num
        # Update the last frame we saw
        last_frame = frame_num
        # Build lists of object class and corresponding probability
        for box in data.bounding_boxes:
            OUTPUT_FILE_DET.writerow([frame_num, box.Class, box.probability, box.xmin, box.ymin, box.xmax, box.ymax])
    return

def setup_subscriber():
    # Initialise the node
    rospy.init_node(NODE_NAME_READER, anonymous=True)
    # Subscribe to topics, specifying the data type and callback function
    rospy.Subscriber(INPUT_BOX_TOPIC, BoundingBoxes, bounds_callback)
    print
    'Listener to /' + INPUT_BOX_TOPIC + ' created'
    print
    'Writing to files with prefix: ' + START_TIME

    # Keeps the listener alive
    rospy.spin()

def argument_parser():
    parser = argparse.ArgumentParser()
    # Specify all possible command line arguments
    parser.add_argument('-ib', '--input_box_topic')
    parser.add_argument('-od', '--output_file_det')
    return parser

if __name__ == '__main__':
    # Parse CLI arguments
    parser = argument_parser()
    args = parser.parse_args()
    # Update configuration if required
    INPUT_BOX_TOPIC = args.input_box_topic if args.input_box_topic else INPUT_BOX_TOPIC
    OUTPUT_FILENAME_DET = args.output_file_det if args.output_file_det else OUTPUT_FILENAME_DET
    # Open our csv files
    OUTPUT_FILE_DET = csv.writer(open(OUTPUT_BOUND_DIR + OUTPUT_FILENAME_DET, 'w'), delimiter=',')
    OUTPUT_FILE_DET.writerow(['FILE_NAME', 'OBJECT', 'PROBABILITY', 'MIN_X', 'MIN_Y', 'MAX_X', 'MAX_Y'])
    # Create the node
    setup_subscriber()
    # run images
    setup_publisher()
