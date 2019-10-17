#!/usr/bin/env python

# Receives bounding box information from yolo and exports it
# out as csv

# Created: 13/10/19 - Dustin Haines
# Last Modified: 13/10/19 - Dustin Haines

import os
import rospkg
import rospy
import time
import argparse
import csv
from std_msgs.msg import Header
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes

# Constants
rospack = rospkg.RosPack()
fp = rospack.get_path('yolo_feeders')
OUTPUT_BOUND_DIR = fp + '/../../output_bounds/'
START_TIME = str(int(time.time()))

# Configuration
INPUT_BOX_TOPIC = 'darknet_ros/bounding_boxes'
OUTPUT_FILENAME_SUM = START_TIME + '_Summary.csv'
OUTPUT_FILE_SUM = None
OUTPUT_FILENAME_DET = START_TIME + '_Detailed.csv'
OUTPUT_FILE_DET = None
NODE_NAME = 'box_reader'

# Variables
# Don't want to write out the same original ROSBAG message twice, keep track of the last
last_frame = -1

def bounds_callback(data):
    global last_frame
    # Obtain the sequence number
    frame_num = data.image_header.seq
    
    # Verify we haven't already recorded the interpretation of the frame
    if frame_num > last_frame:
        print '\n New Frame Recieved'
        # Update the last frame we saw
        last_frame = frame_num
        # Build lists of object class and corresponding probability
        # objects_detected = [i.Class for i in data.bounding_boxes]
        # objects_probability = [i.probability for i in data.bounding_boxes]
        # Calculate the number of objects detected and average probability
        # num_objects_detected = len(objects_detected)
        # avg_objects_probability = sum(objects_probability)/num_objects_detected if num_objects_detected > 0  else 0
        # Summarised output
        # OUTPUT_FILE_SUM.writerow([frame_num, num_objects_detected, avg_objects_probability])
        # Detailed output
        for box in data.bounding_boxes:
            OUTPUT_FILE_DET.writerow([frame_num, box.Class, box.probability, box.xmin,box.ymin, box.xmax,box.ymax])
    return

def argument_parser():
    parser = argparse.ArgumentParser()
    # Specify all possible command line arguments
    parser.add_argument('-ib', '--input_box_topic')
    parser.add_argument('-os', '--output_file_sum')
    parser.add_argument('-od', '--output_file_det')
    return parser

def response():
    # Initialise the node
    rospy.init_node(NODE_NAME, anonymous=True)

    # Subscribe to topics, specifying the data type and callback function
    rospy.Subscriber(INPUT_BOX_TOPIC, BoundingBoxes, bounds_callback)
    print 'Listener to /' + INPUT_BOX_TOPIC + ' created'
    print 'Writing to files with prefix: ' + START_TIME
    
    # Keeps the listener alive
    rospy.spin()

if __name__ == '__main__':
    # Parse CLI arguments
    parser = argument_parser()
    args = parser.parse_args()

    # Update configuration if required
    INPUT_BOX_TOPIC = args.input_box_topic if args.input_box_topic else INPUT_BOX_TOPIC
    OUTPUT_FILENAME_SUM = args.output_file_sum if args.output_file_sum else OUTPUT_FILENAME_SUM
    OUTPUT_FILENAME_DET = args.output_file_det if args.output_file_det else OUTPUT_FILENAME_DET

    # Open our csv files
    OUTPUT_FILE_SUM = csv.writer(open(OUTPUT_BOUND_DIR+OUTPUT_FILENAME_SUM, 'w'), delimiter=',')
    OUTPUT_FILE_SUM.writerow(['SEQUENCE_NO', 'OBJECTS_DETECTED', 'AVERAGE_PROB'])
    OUTPUT_FILE_DET = csv.writer(open(OUTPUT_BOUND_DIR+OUTPUT_FILENAME_DET, 'w'), delimiter=',')
    OUTPUT_FILE_DET.writerow(['SEQUENCE_NO', 'OBJECT', 'PROBABILITY', 'MIN_X', 'MIN_Y', 'MAX_X', 'MAX_Y'])

    # Create the node
    response()
