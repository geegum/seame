#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
from datetime import datetime

def image_callback(msg):
    try:
        # Convert ROS Image message to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("Could not convert image: {}".format(e))
        return

    # Get current time for the filename
    current_time = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    filename = os.path.join(save_path, "{}.jpg".format(current_time))

    # Save image to the specified path
    if cv2.imwrite(filename, cv_image):
        rospy.loginfo("Saved image: {}".format(filename))
    else:
        rospy.logerr("Failed to save image: {}".format(filename))

if __name__ == '__main__':
    rospy.init_node('webcam_capture', anonymous=True)

    # Parameters
    image_topic = rospy.get_param("~image_topic", "/usb_cam/image_raw")
    save_path = rospy.get_param("~save_path", "/home/jetson/catkin_ws/video")

    # Ensure the save path exists
    if not os.path.exists(save_path):
        rospy.logwarn("Save path does not exist. Creating: {}".format(save_path))
        os.makedirs(save_path)
    else:
        rospy.loginfo("Save path exists: {}".format(save_path))

    # Initialize the CvBridge class
    bridge = CvBridge()

    # Subscribe to the image topic
    rospy.Subscriber(image_topic, Image, image_callback)

    rospy.loginfo("Webcam capture node started.")
    rospy.spin()

