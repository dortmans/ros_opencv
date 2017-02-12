#!/usr/bin/env python

"""ROS OpenCV image processing node."""

# For Python2/3 compatibility
from __future__ import print_function
from __future__ import division

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import sys

__author__ = "Eric Dortmans"
__copyright__ = "Copyright 2016, Fontys"


class ImageProcessor:
    """This class processes ROS Images using OpenCV

    """

    def __init__(self):
        self.bridge = CvBridge()
        self.image_subscriber = rospy.Subscriber("image_raw", Image, self.on_image_message, queue_size=1)
        self.image_publisher = rospy.Publisher("image_processed", Image, queue_size=1)
        self.process_image_setup()

    def to_cv2(self, image_msg):
        """Convert ROS image message to OpenCV image

        """
        try:
            image = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
        except CvBridgeError as e:
            print(e)
        return image

    def to_imgmsg(self, image):
        """Convert OpenCV image to ROS image message

        """
        try:
            image_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        except CvBridgeError as e:
            print(e)
        return image_msg

    def on_image_message(self, image_msg):
        """Process received ROS image message
        """
        self.image = self.to_cv2(image_msg)
        self.processed_image = self.image
        self.process_image()
        self.image_publisher.publish(self.to_imgmsg(self.processed_image))

    def on_mouse_event(self, event, x, y, flags, param):
        """ Handle mouse events

        """
        # -------------------------------------------------

        #
        # TODO: Put OpenCV code here to handle mouse events
        #

        # -------------------------------------------------

    def on_trackbar_change(self, trackbarValue):
        """Handle trackbar change

        """
        # -------------------------------------------------

        #
        # TODO: Put OpenCV code here to handle trackbar change
        #

        # -------------------------------------------------

    def process_image_setup(self):
        """Setup for image processing. 

        This code will run only once to setup image processing.
        """
        cv2.namedWindow('image')
        cv2.setMouseCallback('image', self.on_mouse_event)
        # -------------------------------------------------

        #
        # TODO: Put your OpenCV setup code here. 
        #

        # -------------------------------------------------

    def process_image(self):
        """Process the image using OpenCV

        This code is run for reach image
        """

        cv2.imshow("image", self.image)

        # -------------------------------------------------

        #
        # TODO: Put your OpenCV image processing code here
        #

        # -------------------------------------------------

        cv2.imshow("processed_image", self.processed_image)
        cv2.waitKey(3)


def main(args):
    rospy.init_node('image_processor', anonymous=True)
    ip = ImageProcessor()
    rospy.spin()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
