#!/usr/bin/env python

"""image_processor.py: Template image processing node."""

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
    def __init__(self):
        self.bridge = CvBridge()
        self.image_subscriber = rospy.Subscriber("image_raw", Image, self.process_message, queue_size=1)
        self.image_publisher = rospy.Publisher("image_processed", Image, queue_size=1)

    def to_cv2(self, image_msg):
        """Convert ROS image message to OpenCV image
        """
        try:
            image = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
        except CvBridgeError, e:
            print(e)
        return image

    def to_imgmsg(self, image):
        """Convert OpenCV image to ROS image message
        """
        try:
            image_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        except CvBridgeError, e:
            print(e)
        return image_msg

    def process_message(self, image_msg):
        """Process received ROS image message
        """
        image = self.to_cv2(image_msg)
        processed_image = self.process_image(image)
        self.image_publisher.publish(self.to_imgmsg(processed_image))

    def process_image(self, image):
        """Process the image using OpenCV

        This is where the magic happens...
        """
        cv2.imshow("image", image)
        processed_image = image
        # -------------------------------------------------

        #
        # TODO: Put your OpenCV image processing code here
        #

        # -------------------------------------------------
        cv2.imshow("processed_image", processed_image)
        cv2.waitKey(3)

        return processed_image


def main(args):
    rospy.init_node('image_processor', anonymous=True)
    ip = ImageProcessor()
    rospy.spin()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
