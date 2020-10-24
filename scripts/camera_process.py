#!/usr/bin/env python

import cv2
import numpy as np
import rospy as rp
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class Camera:
    """
    Camera class to publish frames from computer camera and processed frames as detected eges on image.
    """
    def __init__(self):
        self.camera_pub = rp.Publisher("/camera_image", Image, queue_size=10)
        self.edge_pub = rp.Publisher("/edge_image", Image, queue_size=10)
        
    def array2imgmsg(self, img_array):
        """Convert numpy array which contain image to ROS Image message.

        Parameters
        ----------
        img_array : np.array
            Image array.

        Returns
        -------
        Image
            ROS sensor message.
        """
        ### Task - convert image array to ROS Image message
        ### Hint - use CVBridge ROS package to convert cv2 array to Image message.

        img_msg = None

        return img_msg

    def publish_camera_image(self, image):
        """Function to publish camera image as ROS Image message. 

        Parameters
        ----------
        image : np.array
            Image array.
        """
        ### Task - publish Image message for camera image.
        ### Hint - convert cv2 to Image message before publish.

        img_msg = None

        if img_msg is None:
            raise NotImplementedError

        self.camera_pub.publish(img_msg)

    def publish_edge_image(self, edge_image):
        """Function to publish detected edges on image as ROS Image message. 

        Parameters
        ----------
        edge_image : np.array
            Image array.
        """
        ### Task - publish Image message for edge image.
        ### Hint - convert cv2 to Image message before publish.

        img_msg = None

        if img_msg is None:
            raise NotImplementedError

        self.edge_pub.publish(img_msg)


if __name__ == '__main__':
    rp.init_node('camera_viewer', log_level=rp.DEBUG)

    cam = Camera()
    cap = cv2.VideoCapture(0)

    while not rp.is_shutdown():
        ret, frame = cap.read()

        cam.publish_camera_image(frame)

        ### Task - convert image to grayscale and detect edges.
        ### Hint - use Canny edge detector to get edges from image.

        edge = np.zeros_like(frame)

        cam.publish_edge_image(edge)

        rp.sleep(0.1)

    cap.release()