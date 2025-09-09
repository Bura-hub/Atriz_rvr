#!/usr/bin/env python3

import rospy
import cv2
import std_srvs.srv
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo
import numpy as np
from sensor_msgs.msg import Image
import time
import std_msgs.msg

# ***************************************************
# *** GLOBALS ***
# ***************************************************
bridge = CvBridge()
depth_threshold = None
DEPTH_RES = None
img_depth_stream = None
rvr_stop_service = None
emergency_pub = None


# ***************************************************
# *** CALLBACKS ***
# ***************************************************

def image_depth_callback(data):
    global img_depth_stream, bridge

    try:
        img_depth_stream = bridge.imgmsg_to_cv2(data, "16UC1")
        check_depth_threshold()
    except CvBridgeError as e:
        print(e)


# ***************************************************
# *** FUNCTIONS ***
# ***************************************************

def print_color_at_mouse(event, x, y, flags, param):
    if event == cv2.EVENT_MOUSEMOVE:
        # Retrieve the pixel value at the mouse coordinates
        color = param[y, x]

        # Print the BGR color values
        print("depth: ", color)

def check_depth_threshold():
    global img_depth_stream, depth_threshold, rvr_stop_service, emergency_pub

    # get avg depth middle of image
    depth_img = np.copy(img_depth_stream)
    np.array(depth_img).astype(dtype=np.float32)
    depth_img[depth_img == 0.0] = 1000.0    # invert zeros, otherwise the will lead to false values due to the gaussian blur
    depth_img = np.clip(depth_img, 0, 1000) / 1000.0        # [m]
    # take closest avg points (big gauss map) in center area of img
    depth_img = cv2.GaussianBlur(depth_img, (9, 9), 0)

    #kernel = np.ones((20,20),np.uint8)      # fill some holes
    #depth_img = cv2.morphologyEx(depth_img, cv2.MORPH_OPEN, kernel)

    # crop top and bottom to focus on the middle of the image
    dist = np.min(depth_img[int(DEPTH_RES[0] * 0.2) : int(DEPTH_RES[0] * 0.8), :])

    if dist < depth_threshold:
        emergency_pub.publish()

    #cv2.imshow("IMG_Color", depth_img)
    #cv2.waitKey(1)


if __name__ == "__main__":
    rospy.init_node("emergency_stop")

    # get depth threshold from param server
    depth_threshold = rospy.get_param("~depth_threshold", 0.19)

    cam_depth_info = rospy.wait_for_message("camera/depth/camera_info", CameraInfo)
    DEPTH_RES = (cam_depth_info.height, cam_depth_info.width)
    img_depth_stream = np.zeros((DEPTH_RES[0], DEPTH_RES[1], 1), np.uint8)

    rospy.Subscriber("camera/aligned_depth_to_color/image_raw", Image, image_depth_callback)
    emergency_pub = rospy.Publisher("rvr/is_emergency_stop", std_msgs.msg.Empty, queue_size=1)

    rospy.spin()
