#!/usr/bin/env python3

import rospy
from sphero_rvr_msgs.msg import Color  # Reemplaza 'your_package' con el nombre del paquete que define el mensaje 'Color'

def color_callback(msg):
    rospy.loginfo("Color detected: R: {}, G: {}, B: {}, Confidence: {}".format(
        msg.rgb_color[0], msg.rgb_color[1], msg.rgb_color[2], msg.confidence
    ))

def listener():
    rospy.init_node('color_listener', anonymous=True)
    rospy.Subscriber('color', Color, color_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
