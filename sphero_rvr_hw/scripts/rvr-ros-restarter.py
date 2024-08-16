#!/usr/bin/env python3

import os
import time

import rospy
from std_srvs.srv import Empty, EmptyResponse

pub_odom_as_tf = None
as_simulation = None
as_visualization = None
robot_name = None

def restart_callback(req):
    global pub_odom_as_tf, as_simulation, as_visualization, robot_name

    rospy.loginfo('Restarting rvr ROS interface ...')

    # get rospy namespace
    os.system(  f'export ROS_NAMESPACE={rospy.get_namespace()}/rvr/; ' \
                f'roslaunch sphero_rvr_hw restart_hw_controller.launch ' \
                f'pub_odom_as_tf:={pub_odom_as_tf} as_simulation:={as_simulation} as_visualization:={as_visualization} robot_name:={robot_name} &'
    )

    # kill old node
    #os.system(f"rosnode kill {rospy.get_namespace()}rvr/ros_interface")
    #time.sleep(2)
    #os.system('roslaunch sphero_rvr_hw hw_controller.launch &')

    return EmptyResponse()


if __name__ == '__main__':
    rospy.init_node('rvr-ros-restarter')

    # preserver startup options
    pub_odom_as_tf = rospy.get_param('~pub_odom_as_tf')
    as_simulation = rospy.get_param('~as_simulation')
    as_visualization = rospy.get_param('~as_visualization')
    robot_name = rospy.get_param('~robot_name')

    # register the restart_callback
    rospy.Service('rvr_ros_restarter/restart', Empty, restart_callback)

    rospy.spin()
