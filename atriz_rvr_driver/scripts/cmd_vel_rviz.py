#!/usr/bin/env python3
import rospy
import tf_conversions
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from sphero_rvr_msgs.srv import SetPosAndYaw
import math  

class RvizRvR:
	_x = 0.0
	_y = 0.0
	_th = 0.0

	def __init__(self):
		self._timestamp = rospy.Time.now()
		self._br = tf2_ros.TransformBroadcaster()
		self._tfBuffer = tf2_ros.Buffer()
		self._tfListener = tf2_ros.TransformListener(self._tfBuffer)
		self._cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self._handle_cmd_vel, queue_size=1)
		self._odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)

		# init services
		self._handle_pos_and_yaw_service = rospy.Service('set_pos_and_yaw', SetPosAndYaw, self._handle_set_pos_and_yaw)

		# Sendes tf of odom periodically
		self._odom_tf_broadcaster()

	def _handle_set_pos_and_yaw(self, msg):
		self._x = msg.position.x
		self._y = msg.position.y
		self._th = msg.yaw

		# publish _x, _y and _th as tf and odom
		# publish odom publisher msg and tf
		self._publish_odom_msg_and_tf()
		return True

	def _handle_cmd_vel(self, msg):
		if rospy.Time.now() < self._timestamp + rospy.Duration.from_sec(0.1):
			return # block for 100ms
		self._timestamp = rospy.Time.now()
		delta_x = (msg.linear.x * math.cos(self._th) - msg.linear.y * math.sin(self._th)) * 0.1
		delta_y = (msg.linear.x * math.sin(self._th) + msg.linear.y * math.cos(self._th)) * 0.1
		delta_th = msg.angular.z * 0.1

		self._x += delta_x
		self._y += delta_y
		self._th += delta_th 

		# publish odom publisher msg and tf
		self._publish_odom_msg_and_tf(msg)
		
	def _publish_odom_msg(self, twist: Twist = Twist()):
		odom = Odometry()
		# position
		odom.pose.pose.position.x = self._x
		odom.pose.pose.position.y = self._y
		odom.pose.pose.position.z = 0.0
		odom.pose.pose.orientation = _quaternion_to_msg(self._th)

		# set velocity
		odom.child_frame_id = 'rvr_base_link'
		odom.twist.twist.linear.x = twist.linear.x
		odom.twist.twist.linear.y = twist.linear.y
		odom.twist.twist.linear.z = twist.angular.z

		self._odom_pub.publish(odom)
	
	def _publish_odom_tf(self):
		t = TransformStamped()
		t.header.stamp = rospy.Time.now()
		t.header.frame_id = "odom"
		t.child_frame_id = "rvr_base_link"
		t.transform.translation.x = self._x
		t.transform.translation.y = self._y
		t.transform.translation.z = 0
		t.transform.rotation = _quaternion_to_msg(self._th)
		self._br.sendTransform(t)
	
	def _publish_odom_msg_and_tf(self, twist: Twist = Twist()):
		self._publish_odom_msg(twist)	#publish odom as msg
		self._publish_odom_tf()		#publish tf transform


	def _odom_tf_broadcaster(self):
		r = rospy.Rate(60) # 0.1 Hz (1/0.1=10s) 
		while not rospy.is_shutdown():
			self._publish_odom_msg_and_tf()
			r.sleep()

def _quaternion_to_msg(th: float) -> Quaternion:
	q = tf_conversions.transformations.quaternion_from_euler(0, 0, th)
	return Quaternion(*q)

	

if __name__ == '__main__':
	rospy.init_node('rviz_rvr')

	rviz_rvr = RvizRvR()

	rospy.spin()
