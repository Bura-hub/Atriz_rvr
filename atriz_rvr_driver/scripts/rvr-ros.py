#!/usr/bin/env python3

# etc
import copy
from math import pi
import time
import os
import sys
import logging
import asyncio
import traceback

import std_msgs.msg

# sphero
from sphero_sdk.asyncio.client.toys.sphero_rvr_async import SpheroRvrAsync
from sphero_sdk import SerialAsyncDal
from sphero_sdk import RvrStreamingServices
from sphero_sdk import RawMotorModesEnum
from sphero_sdk import RvrLedGroups
from sphero_sdk.common.enums.power_enums import BatteryVoltageStatesEnum as VoltageStates
from sphero_sdk.common.enums.drive_enums import DriveFlagsBitmask
from sphero_sdk import Colors
from sphero_sdk.common.log_level import LogLevel
import signal
# ROS
import tf
import tf2_ros
import tf2_geometry_msgs
import rospy
import geometry_msgs
from geometry_msgs.msg import PoseWithCovariance, \
                                Pose, \
                                TwistWithCovariance, \
                                Twist, \
                                Point, \
                                Quaternion, \
                                Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from std_msgs.msg import String
from sphero_rvr_msgs.srv import MoveToPose, MoveToPoseResponse, \
                                MoveToPosAndYaw, MoveToPosAndYawResponse, \
                                BatteryState, BatteryStateResponse, \
                                TriggerLedEventRequest
from sphero_rvr_msgs.msg import Color
from sensor_msgs.msg import Illuminance
import std_srvs.srv
from std_msgs.msg import Bool
import rvr_tools



# * ----------------------------------------------
# *** GLOBALS
# * ----------------------------------------------

# rvr
loop = asyncio.get_event_loop()
rvr = SpheroRvrAsync(
    dal=SerialAsyncDal(loop)
)
rvr_color_picker = rvr_tools.RVRColorPicker(rvr)
#logging.basicConfig(level=logging.DEBUG)

# tf2
tfBuffer = tf2_ros.Buffer()

# movement
received_components = set()
movement_complete = False
movement_success = False

# odom
pub_tf = True
num_msgs_received = {}
robot_pose = Pose()
robot_twist = Twist()
odom = Odometry(
    header=Header(
        frame_id='odom',
    ),
    child_frame_id='rvr_base_link',
)

# sensors
yaw_north = 0                   # the yaw offset from odom to magnetic North in degree
calibration_completed = False   # Flag used to indicate that calibration is complete
color_enabled = False           # Flag indicates that the color sensor is enabled7

is_driving_with_cmd_vel = False  # Flag indicates that the robot is currently driving with cmd_vel
last_cmd_vel_time = 0            # Time of the last cmd_vel message
cmd_vel_timeout = 0.3            # Timeout in seconds after which the robot stops if no cmd_vel message is received
is_in_emergency_stop = False     # Flag indicates that the robot is in emergency stop mode

# publisher
pub_odom = None
br = None
pub_light = None
pub_color = None
pub_magnet = None


# * ----------------------------------------------
# *** CALLBACKS
# * ----------------------------------------------

def release_emergency_stop_callback(req):
    global is_in_emergency_stop

    set_emergency_stop(False)
    rospy.loginfo("({}) emergency stop released".format(rospy.get_name()))

    return std_srvs.srv.EmptyResponse()


def emergency_stop_callback(req):
    """
    TODO maybe move a little bit backwards
    # The first time this is triggered, set a flag that the robot is in emergency stop mode
    # this will prevent any further movement. The only way to get out of this mode is to reset odom
    # when the flag is set, let the robot blink red for a short time
    """
    global is_in_emergency_stop, rvr_color_picker

    if not is_in_emergency_stop:
        # stop current movement
        asyncio.run(rvr.drive_stop())
        set_emergency_stop(True)

        # set color to red
        asyncio.run(rvr_color_picker.trigger_event(TriggerLedEventRequest.EMERGENCY_STOP))

        rospy.loginfo("({}) EMERGENCY STOP TRIGGERED!".format(rospy.get_name()))


def reset_odom_callback(req):
    rospy.loginfo("({}) Resetting odom".format(rospy.get_name()))
    asyncio.run(reset_odom())

    return std_srvs.srv.EmptyResponse()


def move_to_pos_and_yaw_callback(req):
    orientation_q = tf.transformations.quaternion_from_euler(0., 0., req.yaw, axes="sxyz")
    pose = geometry_msgs.msg.Pose()
    pose.position = req.position
    pose.orientation.x = orientation_q[0]
    pose.orientation.y = orientation_q[1]
    pose.orientation.z = orientation_q[2]
    pose.orientation.w = orientation_q[3]

    success = move_to_pose(req.header.frame_id, pose.position, pose.orientation, req.speed, req.speed_in_si)

    return MoveToPosAndYawResponse(success)


def move_to_pose_callback(req):
    success = move_to_pose(req.header.frame_id, req.pose.position, req.pos.orientation, req.speed, req.speed_in_si)

    return MoveToPoseResponse(success)


def cmd_vel_callback(cmdtwist):
    global is_in_emergency_stop

    if is_in_emergency_stop:
        return

    v_x = cmdtwist.linear.x
    v_th = cmdtwist.angular.z       # in [rad/s]

    # the length from the center to the middle of a wheel of the rvr
    radius = 0.1825 / 2
    #radius = 0.09
    #vel_linear = v_th / (2*pi * radius)    # divide by two because we have two motors   in [m/s]
    vel_linear = v_th * radius

    # TODO add a tolerance to approximate the slippage
    l = v_x - vel_linear
    r = v_x + vel_linear

    asyncio.run(write_motors_si(l, r))


def calibrate_magnetometer_callback(req):
    res = std_srvs.srv.TriggerResponse()
    res.success = asyncio.run(calibrate_magnetometer(timeout=10.0))

    return res


def battery_state_callback(req):
    res = BatteryStateResponse()

    battery_percentage = asyncio.run(rvr.get_battery_percentage(timeout=.1))
    asyncio.run(asyncio.sleep(.1))
    res.battery_percentage = battery_percentage["percentage"]

    rospy.loginfo('({}) Battery percentage: {}'.format(rospy.get_name(), battery_percentage))

    battery_voltage_state = asyncio.run(rvr.get_battery_voltage_state(timeout=.1))
    voltage_string = ""
    if battery_voltage_state["state"] == VoltageStates.unknown.value:
        voltage_string = VoltageStates.unknown.name
    elif battery_voltage_state["state"] == VoltageStates.ok.value:
        voltage_string = VoltageStates.ok.name
    elif battery_voltage_state["state"] == VoltageStates.low.value:
        voltage_string = VoltageStates.low.name
    elif battery_voltage_state["state"] == VoltageStates.critical.value:
        voltage_string = VoltageStates.critical.name

    res.voltage_state = voltage_string

    rospy.loginfo('({}) Voltage state: {} [{}]'.format(rospy.get_name(), voltage_string, battery_voltage_state))

    return res


def enable_color_callback(req):
    global color_enabled

    asyncio.run(rvr.enable_color_detection(is_enabled=req.data))
    asyncio.run(asyncio.sleep(.1))
    color_enabled = req.data

    rospy.loginfo("({}) color sensor enabled set to: {}".format(rospy.get_name(), req.data))

    return True, ""


def wake_callback(req):
    rospy.logwarn("({}) This function is not in use anymore since waking the rvr often leads to trouble with the serial connection")
    return
    #asyncio.run(rvr.wake())

    #asyncio.run(rvr_robot(reset_odom=True))
    asyncio.run(rvr_robot(reset_odom=False))
    print("Waking done!")       # TODO debug, remove!
    asyncio.run(asyncio.sleep(0.1))

    return std_srvs.srv.EmptyResponse()


def sleep_callback(req):
    rospy.logwarn("({}) This function is not in use anymore since waking the rvr often leads to trouble with the serial connection")
    return
    asyncio.run(rvr.sleep())
    rospy.loginfo("({}) rvr put to soft sleep".format(rospy.get_name()))
    asyncio.run(asyncio.sleep(0.1))

    return std_srvs.srv.EmptyResponse()


# * ----------------------------------------------
# *** SENSOR HANDLER
# * ----------------------------------------------

# Handler for completion of calibration
async def on_calibration_complete_notify_handler(response):
    """
    This handler is called when the magnetometer calibration is completed.
    The calibration_completed flag will be set and the offset angle from the odom frame to the magnetic north is set as yaw_north
    """
    global calibration_completed, yaw_north

    rospy.loginfo('({}) Calibration complete, response:{}'.format(rospy.get_name(), response))
    yaw_north = response['yaw_north_direction']
    calibration_completed = True


def check_if_need_to_send_msg(component):
    """
    Updates the odom topic. A new message is published when all necessary components are received from the rvr at least once
    :param component: the name of the received component
    :return:
    """
    global robot_pose, odom, received_components

    # TODO also publish velocities
    received_components.add(component)
    if received_components >= {'locator','quaternion','gyroscope','velocity'}:

        received_components.clear()
        odom.header.stamp = rospy.Time.now()

        try:
            pub_odom.publish(odom)
            if pub_tf:
                br.sendTransform(
                    (robot_pose.position.x, robot_pose.position.y, robot_pose.position.z),
                    (robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w),
                    odom.header.stamp,
                    'rvr_base_link',
                    'odom'
                )
        except Exception:
            traceback.print_exc()


async def locator_handler(locator_data):
    global robot_pose

    robot_pose.position.x = locator_data['Locator']['X'] #* 16000.0 / MAX_SENSOR_VALUE
    robot_pose.position.y = locator_data['Locator']['Y'] #* 16000.0 / MAX_SENSOR_VALUE
    robot_pose.position.z = 0

    odom.pose.pose.position = robot_pose.position

    check_if_need_to_send_msg('locator')


async def gyroscope_handler(gyroscope_data):
    global robot_twist

    robot_twist.angular.x = gyroscope_data['Gyroscope']['X'] * 2 * pi / 360
    robot_twist.angular.y = gyroscope_data['Gyroscope']['Y'] * 2 * pi / 360
    robot_twist.angular.z = gyroscope_data['Gyroscope']['Z'] * 2 * pi / 360

    odom.twist.twist.angular = robot_twist.angular

    check_if_need_to_send_msg('gyroscope')


async def velocity_handler(velocity_data):
    global robot_twist

    robot_twist.linear.x = velocity_data['Velocity']['X'] #* 5.0 / MAX_SENSOR_VALUE
    robot_twist.linear.y = velocity_data['Velocity']['Y'] #* 5.0 / MAX_SENSOR_VALUE

    odom.twist.twist.linear = robot_twist.linear

    check_if_need_to_send_msg('velocity')


async def imu_handler(imu_data):
    global robot_pose

    # convert to quaternion
    # orientation_q = tf.transformations.quaternion_from_euler(
    #     imu_data['IMU']['Roll'] * (pi / 180), imu_data['IMU']['Pitch'] * (pi / 180), imu_data['IMU']['Yaw'] * (pi / 180), axes='sxyz')
    # TODO on at least some rvr+ there seems to be an imu offset in x/y orientation
    orientation_q = tf.transformations.quaternion_from_euler(
        0, 0, imu_data['IMU']['Yaw'] * (pi / 180), axes='sxyz')
    robot_pose.orientation.x = orientation_q[0]
    robot_pose.orientation.y = orientation_q[1]
    robot_pose.orientation.z = orientation_q[2]
    robot_pose.orientation.w = orientation_q[3]

    odom.pose.pose.orientation = robot_pose.orientation

    check_if_need_to_send_msg('quaternion')


async def color_handler(color_data):
    global pub_color, color_enabled

    if not color_enabled:
        return

    detected_color = [color_data["ColorDetection"]["R"], color_data["ColorDetection"]["G"], color_data["ColorDetection"]["B"]]

    msg = Color()
    msg.confidence = color_data["ColorDetection"]["Confidence"]
    msg.rgb_color = detected_color
    pub_color.publish(msg)


async def light_handler(light_data):
    global pub_light

    msg = Illuminance()
    msg.header.stamp = rospy.Time()
    msg.illuminance = light_data["AmbientLight"]["Light"]
    msg.variance = 0        # 0 means unknown
    pub_light.publish(msg)


async def on_motor_stall_handler(data):
    print(data)


async def on_motor_fault_handler(data):
    print(data)


async def handle_ros():
    global last_cmd_vel_time, is_driving_with_cmd_vel, cmd_vel_timeout

    # cmd_vel timeout watchdog
    if is_driving_with_cmd_vel and time.time() - last_cmd_vel_time > cmd_vel_timeout:
        await write_motors_si(0, 0)     # this seems to also stop other motions of the rvr

    await asyncio.sleep(0.1)


async def will_sleep_handler():
    """
    This handler is triggered 10sec before the rvr enters soft sleep.
    Since we have encountered problems with the serial communication after entering and releasing soft sleep multiple times,
    we prevent the rvr from entering soft sleep.
    """
    rospy.loginfo("({}) preventing rvr from going to soft sleep".format(rospy.get_name()))
    await rvr.wake()
    await asyncio.sleep(.1)


def on_xy_handler(success):
    """
    This handler is called when the rvr reaches its desired position or an internal timeout is reached.
    The global on_pos signal is set to indicated that the robot is done with its motion.
    The usage of this function is to set on_pos to false before starting an async motion and then wait until this function is called
    :param success: true if the rvr is on its desired pose
    """
    global movement_complete, movement_success

    movement_complete = True
    movement_success = success["success"]
    rospy.loginfo("({}) on pos: {}".format(rospy.get_name(), movement_success))


async def reset_odom():
    """
    Reset the odom of the rvr to the current location.
    """
    global is_in_emergency_stop, rvr_color_picker

    rospy.loginfo("({}) Reset odom".format(rospy.get_name()))

    set_emergency_stop(False)
    await rvr.reset_yaw()
    await asyncio.sleep(.1)
    await rvr.reset_locator_x_and_y()
    await asyncio.sleep(.1)
    await rvr_color_picker.trigger_event(TriggerLedEventRequest.START_DRIVING)


async def rvr_robot():
    """
    Resets the rvr. This includes the registration of all signal handlers
    :param reset_odom: if true the translation from odom to the robots base_link will be set to the current pose.
    """
    global pub_odom, br, rvr_color_picker

    await rvr.wake()
    # Give RVR time to wake up
    await asyncio.sleep(5)
    await reset_sensors()
    await reset_odom()


    # set default color profile
    await rvr_color_picker.trigger_event(TriggerLedEventRequest.STARTUP)

    rospy.loginfo("({}) Sphero RVR HW Python node ready".format(rospy.get_name()))


async def reset_sensors():
    """
    Initialize sensor streams.
    This has to be done every time the rvr is woken up.
    However, this routine seems to crash the serial connection to the rvr sometimes and so shouldn't be called too often.
    We prevent the rvr from falling into soft sleep so that this just has to be called during startup.
    """
    rospy.loginfo("({}) Waking Sphero. Initializing Sensor Streams...".format(rospy.get_name()))

    await rvr.sensor_control.clear()
    await rvr.sensor_control.stop()

    await rvr.sensor_control.add_sensor_data_handler(
        service=RvrStreamingServices.locator,
        handler=locator_handler,
    )
    await rvr.sensor_control.add_sensor_data_handler(
        service=RvrStreamingServices.gyroscope,
        handler=gyroscope_handler,
    )
    await rvr.sensor_control.add_sensor_data_handler(
        service=RvrStreamingServices.velocity,
        handler=velocity_handler,
    )
    await rvr.sensor_control.add_sensor_data_handler(
        service=RvrStreamingServices.imu,
        handler=imu_handler,
    )
    await rvr.sensor_control.add_sensor_data_handler(
        service=RvrStreamingServices.ambient_light,
        handler=light_handler,
    )
    await rvr.on_xy_position_drive_result_notify(
        handler=on_xy_handler
    )
    await rvr.enable_motor_stall_notify(is_enabled=True)
    await rvr.on_motor_stall_notify(
        handler=on_motor_stall_handler
    )
    await rvr.enable_motor_fault_notify(is_enabled=True)
    await rvr.on_motor_fault_notify(
        handler=on_motor_fault_handler
    )
    await rvr.on_will_sleep_notify(
        handler=will_sleep_handler
    )

    await rvr.sensor_control.start(interval=250)


def sig_handler(_signo, _stack_frame):
    rvr.sensor_control.clear(),
    rvr.close()
    print("...done")
    print("RVR program terminated clean.")
    sys.exit(0)


# * ----------------------------------------------
# *** CUSTOM FUNCTIONS
# * ----------------------------------------------

def set_emergency_stop(is_enabled):
    global is_in_emergency_stop

    is_in_emergency_stop = is_enabled
    rospy.set_param("emergency_stop", is_in_emergency_stop)


def message_to_quaternion(orientation):
    orientation_q = [orientation.x,
                     orientation.y,
                     orientation.z,
                     orientation.w]

    return orientation_q


async def calibrate_magnetometer(timeout=None):
    """
    A routine to calibrate the magnetometer.
    Check on_calibration_complete_notify_handler for more information about the result of the calibration.
    Note that the rvr will spin very fast during the calibration
    :param timeout: a timeout in seconds for the calibration routine. None is interpreted as infinite
    :return: True if the calibration was successful
    """
    global calibration_completed, is_in_emergency_stop

    if is_in_emergency_stop:
        return

    # Register for the async on completion of calibration
    calibration_completed = False
    await rvr.on_magnetometer_calibration_complete_notify(handler=on_calibration_complete_notify_handler)
    await rvr.magnetometer_calibrate_to_north(timeout=timeout)
    # wait at last some time for the notifier
    start = time.time()
    while not calibration_completed:
        await asyncio.sleep(.1)
        if time.time() - start >= timeout:
            rospy.logerr("({}) magnetometer calibration timeout".format(rospy.get_name()))
            break

    return calibration_completed


def wait_until_motion_complete():
    """
    waits until the rvr signals that a motion is completed
    :return: True if the desired pose could be reached
    """
    global movement_complete, movement_success, is_in_emergency_stop

    # wait until motion complete
    rate = rospy.Rate(10)
    while not movement_complete and not is_in_emergency_stop:
        rate.sleep()
    movement_complete = False

    return movement_success


async def write_motors_si(l_vel, r_vel):
    """
    Turns asynchron the left and right wheel of the rvr with the given speed in [m/s].
    The rvr is buffering a drive command for two seconds, however,
    here a watchdog is installed such that the rvr is stopping if no new command is received for cmd_vel_timeout seconds.
    :param l_vel: velocity of the left wheel
    :param r_vel: velocity of the right wheel
    """
    global last_cmd_vel_time, is_driving_with_cmd_vel, is_in_emergency_stop

    if is_in_emergency_stop:
        return

    last_cmd_vel_time = time.time()     # set last cmd_vel command timestamp

    if r_vel == 0 and l_vel == 0:       # check if the robot is commanded to stop
        is_driving_with_cmd_vel = False
    else:
        is_driving_with_cmd_vel = True

    await rvr.drive_tank_si_units(l_vel, r_vel)
    await asyncio.sleep(.1)
    #await rvr.drive_rc_si_units(yaw_vel, lin_vel, flags=0)


async def write_motors(l_mode, l_speed, r_mode, r_speed):
    """
    Turns asynchron the left and right wheel of the rvr with a discrete speed [0..255] and a mode for direction
    :param l_mode: use the RawMotorModesEnum.forward.value or RawMotorModesEnum.backward.value of the sphero_sdk
    :param l_speed: speed of the left wheel
    :param r_mode:use the RawMotorModesEnum.forward.value or RawMotorModesEnum.backward.value of the sphero_sdk
    :param r_speed: speed of the right wheel
    """
    global is_in_emergency_stop

    if is_in_emergency_stop:
        return

    await rvr.raw_motors(l_mode, l_speed, r_mode, r_speed)
    await asyncio.sleep(.1)


def move_to_pose(frame_id, pos, ori, speed, speed_in_si):
    """
    Moves the robot to the given pose and orientation with a certain speed.
    Discrete speed and speed in m/s is available (check the README for more information)
    :param frame_id: the frame id of the given position and orientation
    :param pos: the desired position as array
    :param ori: the desired orientation as array
    :param speed: the desired speed
    :param speed_in_si: if true the speed is interpreted in si units. Otherwise, as integer value [0..128]
    :return: true if the desired pose could be reached
    """
    global tfBuffer, on_pos, is_in_emergency_stop

    if is_in_emergency_stop:
        return

    # stop any cmd_vel commands, otherwise the automatic cmd_vel stop will interfere with this movement
    if is_driving_with_cmd_vel:
        asyncio.run(write_motors_si(0, 0))

    # Setting the position of the rvr is a bit complicated since the robot moves in x,y relative to its current position,
    # but the rotation is encoded relative to the initial position (so world in this case)

    # orientation needs to be set relative to world, position relative to the base
    try:
        trans_base_world = tfBuffer.lookup_transform('world', 'rvr_base_link', rospy.Time())            # transform the pos to spheros world axis aligned heading
        trans_frame_id_base = tfBuffer.lookup_transform('rvr_base_link', frame_id, rospy.Time()) # transform the req.pos to base_link
        trans_frame_id_world = tfBuffer.lookup_transform('world', frame_id, rospy.Time())    # transform the req.ori to world (because the spheros heading is in world)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logerr("({}) ERROR while looking for tf2 frame transform".format(rospy.get_name()))
        return False

    # set req.pos relative to base_link
    pos_s = geometry_msgs.msg.PoseStamped()
    pos_s.pose.position = pos
    pos_s.pose.orientation.w = 1
    pos_transformed = tf2_geometry_msgs.do_transform_pose(pos_s, trans_frame_id_base)

    # rotate req.pos by -current_robot_heading
    trans_base_world.transform.translation.x = 0.
    trans_base_world.transform.translation.y = 0.
    trans_base_world.transform.translation.z = 0.
    pos_transformed = tf2_geometry_msgs.do_transform_pose(pos_transformed, trans_base_world)

    # rotate req.ori relative to world
    ori_s = geometry_msgs.msg.PoseStamped()
    ori_s.pose.orientation = ori
    ori_transformed = tf2_geometry_msgs.do_transform_pose(ori_s, trans_frame_id_world)
    (row, pitch, yaw) = tf.transformations.euler_from_quaternion(message_to_quaternion(ori_transformed.pose.orientation), axes="sxyz")
    yaw_d = yaw * (180 / pi)

    # enable backwards movement
    drive_flags = DriveFlagsBitmask.fast_turn
    if speed < 0:
        drive_flags |= DriveFlagsBitmask.drive_reverse

    # call rvr backend
    # if speed in si is set call the non normalized version
    success = True
    rospy.loginfo("({}) Moving to base_link pose: x:{:1.3f} y:{:1.3f} yaw:{:1.3f}".format(rospy.get_name(), pos_transformed.pose.position.x, pos_transformed.pose.position.y, yaw_d))
    if speed_in_si:
        rospy.loginfo("({}) speed(si):{:1.3f} m/s".format(rospy.get_name(), speed))
        asyncio.run(rvr.drive_to_position_si(yaw_d, pos_transformed.pose.position.x, pos_transformed.pose.position.y, speed, flags=drive_flags))
        asyncio.run(asyncio.sleep(.1))
        success = wait_until_motion_complete()
        if not success:
            # somtimes the rvr is not turning at the end, so we do it again in this case
            asyncio.run(rvr.drive_to_position_si(yaw_d, 0.0, 0.0, speed, flags=DriveFlagsBitmask.fast_turn))
            asyncio.run(asyncio.sleep(.1))
            success = wait_until_motion_complete()
    else:
        if int(speed) < 0 or int(speed) > 128:
            rospy.logerr("({}) ERROR: The normalized speed must me 0 >= speed >= 128".format(rospy.get_name()))
            success = False
        else:
            rospy.loginfo("({}) speed(normalized):{}".format(rospy.get_name(), int(speed)))
            asyncio.run(rvr.drive_to_position_normalized(int(yaw_d), pos_transformed.pose.position.x, pos_transformed.pose.position.y, int(speed), flags=drive_flags))
            asyncio.run(asyncio.sleep(.1))
            success = wait_until_motion_complete()
            if not success:
                # somtimes the rvr is not turning at the end, so we do it again in this case
                asyncio.run(rvr.drive_to_position_normalized(int(yaw_d), 0, 0, int(speed), flags=DriveFlagsBitmask.fast_turn))
                asyncio.run(asyncio.sleep(.1))
                success = wait_until_motion_complete()

    return success


def publish_magnet():
    global pub_magnet

    if calibration_completed:
        north_pose_s = geometry_msgs.msg.PoseStamped()
        north_pose_s.header.frame_id = "odom"

        q = tf.transformations.quaternion_from_euler(0, 0, yaw_north * (pi / 180.))
        north_pose_s.pose.orientation.x = q[0]
        north_pose_s.pose.orientation.y = q[1]
        north_pose_s.pose.orientation.z = q[2]
        north_pose_s.pose.orientation.w = q[3]

        # transform to base_frame
        try:
            trans_odom_base = tfBuffer.lookup_transform('rvr_base_link', 'odom', rospy.Time())            # transform the pos to spheros world axis aligned heading
            north_pose_s = tf2_geometry_msgs.do_transform_pose(north_pose_s, trans_odom_base)
            north_pose_s.header.stamp = rospy.get_rostime()
            north_pose_s.header.frame_id = "rvr_base_link"
            north_pose_s.pose.position.x = 0
            north_pose_s.pose.position.y = 0
            north_pose_s.pose.position.z = 0
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("({}) ERROR while looking for tf2 frame transform".format(rospy.get_name()))
            return

        pub_magnet.publish(north_pose_s)


if __name__ == '__main__':
    rospy.init_node('rvr_ros_interface', disable_signals=True)
    rospy.loginfo("({}) Starting Sphero RVR HW Python node...".format(rospy.get_name()))

    pub_tf = rospy.get_param('~pub_tf', True)

    # init rvr
    # signals to be handled
    rospy.loginfo("({}) Registering signal handler...".format(rospy.get_name()))
    signal.signal(signal.SIGINT,  sig_handler)
    signal.signal(signal.SIGHUP,  sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    # tf2
    listener = tf2_ros.TransformListener(tfBuffer)
    br = tf.TransformBroadcaster()

    # topics
    pub_odom = rospy.Publisher('odom', Odometry, queue_size=10)
    pub_light = rospy.Publisher('ambient_light', Illuminance, queue_size=10)
    pub_color = rospy.Publisher('color', Color, queue_size=10)
    pub_magnet = rospy.Publisher('magnet', geometry_msgs.msg.PoseStamped, queue_size=10)

    rospy.Subscriber('cmd_vel', Twist, cmd_vel_callback, queue_size=1)
    rospy.Subscriber("is_emergency_stop", std_msgs.msg.Empty, emergency_stop_callback)

    # services
    rospy.Service('move_to_pose', MoveToPose, move_to_pose_callback)
    rospy.Service('move_to_pos_and_yaw', MoveToPosAndYaw, move_to_pos_and_yaw_callback)
    rospy.Service('enable_color', std_srvs.srv.SetBool, enable_color_callback)
    rospy.Service('calibrate_magnetometer', std_srvs.srv.Trigger, calibrate_magnetometer_callback)
    rospy.Service('battery_state', BatteryState, battery_state_callback)
    #rospy.Service('wake', std_srvs.srv.Empty, wake_callback)
    #rospy.Service('sleep', std_srvs.srv.Empty, sleep_callback)
    rospy.Service('reset_odom', std_srvs.srv.Empty, reset_odom_callback)
    rospy.Service('release_emergency_stop', std_srvs.srv.Empty, release_emergency_stop_callback)

    set_emergency_stop(False)

    # the mainloop to gather and publish sensor values and wait for service calls
    r = rospy.Rate(15) # 10hz
    try:
        asyncio.ensure_future(rvr_robot())

        while not rospy.is_shutdown():
            loop.run_until_complete(asyncio.gather(handle_ros()))
            r.sleep()

    finally:
        loop.run_until_complete(
            asyncio.gather(
                rvr.sensor_control.clear(),
                rvr.close()
            )
        )
        if loop.is_running():
            loop.close()
