#!/usr/bin/env python3

# Importaciones necesarias
import copy
from math import pi
import time
import os
import sys
import logging
import asyncio
import traceback
import signal

import std_msgs.msg

# Importaciones de Sphero
from sphero_sdk.asyncio.client.toys.sphero_rvr_async import SpheroRvrAsync
from sphero_sdk import SerialAsyncDal, RvrStreamingServices, RawMotorModesEnum, RvrLedGroups
from sphero_sdk.common.enums.power_enums import BatteryVoltageStatesEnum as VoltageStates
from sphero_sdk.common.enums.drive_enums import DriveFlagsBitmask
from sphero_sdk import Colors
from sphero_sdk.common.log_level import LogLevel

# Importaciones de ROS
import tf
import tf2_ros
import tf2_geometry_msgs
import rospy
import geometry_msgs
from geometry_msgs.msg import PoseWithCovariance, Pose, TwistWithCovariance, Twist, Point, Quaternion, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, String, Bool
from sphero_rvr_msgs.srv import MoveToPose, MoveToPoseResponse, MoveToPosAndYaw, MoveToPosAndYawResponse, BatteryState, BatteryStateResponse, TriggerLedEventRequest
from sphero_rvr_msgs.msg import Color
from sensor_msgs.msg import Illuminance
import std_srvs.srv
import rvr_tools

# Variables globales
loop = asyncio.get_event_loop()
rvr = SpheroRvrAsync(dal=SerialAsyncDal(loop))
rvr_color_picker = rvr_tools.RVRColorPicker(rvr)
tfBuffer = tf2_ros.Buffer()

# Variables de movimiento
received_components = set()
movement_complete = False
movement_success = False

# Variables de odometría
pub_tf = True
num_msgs_received = {}
robot_pose = Pose()
robot_twist = Twist()
odom = Odometry(header=Header(frame_id='odom'), child_frame_id='rvr_base_link')

# Variables de sensores
yaw_north = 0
calibration_completed = False
color_enabled = False

# Variables de control
is_driving_with_cmd_vel = False
last_cmd_vel_time = 0
cmd_vel_timeout = 0.3
is_in_emergency_stop = False

# Publishers
pub_odom = None
br = None
pub_light = None
pub_color = None
pub_magnet = None

# Funciones de callback

def release_emergency_stop_callback(req):
    """Libera el modo de parada de emergencia."""
    global is_in_emergency_stop
    set_emergency_stop(False)
    rospy.loginfo("({}) Parada de emergencia liberada".format(rospy.get_name()))
    return std_srvs.srv.EmptyResponse()

def emergency_stop_callback(req):
    """Activa el modo de parada de emergencia."""
    global is_in_emergency_stop, rvr_color_picker
    if not is_in_emergency_stop:
        asyncio.run(rvr.drive_stop())
        set_emergency_stop(True)
        asyncio.run(rvr_color_picker.trigger_event(TriggerLedEventRequest.EMERGENCY_STOP))
        rospy.loginfo("({}) ¡PARADA DE EMERGENCIA ACTIVADA!".format(rospy.get_name()))

def reset_odom_callback(req):
    """Reinicia la odometría."""
    rospy.loginfo("({}) Reiniciando odometría".format(rospy.get_name()))
    asyncio.run(reset_odom())
    return std_srvs.srv.EmptyResponse()

def move_to_pos_and_yaw_callback(req):
    """Callback para mover el robot a una posición y orientación específicas."""
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
    """Callback para mover el robot a una pose específica."""
    success = move_to_pose(req.header.frame_id, req.pose.position, req.pose.orientation, req.speed, req.speed_in_si)
    return MoveToPoseResponse(success)

def cmd_vel_callback(cmdtwist):
    """Callback para manejar comandos de velocidad."""
    global is_in_emergency_stop, is_driving_with_cmd_vel, last_cmd_vel_time

    if is_in_emergency_stop:
        return

    v_x = cmdtwist.linear.x
    v_th = cmdtwist.angular.z
    radius = 0.1825 / 2
    vel_linear = v_th * radius

    l = v_x - vel_linear
    r = v_x + vel_linear

    asyncio.run(write_motors_si(l, r))
    is_driving_with_cmd_vel = True
    last_cmd_vel_time = time.time()

def calibrate_magnetometer_callback(req):
    """Callback para calibrar el magnetómetro."""
    res = std_srvs.srv.TriggerResponse()
    res.success = asyncio.run(calibrate_magnetometer(timeout=10.0))
    return res

def battery_state_callback(req):
    """Callback para obtener el estado de la batería."""
    res = BatteryStateResponse()
    battery_percentage = asyncio.run(rvr.get_battery_percentage(timeout=.1))
    asyncio.run(asyncio.sleep(.1))
    res.battery_percentage = battery_percentage["percentage"]
    
    battery_voltage_state = asyncio.run(rvr.get_battery_voltage_state(timeout=.1))
    voltage_string = VoltageStates(battery_voltage_state["state"]).name
    res.voltage_state = voltage_string

    rospy.loginfo('({}) Estado de batería: {}% - Estado de voltaje: {}'.format(rospy.get_name(), res.battery_percentage, voltage_string))
    return res

def enable_color_callback(req):
    """Callback para habilitar o deshabilitar el sensor de color."""
    global color_enabled
    asyncio.run(rvr.enable_color_detection(is_enabled=req.data))
    
    asyncio.run(asyncio.sleep(.1))
    color_enabled = req.data
    rospy.loginfo("({}) Sensor de color habilitado: {}".format(rospy.get_name(), req.data))
    return True, ""
   
# Funciones de manejo de sensores

async def locator_handler(locator_data):
    """Maneja los datos del localizador."""
    global robot_pose
    robot_pose.position.x = locator_data['Locator']['X']
    robot_pose.position.y = locator_data['Locator']['Y']
    robot_pose.position.z = 0
    odom.pose.pose.position = robot_pose.position
    check_if_need_to_send_msg('locator')

async def gyroscope_handler(gyroscope_data):
    """Maneja los datos del giroscopio."""
    global robot_twist
    robot_twist.angular.x = gyroscope_data['Gyroscope']['X'] * 2 * pi / 360
    robot_twist.angular.y = gyroscope_data['Gyroscope']['Y'] * 2 * pi / 360
    robot_twist.angular.z = gyroscope_data['Gyroscope']['Z'] * 2 * pi / 360
    odom.twist.twist.angular = robot_twist.angular
    check_if_need_to_send_msg('gyroscope')

async def velocity_handler(velocity_data):
    """Maneja los datos de velocidad."""
    global robot_twist
    robot_twist.linear.x = velocity_data['Velocity']['X']
    robot_twist.linear.y = velocity_data['Velocity']['Y']
    odom.twist.twist.linear = robot_twist.linear
    check_if_need_to_send_msg('velocity')

async def imu_handler(imu_data):
    """Maneja los datos del IMU."""
    global robot_pose
    orientation_q = tf.transformations.quaternion_from_euler(0, 0, imu_data['IMU']['Yaw'] * (pi / 180), axes='sxyz')
    robot_pose.orientation.x = orientation_q[0]
    robot_pose.orientation.y = orientation_q[1]
    robot_pose.orientation.z = orientation_q[2]
    robot_pose.orientation.w = orientation_q[3]
    odom.pose.pose.orientation = robot_pose.orientation
    check_if_need_to_send_msg('quaternion')

async def color_handler(color_data):
    """Maneja los datos del sensor de color."""
    global pub_color, color_enabled
    if not color_enabled:
        return
    detected_color = [color_data["ColorDetection"]["R"], color_data["ColorDetection"]["G"], color_data["ColorDetection"]["B"]]
    msg = Color()
    msg.confidence = color_data["ColorDetection"]["Confidence"]
    msg.rgb_color = detected_color
    pub_color.publish(msg)

async def light_handler(light_data):
    """Maneja los datos del sensor de luz ambiental."""
    global pub_light
    msg = Illuminance()
    msg.header.stamp = rospy.Time()
    msg.illuminance = light_data["AmbientLight"]["Light"]
    msg.variance = 0
    pub_light.publish(msg)

async def on_motor_stall_handler(data):
    """Maneja los eventos de bloqueo del motor."""
    rospy.logwarn("({}) Bloqueo de motor detectado: {}".format(rospy.get_name(), data))

async def on_motor_fault_handler(data):
    """Maneja los eventos de fallo del motor."""
    rospy.logwarn("({}) Fallo de motor detectado: {}".format(rospy.get_name(), data))

async def handle_ros():
    """Maneja las tareas periódicas de ROS."""
    global last_cmd_vel_time, is_driving_with_cmd_vel, cmd_vel_timeout
    if is_driving_with_cmd_vel and time.time() - last_cmd_vel_time > cmd_vel_timeout:
        await write_motors_si(0, 0)
    await asyncio.sleep(0.1)

async def will_sleep_handler():
    """Maneja el evento de que el RVR esté a punto de entrar en modo de suspensión."""
    rospy.loginfo("({}) Previniendo que el RVR entre en modo de suspensión".format(rospy.get_name()))
    await rvr.wake()
    await asyncio.sleep(.1)

def on_xy_handler(success):
    """Maneja la finalización del movimiento a una posición XY."""
    global movement_complete, movement_success
    movement_complete = True
    movement_success = success["success"]
    rospy.loginfo("({}) En posición: {}".format(rospy.get_name(), movement_success))

# Funciones personalizadas

def set_emergency_stop(is_enabled):
    """Establece el estado de parada de emergencia."""
    global is_in_emergency_stop
    is_in_emergency_stop = is_enabled
    rospy.set_param("emergency_stop", is_in_emergency_stop)

async def calibrate_magnetometer(timeout=None):
    """Calibra el magnetómetro."""
    global calibration_completed, is_in_emergency_stop
    if is_in_emergency_stop:
        return
    calibration_completed = False
    await rvr.on_magnetometer_calibration_complete_notify(handler=on_calibration_complete_notify_handler)
    await rvr.magnetometer_calibrate_to_north(timeout=timeout)
    start = time.time()
    while not calibration_completed:
        await asyncio.sleep(.1)
        if time.time() - start >= timeout:
            rospy.logerr("({}) Tiempo de espera agotado en la calibración del magnetómetro".format(rospy.get_name()))
            break
    return calibration_completed

async def reset_odom():
    """Reinicia la odometría del RVR."""
    global is_in_emergency_stop, rvr_color_picker
    rospy.loginfo("({}) Reiniciando odometría".format(rospy.get_name()))
    set_emergency_stop(False)
    await rvr.reset_yaw()
    await asyncio.sleep(.1)
    await rvr.reset_locator_x_and_y()
    await asyncio.sleep(.1)
    await rvr_color_picker.trigger_event(TriggerLedEventRequest.START_DRIVING)

async def rvr_robot():
    """Inicializa el robot RVR."""
    global pub_odom, br, rvr_color_picker
    await rvr.wake()
    await asyncio.sleep(5)
    await reset_sensors()
    await reset_odom()
    await rvr_color_picker.trigger_event(TriggerLedEventRequest.STARTUP)
    rospy.loginfo("({}) Nodo Python de Sphero RVR HW listo".format(rospy.get_name()))

async def reset_sensors():
    """Reinicia los sensores del RVR."""
    rospy.loginfo("({}) Despertando Sphero. Inicializando flujos de sensores...".format(rospy.get_name()))
    await rvr.sensor_control.clear()
    await rvr.sensor_control.stop()
    await rvr.sensor_control.add_sensor_data_handler(
        service=RvrStreamingServices.locator,
        handler=locator_handler,
    )   
    await rvr.sensor_control.add_sensor_data_handler(
        service=RvrStreamingServices.color_detection,
        handler=color_handler
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
    """Manejador de señales para finalización limpia."""
    rvr.sensor_control.clear()
    rvr.close()
    print("Programa del RVR terminado limpiamente.")
    sys.exit(0)

def check_if_need_to_send_msg(component):
    """Comprueba si es necesario enviar un mensaje de odometría."""
    global robot_pose, odom, received_components

    received_components.add(component)
    if received_components >= {'locator', 'quaternion', 'gyroscope', 'velocity'}:
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

async def write_motors_si(l_vel, r_vel):
    """Escribe las velocidades de los motores en unidades SI."""
    global last_cmd_vel_time, is_driving_with_cmd_vel, is_in_emergency_stop

    if is_in_emergency_stop:
        return

    last_cmd_vel_time = time.time()

    if r_vel == 0 and l_vel == 0:
        is_driving_with_cmd_vel = False
    else:
        is_driving_with_cmd_vel = True

    await rvr.drive_tank_si_units(l_vel, r_vel)
    await asyncio.sleep(.1)

def move_to_pose(frame_id, pos, ori, speed, speed_in_si):
    """Mueve el robot a una pose específica."""
    global tfBuffer, is_in_emergency_stop, is_driving_with_cmd_vel

    if is_in_emergency_stop:
        return False

    if is_driving_with_cmd_vel:
        asyncio.run(write_motors_si(0, 0))

    try:
        trans_base_world = tfBuffer.lookup_transform('world', 'rvr_base_link', rospy.Time())
        trans_frame_id_base = tfBuffer.lookup_transform('rvr_base_link', frame_id, rospy.Time())
        trans_frame_id_world = tfBuffer.lookup_transform('world', frame_id, rospy.Time())
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logerr("({}) ERROR al buscar la transformación de marcos tf2".format(rospy.get_name()))
        return False

    pos_s = geometry_msgs.msg.PoseStamped()
    pos_s.pose.position = pos
    pos_s.pose.orientation.w = 1
    pos_transformed = tf2_geometry_msgs.do_transform_pose(pos_s, trans_frame_id_base)

    trans_base_world.transform.translation.x = 0.
    trans_base_world.transform.translation.y = 0.
    trans_base_world.transform.translation.z = 0.
    pos_transformed = tf2_geometry_msgs.do_transform_pose(pos_transformed, trans_base_world)

    ori_s = geometry_msgs.msg.PoseStamped()
    ori_s.pose.orientation = ori
    ori_transformed = tf2_geometry_msgs.do_transform_pose(ori_s, trans_frame_id_world)
    (row, pitch, yaw) = tf.transformations.euler_from_quaternion(message_to_quaternion(ori_transformed.pose.orientation), axes="sxyz")
    yaw_d = yaw * (180 / pi)

    drive_flags = DriveFlagsBitmask.fast_turn
    if speed < 0:
        drive_flags |= DriveFlagsBitmask.drive_reverse

    success = True
    rospy.loginfo("({}) Moviendo a pose base_link: x:{:1.3f} y:{:1.3f} yaw:{:1.3f}".format(rospy.get_name(), pos_transformed.pose.position.x, pos_transformed.pose.position.y, yaw_d))
    if speed_in_si:
        rospy.loginfo("({}) velocidad(si):{:1.3f} m/s".format(rospy.get_name(), speed))
        asyncio.run(rvr.drive_to_position_si(yaw_d, pos_transformed.pose.position.x, pos_transformed.pose.position.y, speed, flags=drive_flags))
        asyncio.run(asyncio.sleep(.1))
        success = wait_until_motion_complete()
        if not success:
            asyncio.run(rvr.drive_to_position_si(yaw_d, 0.0, 0.0, speed, flags=DriveFlagsBitmask.fast_turn))
            asyncio.run(asyncio.sleep(.1))
            success = wait_until_motion_complete()
    else:
        if int(speed) < 0 or int(speed) > 128:
            rospy.logerr("({}) ERROR: La velocidad normalizada debe ser 0 >= velocidad >= 128".format(rospy.get_name()))
            success = False
        else:
            rospy.loginfo("({}) velocidad(normalizada):{}".format(rospy.get_name(), int(speed)))
            asyncio.run(rvr.drive_to_position_normalized(int(yaw_d), pos_transformed.pose.position.x, pos_transformed.pose.position.y, int(speed), flags=drive_flags))
            asyncio.run(asyncio.sleep(.1))
            success = wait_until_motion_complete()
            if not success:
                asyncio.run(rvr.drive_to_position_normalized(int(yaw_d), 0, 0, int(speed), flags=DriveFlagsBitmask.fast_turn))
                asyncio.run(asyncio.sleep(.1))
                success = wait_until_motion_complete()

    return success

def wait_until_motion_complete():
    """Espera hasta que el movimiento se complete."""
    global movement_complete, movement_success, is_in_emergency_stop

    rate = rospy.Rate(10)
    while not movement_complete and not is_in_emergency_stop:
        rate.sleep()
    movement_complete = False

    return movement_success

def message_to_quaternion(orientation):
    """Convierte un mensaje de orientación a un cuaternión."""
    return [orientation.x, orientation.y, orientation.z, orientation.w]

if __name__ == '__main__':
    rospy.init_node('rvr_ros_interface', disable_signals=True)
    rospy.loginfo("({}) Iniciando nodo Python de Sphero RVR HW...".format(rospy.get_name()))

    pub_tf = rospy.get_param('~pub_tf', True)

    rospy.loginfo("({}) Registrando manejador de señales...".format(rospy.get_name()))
    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGHUP, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    listener = tf2_ros.TransformListener(tfBuffer)
    br = tf.TransformBroadcaster()

    pub_odom = rospy.Publisher('odom', Odometry, queue_size=10)
    pub_light = rospy.Publisher('ambient_light', Illuminance, queue_size=10)
    pub_color = rospy.Publisher('color', Color, queue_size=10)
    pub_magnet = rospy.Publisher('magnet', geometry_msgs.msg.PoseStamped, queue_size=10)

    rospy.Subscriber('cmd_vel', Twist, cmd_vel_callback, queue_size=1)
    rospy.Subscriber("is_emergency_stop", std_msgs.msg.Empty, emergency_stop_callback)

    rospy.Service('move_to_pose', MoveToPose, move_to_pose_callback)
    rospy.Service('move_to_pos_and_yaw', MoveToPosAndYaw, move_to_pos_and_yaw_callback)
    rospy.Service('enable_color', std_srvs.srv.SetBool, enable_color_callback)
    rospy.Service('calibrate_magnetometer', std_srvs.srv.Trigger, calibrate_magnetometer_callback)
    rospy.Service('battery_state', BatteryState, battery_state_callback)
    rospy.Service('reset_odom', std_srvs.srv.Empty, reset_odom_callback)
    rospy.Service('release_emergency_stop', std_srvs.srv.Empty, release_emergency_stop_callback)

    set_emergency_stop(False)

    r = rospy.Rate(15)  # 15hz
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
