#!/usr/bin/env python3

# =======================================================
# Importaciones necesarias
# =======================================================
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

# =======================================================
# Importaciones de Sphero
# =======================================================
from sphero_sdk.asyncio.client.toys.sphero_rvr_async import SpheroRvrAsync
from sphero_sdk import SerialAsyncDal, RvrStreamingServices, InfraredControlAsync, RawMotorModesEnum, RvrLedGroups
from sphero_sdk.common.enums.power_enums import BatteryVoltageStatesEnum as VoltageStates
from sphero_sdk.common.enums.drive_enums import DriveFlagsBitmask
from sphero_sdk import Colors
from sphero_sdk.common.log_level import LogLevel

# =======================================================
# Importaciones de ROS
# =======================================================
import tf
import tf2_ros
import tf2_geometry_msgs
import rospy
import geometry_msgs
from geometry_msgs.msg import PoseWithCovariance, Pose, TwistWithCovariance, Twist, Point, Quaternion, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Header, String, Bool
from sphero_rvr_msgs.srv import MoveToPose, MoveToPoseResponse, MoveToPosAndYaw, MoveToPosAndYawResponse, BatteryState, BatteryStateResponse, TriggerLedEventRequest
from sphero_rvr_msgs.msg import Color
from sensor_msgs.msg import Illuminance
import std_srvs.srv
import rvr_tools

# =======================================================
# Variables globales
# =======================================================
loop = asyncio.get_event_loop()
rvr = SpheroRvrAsync(dal=SerialAsyncDal(loop))
rvr_color_picker = rvr_tools.RVRColorPicker(rvr)
tfBuffer = tf2_ros.Buffer()
covariance = [1e-6, .0, .0,
            .0, 1e-6, .0,
            .0, .0, 1e-6,]

# =======================================================
# Variables de movimiento
# =======================================================
received_components = set()
movement_complete = False
movement_success = False

# =======================================================
# Variables de odometría
# =======================================================
pub_tf = True
num_msgs_received = {}
robot_pose = Pose()
robot_twist = Twist()
odom = Odometry(header=Header(frame_id='odom'), child_frame_id='rvr_base_link')
imu = Imu(header=Header(frame_id='imu'))

# =======================================================
# Variables de sensores
# =======================================================
yaw_north = 0
calibration_completed = False
color_enabled = False
NO_SIGNAL: int = 00000
ir_signal = NO_SIGNAL

# =======================================================
# Variables de control
# =======================================================
is_driving_with_cmd_vel = False
last_cmd_vel_time = 0
cmd_vel_timeout = 0.3
is_in_emergency_stop = False

# =======================================================
# Publishers
# =======================================================
pub_odom = None
br = None
pub_light = None
pub_color = None
pub_imu = None
pub_ir_signal = None

# =======================================================
# Funciones de callback
# =======================================================
def emergency_stop_callback():
    """
    Activa el modo de parada de emergencia.

    En este callback, se llama a drive_stop() en el objeto rvr para detener el
    robot inmediatamente. Luego, se activa la bandera is_in_emergency_stop y se
    envía un comando a rvr_color_picker para que encienda los LEDs en rojo y
    destelle durante un tiempo breve, lo que indica que el robot ha entrado en
    parada de emergencia. Finalmente, se escribe un mensaje en el log para
    indicar que se ha activado la parada de emergencia.

    La parada de emergencia es un estado en el que el robot se detiene y no
    permite que se le envíen comandos de movimiento hasta que se llama a
    release_emergency_stop_callback().

    Args:
        None

    Returns:
        None
    """
    global is_in_emergency_stop, rvr_color_picker
    if not is_in_emergency_stop:
        # Detenemos el robot
        asyncio.run(rvr.drive_stop())
        # Activamos la bandera de parada de emergencia
        set_emergency_stop(True)
        # Encendemos los LEDs en rojo y destellamos durante un breve tiempo
        asyncio.run(
            rvr_color_picker.trigger_event(TriggerLedEventRequest.EMERGENCY_STOP)
        )
        # Escribimos un mensaje en el log para indicar que se ha activado la parada de emergencia
        rospy.loginfo('({}) ¡PARADA DE EMERGENCIA ACTIVADA!'.format(rospy.get_name()))

def release_emergency_stop_callback():
    """
    Libera el modo de parada de emergencia. El modo de parada de emergencia es un
    estado en el que el robot se detiene y no permite que se le envíen comandos de
    movimiento hasta que se llama a este callback. El callback simplemente
    desactiva la bandera que indica que el robot está en parada de emergencia y
    escribe un mensaje en el log.

    Returns:
        std_srvs.srv.EmptyResponse: Un mensaje vacío que indica que el servicio
            ha sido procesado correctamente.
    """
    global is_in_emergency_stop
    set_emergency_stop(False)
    rospy.loginfo('({}) Parada de emergencia liberada'.format(rospy.get_name()))
    return std_srvs.srv.EmptyResponse()

def reset_odom_callback():
    """Reinicia la odometría."""
    rospy.loginfo('({}) Reiniciando odometría'.format(rospy.get_name()))
    asyncio.run(reset_odom())
    return std_srvs.srv.EmptyResponse()

# TODO: Implementar estas funcionalidades correctamente: 
# move_to_pos_and_yaw_callback
# move_to_pose_callback
""" def move_to_pos_and_yaw_callback(req):
    """ """ Callback para mover el robot a una posición y orientación específicas.""" """
    orientation_q = tf.transformations.quaternion_from_euler(0., 0., req.yaw, axes='sxyz')
    pose = geometry_msgs.msg.Pose()
    pose.position = req.position
    pose.orientation.x = orientation_q[0]
    pose.orientation.y = orientation_q[1]
    pose.orientation.z = orientation_q[2]
    pose.orientation.w = orientation_q[3]
    success = move_to_pose(req.header.frame_id, pose.position, pose.orientation, req.speed, req.speed_in_si)
    return MoveToPosAndYawResponse(success)

def move_to_pose_callback(req):
    """ """Callback para mover el robot a una pose específica.""" """
    success = move_to_pose(req.header.frame_id, req.pose.position, req.pose.orientation, req.speed, req.speed_in_si)
    return MoveToPoseResponse(success) """

def cmd_vel_callback(cmdtwist):
    """
    Callback para manejar comandos de velocidad.

    Este callback se llama cada vez que se envía un mensaje de tipo
    geometry_msgs/Twist a este nodo. El mensaje de tipo Twist contiene
    dos componentes: linear y angular. El componente linear es la
    velocidad lineal del robot en metros por segundo, y el componente
    angular es la velocidad angular del robot en radianes por segundo.

    La velocidad lineal se traduce en velocidades para los motores
    izquierdo y derecho del robot. La velocidad angular se traduce en
    una diferencia de velocidad entre los motores izquierdo y derecho.
    """

    global is_in_emergency_stop, is_driving_with_cmd_vel, last_cmd_vel_time

    if is_in_emergency_stop:
        # Si el robot está en parada de emergencia, no se permite que se
        # le envíen comandos de movimiento.
        return
    v_x = cmdtwist.linear.x
    v_th = cmdtwist.angular.z
    radius = 0.1825 / 2
    vel_linear = v_th * radius
    # La velocidad lineal se traduce en velocidades para los motores
    # izquierdo y derecho del robot. La velocidad angular se traduce en
    # una diferencia de velocidad entre los motores izquierdo y derecho.
    l = v_x - vel_linear
    r = v_x + vel_linear
    # Se llama a la función write_motors_si para enviar las velocidades
    # a los motores del robot.
    asyncio.run(write_motors_si(l, r))
    is_driving_with_cmd_vel = True
    # Se almacena el momento en el que se envió el último comando de
    # velocidad.
    last_cmd_vel_time = time.time()

def battery_state_callback(res):
    """
    Callback para obtener el estado de la batería.

    Devuelve un mensaje de tipo BatteryStateResponse con dos campos:
    - battery_percentage: la carga actual de la batería en porcentaje.
    - voltage_state: el estado de la tensión de la batería en texto.

    Para obtener la carga actual de la batería, se llama a la función
    get_battery_percentage de la clase RVR. La función devuelve un
    diccionario con dos campos: 'percentage' y 'state'. El campo
    'percentage' contiene la carga actual de la batería en porcentaje,
    y el campo 'state' contiene el estado de la tensión de la batería.

    Para obtener el estado de la tensión de la batería en texto, se
    llama a la función get_battery_voltage_state de la clase RVR. La
    función devuelve un diccionario con un campo 'state' que contiene el
    estado de la tensión de la batería. Se utiliza la enumeración
    VoltageStates para convertir el estado numérico a texto.

    Finalmente, se devuelve el mensaje BatteryStateResponse con los
    campos battery_percentage y voltage_state.
    """
    res = BatteryStateResponse()
    battery_percentage = asyncio.run(rvr.get_battery_percentage(timeout=.1))
    asyncio.run(asyncio.sleep(.1))
    res.battery_percentage = battery_percentage['percentage']
    battery_voltage_state = asyncio.run(rvr.get_battery_voltage_state(timeout=.1))
    voltage_string = VoltageStates(battery_voltage_state['state']).name
    res.voltage_state = voltage_string
    rospy.loginfo('({}) Estado de batería: {}% - Estado de voltaje: {}'.format(rospy.get_name(), res.battery_percentage, voltage_string))
    return res

def enable_color_callback(req):
    """
    Callback para habilitar o deshabilitar el sensor de color.

    Recibe un mensaje de tipo Bool y habilita o deshabilita el sensor de
    color en función de su valor. Almacena el valor actual del sensor en
    la variable global color_enabled.
    """
    global color_enabled
    asyncio.run(rvr.enable_color_detection(is_enabled=req.data))
    asyncio.run(asyncio.sleep(.1))
    color_enabled = req.data
    rospy.loginfo("({}) Sensor de color habilitado: {}".format(rospy.get_name(), req.data))
    return True, ""
# =======================================================
# Funciones de manejo de sensores
# =======================================================
async def locator_handler(locator_data):
    """
    Maneja los datos del localizador.

    El localizador es un sensor que proporciona la posición del robot en el espacio
    en coordenadas cartesianas. La información del localizador se recibe en formato
    de diccionario con las claves "X" y "Y", que representan las coordenadas x e y
    de la posición del robot, respectivamente.

    Los datos se almacenan en la variable global `robot_pose.position` y se actualiza
    el mensaje de odometría con la nueva posición.
    """
    global robot_pose
    robot_pose.position.x = locator_data['Locator']['X']
    robot_pose.position.y = locator_data['Locator']['Y']
    robot_pose.position.z = 0
    odom.pose.pose.position = robot_pose.position
    check_if_need_to_send_msg('locator')

async def gyroscope_handler(gyroscope_data):
    """
    Maneja los datos del giroscopio. La información del giroscopio se recibe en
    formato de diccionario con las claves "X", "Y" y "Z", que representan las
    velocidades angulares en grados por segundo alrededor de los ejes x, y y z,
    respectivamente.

    Los datos se convierten a radianes por segundo y se almacenan en la variable
    global `robot_twist.angular`. Luego se actualiza el mensaje de odometría con
    la velocidad angular actualizada.

    El mensaje de odometría se utiliza para publicar la velocidad lineal y angular
    del robot en el tópico `/odom`.
    """
    global robot_twist
    robot_twist.angular.x = gyroscope_data['Gyroscope']['X'] # deg/s
    robot_twist.angular.y = gyroscope_data['Gyroscope']['Y'] # deg/s
    robot_twist.angular.z = gyroscope_data['Gyroscope']['Z'] # deg/s
    imu.angular_velocity = robot_twist.angular
    imu.angular_velocity_covariance = covariance
    check_if_need_to_send_msg('gyroscope')
    robot_twist.angular.x = gyroscope_data['Gyroscope']['X'] * 2 * pi / 360  # rad/s
    robot_twist.angular.y = gyroscope_data['Gyroscope']['Y'] * 2 * pi / 360  # rad/s
    robot_twist.angular.z = gyroscope_data['Gyroscope']['Z'] * 2 * pi / 360  # rad/s
    odom.twist.twist.angular = robot_twist.angular
    check_if_need_to_send_msg('gyroscope')

async def velocity_handler(velocity_data):
    """
    Maneja los datos de velocidad del robot y actualiza el mensaje de odometria
    con la velocidad lineal actual.

    El mensaje de odometria se utiliza para publicar la velocidad lineal y angular
    del robot en el tópico `/odom`.
    """
    global robot_twist
    # Actualiza la velocidad lineal del robot
    robot_twist.linear.x = velocity_data['Velocity']['X']
    robot_twist.linear.y = velocity_data['Velocity']['Y']
    # Actualiza el mensaje de odometria con la velocidad lineal actual
    odom.twist.twist.linear = robot_twist.linear
    # Verifica si es necesario enviar el mensaje de odometria
    check_if_need_to_send_msg('velocity')

async def acceleration_handler(acceleration_data):
    """
    Maneja los datos de aceleración.

    La aceleración se mide en metros por segundo cuadrado y se almacena en la
    variable global `robot_twist.linear`. Luego se actualiza el mensaje de IMU
    con la aceleración lineal actualizada.

    El mensaje de IMU se utiliza para publicar la aceleración lineal y angular del
    robot en el tópico `/imu`.
    """
    global robot_twist
    # Lee los datos de aceleración y los almacena en la variable global
    robot_twist.linear.x = acceleration_data['Accelerometer']['X']
    robot_twist.linear.y = acceleration_data['Accelerometer']['Y']
    robot_twist.linear.z = acceleration_data['Accelerometer']['Z']
    # Actualiza el mensaje de IMU con la aceleración lineal actual
    imu.linear_acceleration = robot_twist.linear
    # Establece la covarianza de la aceleración
    imu.linear_acceleration_covariance = covariance
    # Verifica si es necesario enviar el mensaje de IMU
    check_if_need_to_send_msg('accelerometer')


async def imu_handler(imu_data):
    """ Maneja los datos del IMU """
    # Lee los datos del IMU y los convierte a radianes
    roll = imu_data['IMU']['Roll'] * (pi / 180)
    pitch = imu_data['IMU']['Pitch'] * (pi / 180)
    yaw = imu_data['IMU']['Yaw'] * (pi / 180)
    # Convierte las rotaciones de Euler a un quaternion
    orientation_q = tf.transformations.quaternion_from_euler(roll, pitch, yaw, axes='sxyz')
    # Actualiza la orientación del robot en el mensaje de odometria
    robot_pose.orientation.x = orientation_q[0]
    robot_pose.orientation.y = orientation_q[1]
    robot_pose.orientation.z = orientation_q[2]
    robot_pose.orientation.w = orientation_q[3]
    # Actualiza el mensaje de odometria con la orientación actualizada
    odom.pose.pose.orientation = robot_pose.orientation
    # Actualiza el mensaje de IMU con la orientación actualizada
    imu.orientation = robot_pose.orientation
    # Establece la covarianza de la orientación en el mensaje de IMU
    imu.orientation_covariance = covariance
    # Verifica si es necesario enviar el mensaje de odometria
    check_if_need_to_send_msg('quaternion')

async def color_handler(color_data):
    """
    Maneja los datos del sensor de color.

    La función 'color_handler' se encarga de manejar los datos del sensor de color
    del Sphero RVR. Los datos se reciben en formato de diccionario con las claves
    "ColorDetection", que a su vez contiene los valores "R", "G" y "B", que
    representan los valores de los colores rojo, verde y azul detectados,
    respectivamente, y "Confidence", que representa la confianza en la
    detección del color.

    La función crea un mensaje de tipo 'Color' y lo llena con los datos
    recibidos. El mensaje se publica en el tópico '/color'.
    """
    global pub_color, color_enabled
    if not color_enabled:
        # Si el sensor de color no está habilitado, se sale de la función sin
        # hacer nada.
        return
    detected_color = [color_data['ColorDetection']['R'], color_data['ColorDetection']['G'], color_data['ColorDetection']['B']]
    # Crea un mensaje de tipo 'Color' y lo llena con los datos recibidos.
    msg = Color()
    msg.confidence = color_data['ColorDetection']['Confidence']
    msg.rgb_color = detected_color
    # Publica el mensaje en el tópico '/color'.
    pub_color.publish(msg)

async def light_handler(light_data):
    """
    Maneja los datos del sensor de luz ambiental.

    La función 'light_handler' se encarga de manejar los datos del sensor de luz
    ambiental del Sphero RVR. Los datos se reciben en formato de diccionario con
    la clave "AmbientLight", que a su vez contiene los valores "Light" y
    "Variance", que representan la intensidad de la luz y su varianza,
    respectivamente.

    La función crea un mensaje de tipo 'Illuminance' y lo llena con los datos
    recibidos. El mensaje se publica en el tópico '/illuminance'.
    """
    global pub_light
    # Crea un mensaje de tipo 'Illuminance' y lo llena con los datos recibidos.
    msg = Illuminance()
    # Establece el timestamp del mensaje en el momento actual.
    msg.header.stamp = rospy.Time()
    # Establece la intensidad de la luz en el mensaje.
    msg.illuminance = light_data['AmbientLight']['Light']
    # Establece la varianza de la intensidad de la luz en el mensaje en 0, ya que
    # el sensor no proporciona esta información.
    msg.variance = 0
    # Publica el mensaje en el tópico '/illuminance'.
    pub_light.publish(msg)

""""
async def ir_handler(ir_data):
    Maneja los datos del sensor de infrarrojos.
    global pub_ir_signal
    ir_readings = rvr.get_bot_to_bot_infrared_readings()
    
    msg.header.stamp = rospy.Time()
    msg.radiation_type = 0
    msg.field_of_view = 0
    msg.min_range = 0
    msg.max_range = 0
    msg.range = ir_data['Infrared']['Distance']
    pub_ir_signal.publish(msg)
"""

#TODO: Revisar este handler
""""
async def speed_handler(speed_data):
    """#Maneja los datos del sensor de luz ambiental.
"""
    global pub_light
    msg = Illuminance()
    msg.header.stamp = rospy.Time()
    msg.illuminance = light_data['AmbientLight']['Light']
    msg.variance = 0
    pub_light.publish(msg)
"""
async def on_motor_stall_handler(data):
    """Maneja los eventos de bloqueo del motor."""
    rospy.logwarn('({}) Bloqueo de motor detectado: {}'.format(rospy.get_name(), data))

async def on_motor_fault_handler(data):
    """Maneja los eventos de fallo del motor."""
    rospy.logwarn('({}) Fallo de motor detectado: {}'.format(rospy.get_name(), data))

async def handle_ros():
    """Maneja las tareas periódicas de ROS.

    En particular, esta función se encarga de manejar el watchdog de timeout
    para el comando de velocidad lineal y angular (cmd_vel). Si pasan más de
    'cmd_vel_timeout' segundos desde la última vez que se recibió un comando
    de velocidad, se asume que el robot debe detenerse.

    Si se cumple esta condición, se envía un comando de velocidad lineal y
    angular de 0 al robot, lo que lo detendrá.

    Luego, se duerme durante 0.1 segundos antes de volver a ejecutar esta
    función.
    """
    global last_cmd_vel_time, is_driving_with_cmd_vel, cmd_vel_timeout
    if is_driving_with_cmd_vel and time.time() - last_cmd_vel_time > cmd_vel_timeout:
        # Si ha pasado demasiado tiempo desde la última vez que se recibió
        # un comando de velocidad, detenemos el robot.
        await write_motors_si(0, 0)
    # Duerme durante 0.1 segundos antes de volver a ejecutar esta función.
    await asyncio.sleep(0.1)

async def will_sleep_handler():
    """Maneja el evento de que el RVR est a punto de entrar en modo de suspension.

    Este evento se dispara 10 segundos antes de que el RVR entre en modo de
    suspension. Para evitar que el RVR entre en este modo, se llama a la funcion
    wake() para mantener al RVR despierto.
    """
    rospy.loginfo('({}) Previniendo que el RVR entre en modo de suspension'.format(rospy.get_name()))
    await rvr.wake()
    await asyncio.sleep(.1)

def on_xy_handler(success):
    """Maneja la finalización del movimiento a una posición XY."""
    global movement_complete, movement_success
    movement_complete = True
    movement_success = success['success']
    rospy.loginfo('({}) En posición: {}'.format(rospy.get_name(), movement_success))

# =======================================================
# Funciones personalizadas
# =======================================================
def set_emergency_stop(is_enabled):
    """
    Establece el estado de parada de emergencia.

    La parada de emergencia es un estado en el que el robot se detiene y no
    permite que se le envíen comandos de movimiento hasta que se llama a
    release_emergency_stop_callback().

    Args:
        is_enabled (bool): True si se debe activar la parada de emergencia,
            False en caso contrario.
    """
    global is_in_emergency_stop
    is_in_emergency_stop = is_enabled
    rospy.set_param('emergency_stop', is_in_emergency_stop)

async def reset_odom():
    """Reinicia la odometría del RVR."""
    global is_in_emergency_stop, rvr_color_picker
    # Desactiva la parada de emergencia para permitir el reinicio de la odometría
    rospy.loginfo('({}) Reiniciando odometría'.format(rospy.get_name()))
    set_emergency_stop(False)
    # Reinicia la orientación del RVR
    await rvr.reset_yaw()
    await asyncio.sleep(.1)
    # Reinicia la posición del RVR
    await rvr.reset_locator_x_and_y()
    await asyncio.sleep(.1)
    # Inicia el patrón de color para el modo de conducción
    await rvr_color_picker.trigger_event(TriggerLedEventRequest.START_DRIVING)

async def rvr_robot():
    """Inicializa el robot RVR."""
    global pub_odom, br, rvr_color_picker
    # 1. Despierta al robot
    await rvr.wake()
    # 2. Espera 5 segundos para que el robot se inicialice
    await asyncio.sleep(5)
    # 3. Reinicia los sensores
    await reset_sensors()
    # 4. Reinicia la odometría
    await reset_odom()
    # 5. Muestra el patrón de color para el modo de arranque
    await rvr_color_picker.trigger_event(TriggerLedEventRequest.STARTUP)
    
    # Muestra informacion de estado
    # 1. Obtiene el porcentaje de batería
    battery_percentage = await rvr.get_battery_percentage()
    # 2. Obtiene el estado de voltaje de la batería
    battery_voltage_state = await rvr.get_battery_voltage_state()
    # 3. Muestra el estado de batería y el estado de voltaje
    rospy.loginfo('({}) Estado de batería: {}% - Estado de voltaje: {}'.format(rospy.get_name(), battery_percentage["percentage"], VoltageStates(battery_voltage_state["state"]).name))
    rospy.loginfo('({}) Nodo Python de Sphero RVR HW listo'.format(rospy.get_name()))

async def reset_sensors():
    """Reinicia los sensores del RVR."""
    # Se muestra un mensaje en la consola para indicar que se está reiniciando los sensores
    rospy.loginfo('({}) Despertando Sphero. Inicializando flujos de sensores...'.format(rospy.get_name()))

    # Se limpian los handlers de los sensores
    await rvr.sensor_control.clear()
    # Se detiene el flujo de los sensores
    await rvr.sensor_control.stop()

    # Se establecen los handlers para cada sensor
    # 1. Locator
    await rvr.sensor_control.add_sensor_data_handler(
        service=RvrStreamingServices.locator,
        handler=locator_handler,
    )
    # 2. Color detection
    await rvr.sensor_control.add_sensor_data_handler(
        service=RvrStreamingServices.color_detection,
        handler=color_handler
    )
    # 3. Gyroscope
    await rvr.sensor_control.add_sensor_data_handler(
        service=RvrStreamingServices.gyroscope,
        handler=gyroscope_handler,
    )
    # 4. Velocidad
    await rvr.sensor_control.add_sensor_data_handler(
        service=RvrStreamingServices.velocity,
        handler=velocity_handler,
    )
    # 5. Acelerómetro
    await rvr.sensor_control.add_sensor_data_handler(
        service=RvrStreamingServices.accelerometer,
        handler=acceleration_handler,
    )
    # 6. IMU
    await rvr.sensor_control.add_sensor_data_handler(
        service=RvrStreamingServices.imu,
        handler=imu_handler,
    )
    # 7. Luz ambiental
    await rvr.sensor_control.add_sensor_data_handler(
        service=RvrStreamingServices.ambient_light,
        handler=light_handler,
    )
    # 9. Sensor IR
    await rvr.sensor_control.add_sensor_data_handler(
        service=RvrStreamingServices.ambient_light,
        handler=light_handler,
    )
    # 9. Resultado de drive
    await rvr.on_xy_position_drive_result_notify(
        handler=on_xy_handler
    )
    # Se habilitan los handlers de motor stall y motor fault
    await rvr.enable_motor_stall_notify(is_enabled=True)
    await rvr.on_motor_stall_notify(
        handler=on_motor_stall_handler
    )
    await rvr.enable_motor_fault_notify(is_enabled=True)
    await rvr.on_motor_fault_notify(
        handler=on_motor_fault_handler
    )
    # Se habilita el handler de will sleep
    await rvr.on_will_sleep_notify(
        handler=will_sleep_handler
    )

    # Se inicia el flujo de los sensores con un intervalo de 250ms
    await rvr.sensor_control.start(interval=250)
    
def sig_handler(_signo, _stack_frame):
    """
    Manejador de señales para finalización limpia.

    Se encarga de limpiar los recursos utilizados por el RVR y
    cerrar la conexión con el robot.
    """
    rvr.sensor_control.clear()
    rvr.close()
    print("Programa del RVR terminado limpiamente.")
    sys.exit(0)

def check_if_need_to_send_msg(component):
    """
    Comprueba si es necesario enviar un mensaje de odometría.

    Se utiliza un conjunto para almacenar los componentes que se han recibido.
    Si se ha recibido al menos un mensaje de cada componente se envía el mensaje
    de odometría.

    Componentes que se esperan recibir:

    - Locator: Posición del robot en coordenadas cartesianas.
    - Quaternion: Orientación del robot en formato cuaternión.
    - Gyroscope: Velocidad angular del robot.
    - Velocity: Velocidad lineal del robot.
    - Accelerometer: Aceleración lineal del robot.

    Si se produce un error al enviar el mensaje de odometría, se imprime la
    excepción y se continua con la ejecución del programa.
    """
    global robot_pose, odom, imu, received_components

    received_components.add(component)
    if received_components >= {'locator', 'quaternion', 'gyroscope', 'velocity', 'accelerometer'}:
        received_components.clear()
        odom.header.stamp = rospy.Time.now()
        imu.header.stamp = rospy.Time.now()

        try:
            # Se envía el mensaje de odometría
            pub_odom.publish(odom)
            # Se envía el mensaje de imu
            pub_imu.publish(imu)

            # Si se ha especificado que se debe publicar el tf, se publica
            if pub_tf:
                br.sendTransform(
                    (robot_pose.position.x, robot_pose.position.y, robot_pose.position.z),
                    (robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w),
                    odom.header.stamp,
                    'rvr_base_link',
                    'odom'
                )
        except Exception:
            # Si se produce un error, se imprime la excepción y se continua
            traceback.print_exc()

async def write_motors_si(l_vel, r_vel):
    """
    Escribe las velocidades de los motores en unidades SI.

    Este método se encarga de escribir las velocidades de los motores en
    unidades del sistema internacional (SI, metros por segundo) en el
    objeto rvr que se ha creado en el módulo principal.

    El método utiliza las variables globales last_cmd_vel_time,
    is_driving_with_cmd_vel e is_in_emergency_stop para conocer el estado
    actual del robot y decidir si se debe escribir la velocidad o no.

    Si el robot está en un estado de emergencia, no se escribe nada y se
    sale del método.

    Si se ha especificado una velocidad de 0 para ambos motores, se
    considera que el robot debe detenerse y se pone la variable
    is_driving_with_cmd_vel a False.

    Si se ha especificado una velocidad diferente de 0 para alguno de los
    motores, se considera que el robot debe moverse y se pone la variable
    is_driving_with_cmd_vel a True.

    Después de escribir la velocidad, se espera 0.1 segundos antes de
    salir del método.
    """
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
    """
    Mueve el robot a una pose específica.

    Este método se encarga de mover el robot a una pose específica en el
    marco de referencia definido por frame_id. La velocidad de movimiento
    se especifica en unidades del sistema internacional (SI, metros por
    segundo) si speed_in_si es True, o en unidades normalizadas (0-128) si
    speed_in_si es False.

    El método utiliza las variables globales tfBuffer, is_in_emergency_stop
    e is_driving_with_cmd_vel para conocer el estado actual del robot y
    decidir si se debe escribir la velocidad o no.

    Si el robot está en un estado de emergencia, no se escribe nada y se
    sale del método.

    Si se ha especificado una velocidad de 0 para ambos motores, se
    considera que el robot debe detenerse y se pone la variable
    is_driving_with_cmd_vel a False.

    Si se ha especificado una velocidad diferente de 0 para alguno de los
    motores, se considera que el robot debe moverse y se pone la variable
    is_driving_with_cmd_vel a True.

    Después de escribir la velocidad, se espera 0.1 segundos antes de
    salir del método.
    """
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
        rospy.logerr('({}) ERROR al buscar la transformación de marcos tf2'.format(rospy.get_name()))
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
    (row, pitch, yaw) = tf.transformations.euler_from_quaternion(message_to_quaternion(ori_transformed.pose.orientation), axes='sxyz')
    yaw_d = yaw * (180 / pi)
    # Si se especifica una velocidad negativa, se considera que se debe
    # mover marcha atrás
    drive_flags = DriveFlagsBitmask.fast_turn
    if speed < 0:
        drive_flags |= DriveFlagsBitmask.drive_reverse
    success = True
    rospy.loginfo('({}) Moviendo a pose base_link: x:{:1.3f} y:{:1.3f} yaw:{:1.3f}'.format(rospy.get_name(), pos_transformed.pose.position.x, pos_transformed.pose.position.y, yaw_d))
    if speed_in_si:
        rospy.loginfo('({}) velocidad(si):{:1.3f} m/s'.format(rospy.get_name(), speed))
        asyncio.run(rvr.drive_to_position_si(yaw_d, pos_transformed.pose.position.x, pos_transformed.pose.position.y, speed, flags=drive_flags))
        asyncio.run(asyncio.sleep(.1))
        success = wait_until_motion_complete()
        if not success:
            asyncio.run(rvr.drive_to_position_si(yaw_d, 0.0, 0.0, speed, flags=DriveFlagsBitmask.fast_turn))
            asyncio.run(asyncio.sleep(.1))
            success = wait_until_motion_complete()
    else:
        if int(speed) < 0 or int(speed) > 128:
            rospy.logerr('({}) ERROR: La velocidad normalizada debe ser 0 >= velocidad >= 128'.format(rospy.get_name()))
            success = False
        else:
            rospy.loginfo('({}) velocidad(normalizada):{}'.format(rospy.get_name(), int(speed)))
            asyncio.run(rvr.drive_to_position_normalized(int(yaw_d), pos_transformed.pose.position.x, pos_transformed.pose.position.y, int(speed), flags=drive_flags))
            asyncio.run(asyncio.sleep(.1))
            success = wait_until_motion_complete()
            if not success:
                asyncio.run(rvr.drive_to_position_normalized(int(yaw_d), 0, 0, int(speed), flags=DriveFlagsBitmask.fast_turn))
                asyncio.run(asyncio.sleep(.1))
                success = wait_until_motion_complete()
    return success

def wait_until_motion_complete():
    """
    Espera hasta que el movimiento se complete.
    
    Es un bucle que se queda esperando hasta que el movimiento se complete.
    Mientras tanto, si se produce un stop de emergencia, sale del bucle.
    """
    global movement_complete, movement_success, is_in_emergency_stop
    # Bucle que se queda esperando hasta que el movimiento se complete
    rate = rospy.Rate(10)
    while not movement_complete and not is_in_emergency_stop:
        rate.sleep()
    # Reseteamos la variable de movimiento
    movement_complete = False
    # Devuelve el valor de exito del movimiento
    return movement_success

def message_to_quaternion(orientation):
    """
    Convierte un mensaje de orientación a un cuaternión.

    El mensaje de orientación se supone que es un objeto de tipo
    geometry_msgs.msg.Quaternion.

    Devuelve una lista de cuatro elementos que representan el cuaternión
    de la orientación del robot.
    """
    return [orientation.x, orientation.y, orientation.z, orientation.w]

if __name__ == '__main__':
    rospy.init_node('driver_rvr', disable_signals=True)
    rospy.loginfo('({}) Iniciando nodo Python de Sphero RVR HW...'.format(rospy.get_name()))
    # Carga el parámetro pub_tf. Si no se especifica, se asume True
    pub_tf = rospy.get_param('~pub_tf', True)
    rospy.loginfo('({}) Registrando interpreter de señales...'.format(rospy.get_name()))
    # Crea un signal handler para que cuando se envíe una señal (Ctrl+C, por ejemplo)
    # se llame a la función sig_handler
    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGHUP, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)
    # Crea un listener de transformaciones. Este listener se encarga de
    # guardar las transformaciones en el buffer tfBuffer
    listener = tf2_ros.TransformListener(tfBuffer)

    # Crea un broadcaster de transformaciones. Este broadcaster se encarga de
    # enviar las transformaciones guardadas en el buffer tfBuffer al tópico
    # /tf
    br = tf.TransformBroadcaster()

    # Crea los publishers para los tópicos /odom, /ambient_light, /color y /imu
    pub_odom = rospy.Publisher('odom', Odometry, queue_size=10)
    pub_light = rospy.Publisher('ambient_light', Illuminance, queue_size=10)
    pub_color = rospy.Publisher('color', Color, queue_size=10)
    pub_imu = rospy.Publisher('imu', Imu, queue_size=10)
    pub_ir_signal = rospy.Publisher('ir_signal', Bool, queue_size=20)

    # Crea los subscribers para los tópicos /cmd_vel y /is_emergency_stop
    rospy.Subscriber('cmd_vel', Twist, cmd_vel_callback, queue_size=1)
    rospy.Subscriber('is_emergency_stop', std_msgs.msg.Empty, emergency_stop_callback)

    # Crea los servicios para los tópicos
    #rospy.Service('move_to_pose', MoveToPose, move_to_pose_callback)
    #rospy.Service('move_to_pos_and_yaw', MoveToPosAndYaw, move_to_pos_and_yaw_callback)
    rospy.Service('enable_color', std_srvs.srv.SetBool, enable_color_callback)
    rospy.Service('battery_state', BatteryState, battery_state_callback)
    rospy.Service('reset_odom', std_srvs.srv.Empty, reset_odom_callback)
    rospy.Service('release_emergency_stop', std_srvs.srv.Empty, release_emergency_stop_callback)

    # Inicializa el Flag de stop de emergencia en False
    set_emergency_stop(False)

    # Crea un Rate para que el bucle principal se ejecute a 15 Hz
    r = rospy.Rate(15)

    try:
        # Inicializa el bucle principal
        asyncio.ensure_future(rvr_robot())

        while not rospy.is_shutdown():
            # Ejecuta el bucle principal
            loop.run_until_complete(asyncio.gather(handle_ros()))
            # Espera a que el bucle principal termine
            r.sleep()

    finally:
        # Cierra el bucle principal
        loop.run_until_complete(
            asyncio.gather(
                rvr.sensor_control.clear(),
                rvr.close()
            )
        )
        # Cierra el loop
        if loop.is_running():
            loop.close()