#!/usr/bin/env python3

import random
import time
import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
from sphero_rvr_msgs.msg import Color

# Variables globales
color_enabled = False
detected_color = None
pub_color = None

# Inicializamos el nodo de ROS
rospy.init_node('rvr_random_walk', anonymous=True)

# Publicador para enviar comandos de velocidad
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

# Servicio para habilitar el color
rospy.wait_for_service('/enable_color')
try:
    enable_color_service = rospy.ServiceProxy('/enable_color', SetBool)
    enable_color_service(True)
    rospy.loginfo("Color sensor enabled")
    color_enabled = True
except rospy.ServiceException as e:
    rospy.logerr("Service call failed: %s", e)

# Parámetros para ajustar el movimiento aleatorio
LINEAR_VEL_MIN = 0.1  # Velocidad lineal mínima
LINEAR_VEL_MAX = 0.5  # Velocidad lineal máxima
ANGULAR_VEL_MIN = 0.5  # Velocidad angular mínima
ANGULAR_VEL_MAX = 2.0  # Velocidad angular máxima
TURN_CHANCE = 0.2      # Probabilidad de realizar un giro en cada ciclo (20%)

# Función para leer el color del tópico y verificar condiciones
def check_color_and_respond():
    global detected_color, color_enabled
    if not color_enabled:
        # Si el sensor de color no está habilitado, se sale de la función
        return

    try:
        color_data = rospy.wait_for_message('/color', Color, timeout=1.0)
        detected_color = [color_data.rgb_color[0], color_data.rgb_color[1], color_data.rgb_color[2]]

        # Verificamos si los valores están por debajo del umbral
        if detected_color[0] < 100 and detected_color[1] < 100 and detected_color[2] < 100:
            rospy.loginfo("Color detected below threshold, executing avoidance maneuver")
            avoid_obstacle()
    except rospy.ROSException as e:
        rospy.logwarn("Failed to receive color data: %s", e)

# Función para retroceder y girar el robot
def avoid_obstacle():
    # Creamos un mensaje Twist para retroceder y luego girar 90 grados
    twist = Twist()

    # Retrocedemos por 1 segundo
    twist.linear.x = -0.3  # Velocidad de retroceso
    twist.angular.z = 0.0
    cmd_vel_pub.publish(twist)
    time.sleep(1)

    # Luego giramos 90 grados
    twist.linear.x = 0.0
    twist.angular.z = 2 * 3.14
    cmd_vel_pub.publish(twist)
    time.sleep(2)  # Giramos por 1 segundo

    # Detenemos el robot
    twist.angular.z = 0.0
    cmd_vel_pub.publish(twist)

# Función para el movimiento aleatorio del robot
def random_walk():
    rate = rospy.Rate(10)  # Frecuencia de 10 Hz para la ejecución del ciclo
    while not rospy.is_shutdown():
        # Verificamos el color y respondemos si es necesario
        check_color_and_respond()

        # Creamos un mensaje Twist para enviar velocidades lineales y angulares
        twist = Twist()
        
        # Determinamos aleatoriamente si el robot debe girar o moverse hacia adelante
        if random.random() < TURN_CHANCE:
            # Generamos un giro aleatorio
            twist.angular.z = random.uniform(ANGULAR_VEL_MIN, ANGULAR_VEL_MAX)
            twist.linear.x = 0.2  # Detenemos el movimiento lineal al girar
        else:
            # Movemos el robot hacia adelante con una velocidad aleatoria
            twist.linear.x = 0.3 #random.uniform(LINEAR_VEL_MIN, LINEAR_VEL_MAX)
            twist.angular.z = 0.0 #random.uniform(-ANGULAR_VEL_MIN, ANGULAR_VEL_MIN)
        
        # Publicamos el mensaje Twist
        cmd_vel_pub.publish(twist)
        
        # Pausa hasta el siguiente ciclo
        rate.sleep()

# Nodo principal
if __name__ == '__main__':
    try:
        # Inicializamos el publicador para el tópico /color
        pub_color = rospy.Publisher('/color', Color, queue_size=10)
        
        # Comenzamos el movimiento aleatorio
        random_walk()
    except rospy.ROSInterruptException:
        pass
