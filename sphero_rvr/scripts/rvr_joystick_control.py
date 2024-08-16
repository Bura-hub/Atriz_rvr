#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from sphero_rvr_msgs.srv import SetLeds, SetLedsRequest

# Variables globales para almacenar el estado actual del joystick
current_twist = Twist()


def joystick_callback(data):
    global current_twist

    # Mapeo del joystick
    x_axis = data.axes[0]  # Eje izquierdo-derecha
    rt = data.axes[5]  # RT para acelerar
    lt = data.axes[2]  # LT para frenar
    y_button = data.buttons[3]  # Botón Y para alternar LEDs

    # Ajustar la velocidad de avance y reversa
    forward_speed = (1/4)*(1 - rt)
    reverse_speed = (1/8)*(lt - 1)
    
    # Ajustar la rotación con una escala más suave
    angular_scale = 2.0  # Factor de escala para suavizar el giro
    if rt < 1:
    	if lt < 1:
    		current_twist.linear.x = forward_speed+reverse_speed
    		current_twist.angular.z = -x_axis * angular_scale
    	else:
    		current_twist.linear.x = forward_speed
    		current_twist.angular.z = x_axis * angular_scale
    elif lt < 1:
    	current_twist.linear.x = reverse_speed
    	current_twist.angular.z = -x_axis * angular_scale
    elif rt == 1 and lt == 1:
    	current_twist.linear.x = 0
    	current_twist.angular.z = x_axis * angular_scale
    
    # Cambiar LEDs cuando se presiona el botón Y
    if y_button:
        led_color = [255, 0, 0]  # Cambiar a rojo como ejemplo
    else:
        led_color = [0, 0, 0]  # Apagar LEDs si el botón Y no está presionado

def timer_callback(event):
    global current_twist

    # Publicar comandos de movimiento
    cmd_vel_pub.publish(current_twist)

def set_leds(rgb_color):
    rospy.wait_for_service('/set_leds')
    try:
        set_leds_srv = rospy.ServiceProxy('/set_leds', SetLeds)
        request = SetLedsRequest()
        request.rgb_color = rgb_color
        response = set_leds_srv(request)
        return response
    except rospy.ServiceException as e:
        rospy.logerr("Fallo en la llamada al servicio: %s" % e)

def listener():
    global cmd_vel_pub

    rospy.init_node('rvr_joystick_control')
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("/joy", Joy, joystick_callback)

    # Configurar un temporizador para publicar los comandos periódicamente
    rospy.Timer(rospy.Duration(0.1), timer_callback)  # Publicar cada 0.1 segundos

    rospy.spin()

if __name__ == '__main__':
    listener()

