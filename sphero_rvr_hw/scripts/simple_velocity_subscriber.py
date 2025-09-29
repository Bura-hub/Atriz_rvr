#!/usr/bin/env python3

# Ejemplo básico de nodo ROS Subscriber para Sphero RVR
import rospy
from geometry_msgs.msg import Twist

def velocity_callback(msg):
    """Callback que se ejecuta cuando llega un mensaje de velocidad"""
    
    # Extraer valores del mensaje
    linear_x = msg.linear.x
    angular_z = msg.angular.z
    
    # Mostrar información del comando recibido
    rospy.loginfo("Comando recibido:")
    rospy.loginfo("  Velocidad lineal: %.2f m/s" % linear_x)
    rospy.loginfo("  Velocidad angular: %.2f rad/s" % angular_z)
    
    # Simular procesamiento del comando
    if linear_x > 0:
        rospy.loginfo("  → Robot se mueve hacia ADELANTE")
    elif linear_x < 0:
        rospy.loginfo("  → Robot se mueve hacia ATRÁS")
    else:
        rospy.loginfo("  → Robot NO se mueve linealmente")
    
    if angular_z > 0:
        rospy.loginfo("  → Robot gira hacia la IZQUIERDA")
    elif angular_z < 0:
        rospy.loginfo("  → Robot gira hacia la DERECHA")
    else:
        rospy.loginfo("  → Robot NO gira")

def velocity_subscriber():
    """Nodo simple que escucha comandos de velocidad"""
    
    # Inicializar el nodo
    rospy.init_node('simple_velocity_subscriber', anonymous=True)
    
    # Crear subscriber para el tópico cmd_vel
    rospy.Subscriber('/cmd_vel', Twist, velocity_callback)
    
    rospy.loginfo("Nodo simple_velocity_subscriber iniciado")
    rospy.loginfo("Escuchando comandos de velocidad en /cmd_vel")
    
    # Mantener el nodo activo
    rospy.spin()

if __name__ == '__main__':
    try:
        velocity_subscriber()
    except rospy.ROSInterruptException:
        rospy.loginfo("Nodo interrumpido por el usuario")
