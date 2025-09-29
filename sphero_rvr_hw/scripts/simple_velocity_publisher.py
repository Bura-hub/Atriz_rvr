#!/usr/bin/env python3

# Ejemplo básico de nodo ROS para Sphero RVR
import rospy
from geometry_msgs.msg import Twist

def velocity_publisher():
    """Nodo simple que publica comandos de velocidad para el Sphero RVR"""
    
    # Inicializar el nodo
    rospy.init_node('simple_velocity_publisher', anonymous=True)
    
    # Crear publisher para el tópico cmd_vel
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    # Frecuencia de publicación (10 Hz)
    rate = rospy.Rate(10)
    
    rospy.loginfo("Nodo simple_velocity_publisher iniciado")
    rospy.loginfo("Publicando comandos de velocidad en /cmd_vel")
    
    while not rospy.is_shutdown():
        # Crear mensaje Twist
        twist_msg = Twist()
        
        # Configurar velocidad lineal (solo X funciona en Sphero RVR)
        twist_msg.linear.x = 0.2    # Velocidad hacia adelante (0.2 m/s)
        twist_msg.linear.y = 0.0    # Velocidad lateral (no funciona en RVR)
        twist_msg.linear.z = 0.0    # Velocidad vertical (no funciona en RVR)
        
        # Configurar velocidad angular (solo Z funciona en Sphero RVR)
        twist_msg.angular.x = 0.0   # Rotación X (no funciona en RVR)
        twist_msg.angular.y = 0.0   # Rotación Y (no funciona en RVR)
        twist_msg.angular.z = 0.1   # Giro lento (0.1 rad/s)
        
        # Publicar el mensaje
        pub.publish(twist_msg)
        
        # Log del comando enviado
        rospy.loginfo("Comando enviado - Linear: %.2f m/s, Angular: %.2f rad/s" % 
                     (twist_msg.linear.x, twist_msg.angular.z))
        
        # Esperar hasta la siguiente iteración
        rate.sleep()

if __name__ == '__main__':
    try:
        velocity_publisher()
    except rospy.ROSInterruptException:
        rospy.loginfo("Nodo interrumpido por el usuario")
