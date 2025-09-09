#!/usr/bin/env python3

"""
Script de ejemplo para controlar el Sphero RVR usando grados directamente.
Este script demuestra cómo usar las nuevas funciones que trabajan con grados/segundo.
"""

import rospy
import asyncio
import time
from geometry_msgs.msg import Twist

def send_degree_command(linear_vel_m_s, angular_vel_deg_s, duration=0):
    """
    Envía un comando de velocidad usando grados/segundo.
    
    Args:
        linear_vel_m_s (float): Velocidad lineal en m/s
        angular_vel_deg_s (float): Velocidad angular en grados/s
        duration (float): Duración en segundos (0 = continuo)
    """
    rospy.init_node('degree_control_example', anonymous=True)
    
    # Publisher para comandos de velocidad
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    # Esperar a que el publisher se conecte
    rospy.sleep(0.5)
    
    # Crear mensaje de comando
    twist = Twist()
    twist.linear.x = linear_vel_m_s
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    # Convertir grados/s a rad/s para el mensaje ROS
    twist.angular.z = angular_vel_deg_s * (3.14159 / 180.0)
    
    if duration > 0:
        # Comando temporal
        rospy.loginfo(f"Ejecutando: {linear_vel_m_s} m/s, {angular_vel_deg_s}°/s por {duration}s")
        start_time = time.time()
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown() and (time.time() - start_time) < duration:
            cmd_vel_pub.publish(twist)
            rate.sleep()
        
        # Parar el robot
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        cmd_vel_pub.publish(twist)
        rospy.loginfo("Comando completado, robot detenido")
    else:
        # Comando continuo
        rospy.loginfo(f"Enviando comando continuo: {linear_vel_m_s} m/s, {angular_vel_deg_s}°/s")
        cmd_vel_pub.publish(twist)

def main():
    """Función principal con ejemplos de uso."""
    print("=== EJEMPLOS DE CONTROL CON GRADOS ===")
    print("1. Girar 90 grados por segundo por 4 segundos")
    print("2. Mover hacia adelante a 0.5 m/s por 3 segundos")
    print("3. Girar a la izquierda a 45 grados/s por 2 segundos")
    print("4. Movimiento combinado: adelante + giro por 5 segundos")
    print("5. Girar 360 grados en 10 segundos")
    print("6. Salir")
    
    while True:
        try:
            choice = input("\nSelecciona una opción (1-6): ")
            
            if choice == '1':
                # Girar 90 grados por segundo por 4 segundos
                send_degree_command(0.0, 90.0, 4.0)
                
            elif choice == '2':
                # Mover hacia adelante
                send_degree_command(0.5, 0.0, 3.0)
                
            elif choice == '3':
                # Girar a la izquierda
                send_degree_command(0.0, 45.0, 2.0)
                
            elif choice == '4':
                # Movimiento combinado
                send_degree_command(0.3, 30.0, 5.0)
                
            elif choice == '5':
                # Girar 360 grados en 10 segundos (36 grados/s)
                send_degree_command(0.0, 36.0, 10.0)
                
            elif choice == '6':
                print("Saliendo...")
                break
                
            else:
                print("Opción inválida. Selecciona 1-6.")
                
        except KeyboardInterrupt:
            print("\nInterrumpido por el usuario")
            break
        except Exception as e:
            print(f"Error: {e}")

if __name__ == '__main__':
    main()
