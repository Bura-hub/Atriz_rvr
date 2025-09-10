#!/usr/bin/env python3

"""
Script de ejemplo para controlar el Sphero RVR usando grados/segundo directamente.
Este script usa el nuevo tópico /cmd_degrees que recibe comandos en grados/segundo.
"""

import rospy
import time
from atriz_rvr_msgs.msg import DegreesTwist

def send_degrees_command(linear_x, angular_z_deg_s, duration=0):
    """
    Envía un comando de velocidad usando grados/segundo directamente.
    
    Args:
        linear_x (float): Velocidad lineal en m/s
        angular_z_deg_s (float): Velocidad angular en grados/s
        duration (float): Duración en segundos (0 = continuo)
    """
    rospy.init_node('degrees_control_example', anonymous=True)
    
    # Publisher para comandos de velocidad en grados
    cmd_degrees_pub = rospy.Publisher('/cmd_degrees', DegreesTwist, queue_size=1)
    
    # Esperar a que el publisher se conecte
    rospy.sleep(0.5)
    
    # Crear mensaje de comando
    degrees_twist = DegreesTwist()
    degrees_twist.linear_x = linear_x
    degrees_twist.linear_y = 0.0
    degrees_twist.linear_z = 0.0
    degrees_twist.angular_x = 0.0
    degrees_twist.angular_y = 0.0
    degrees_twist.angular_z = angular_z_deg_s  # Directamente en grados/s
    
    if duration > 0:
        # Comando temporal
        rospy.loginfo(f"Ejecutando: {linear_x} m/s, {angular_z_deg_s}°/s por {duration}s")
        start_time = time.time()
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown() and (time.time() - start_time) < duration:
            cmd_degrees_pub.publish(degrees_twist)
            rate.sleep()
        
        # Parar el robot
        degrees_twist.linear_x = 0.0
        degrees_twist.angular_z = 0.0
        cmd_degrees_pub.publish(degrees_twist)
        rospy.loginfo("Comando completado, robot detenido")
    else:
        # Comando continuo
        rospy.loginfo(f"Enviando comando continuo: {linear_x} m/s, {angular_z_deg_s}°/s")
        cmd_degrees_pub.publish(degrees_twist)

def main():
    """Función principal con ejemplos de uso."""
    print("=== CONTROL DIRECTO CON GRADOS/SEGUNDO ===")
    print("Usando el tópico /cmd_degrees (más intuitivo)")
    print()
    print("1. Girar 90 grados por segundo por 4 segundos")
    print("2. Mover hacia adelante a 0.5 m/s por 3 segundos")
    print("3. Girar a la izquierda a 45 grados/s por 2 segundos")
    print("4. Movimiento combinado: adelante + giro por 5 segundos")
    print("5. Girar 360 grados en 10 segundos (36°/s)")
    print("6. Girar 1 grado por segundo por 10 segundos")
    print("7. Salir")
    
    while True:
        try:
            choice = input("\nSelecciona una opción (1-7): ")
            
            if choice == '1':
                # Girar 90 grados por segundo por 4 segundos
                send_degrees_command(0.0, 90.0, 4.0)
                
            elif choice == '2':
                # Mover hacia adelante
                send_degrees_command(0.5, 0.0, 3.0)
                
            elif choice == '3':
                # Girar a la izquierda
                send_degrees_command(0.0, 45.0, 2.0)
                
            elif choice == '4':
                # Movimiento combinado
                send_degrees_command(0.3, 30.0, 5.0)
                
            elif choice == '5':
                # Girar 360 grados en 10 segundos (36 grados/s)
                send_degrees_command(0.0, 36.0, 10.0)
                
            elif choice == '6':
                # Girar 1 grado por segundo por 10 segundos
                send_degrees_command(0.0, 1.0, 10.0)
                
            elif choice == '7':
                print("Saliendo...")
                break
                
            else:
                print("Opción inválida. Selecciona 1-7.")
                
        except KeyboardInterrupt:
            print("\nInterrumpido por el usuario")
            break
        except Exception as e:
            print(f"Error: {e}")

if __name__ == '__main__':
    main()
