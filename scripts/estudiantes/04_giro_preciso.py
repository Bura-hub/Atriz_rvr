#!/usr/bin/env python3
"""Script con función mejorada para giros precisos en grados"""

import rospy
from geometry_msgs.msg import Twist
import signal, sys
import math

# Variable global
pub = None

def detener(signum=None, frame=None):
    """Detiene el robot"""
    cmd = Twist()
    for _ in range(5):
        pub.publish(cmd)
        rospy.sleep(0.1)
    sys.exit(0)

def girar_grados(grados, velocidad_angular=0.5):
    """
    Gira el robot un número específico de grados.
    
    Args:
        grados: Ángulo a girar (positivo = izquierda, negativo = derecha)
        velocidad_angular: Velocidad de rotación en rad/s (default: 0.5)
    
    Ejemplo:
        girar_grados(90)    # Gira 90° a la izquierda
        girar_grados(-90)   # Gira 90° a la derecha
        girar_grados(120)   # Gira 120° (para triángulo)
    """
    # Convertir grados a radianes
    radianes = grados * (math.pi / 180)
    
    # Calcular tiempo necesario
    tiempo = abs(radianes) / velocidad_angular
    
    # Determinar dirección (positivo = izq, negativo = der)
    direccion = velocidad_angular if grados > 0 else -velocidad_angular
    
    print(f"⚙️  Girando {grados}° ({radianes:.3f} rad) en {tiempo:.2f}s")
    
    # Ejecutar giro
    cmd = Twist()
    cmd.angular.z = direccion
    
    for _ in range(int(tiempo * 10)):
        pub.publish(cmd)
        rospy.sleep(0.1)
    
    print(f"✅ Completado: {grados}°")

# Configurar Ctrl+C
signal.signal(signal.SIGINT, detener)

# Iniciar ROS
rospy.init_node('giro_preciso')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
rospy.sleep(0.5)

print("🎯 Demostrando giros precisos...")
print()

# Demostración de diferentes ángulos
print("1️⃣ Giro de 45° (izquierda)")
girar_grados(45)
rospy.sleep(1)

print("\n2️⃣ Giro de 90° (izquierda)")
girar_grados(90)
rospy.sleep(1)

print("\n3️⃣ Giro de -90° (derecha)")
girar_grados(-90)
rospy.sleep(1)

print("\n4️⃣ Giro de 180° (media vuelta)")
girar_grados(180)
rospy.sleep(1)

print("\n5️⃣ Vuelta completa (360°)")
girar_grados(360)

print("\n✅ Demo completada!")
detener()

