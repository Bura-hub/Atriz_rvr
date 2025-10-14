#!/usr/bin/env python3
"""Script ULTRA SIMPLE: Dibuja un cuadrado"""

import rospy
from geometry_msgs.msg import Twist
import signal, sys

# Variable global
pub = None

def detener(signum=None, frame=None):
    """Detiene el robot"""
    cmd = Twist()
    for _ in range(5):
        pub.publish(cmd)
        rospy.sleep(0.1)
    sys.exit(0)

def avanzar(segundos=2):
    """Avanza durante X segundos"""
    cmd = Twist()
    cmd.linear.x = 0.3
    for _ in range(int(segundos * 10)):
        pub.publish(cmd)
        rospy.sleep(0.1)

def girar(segundos=3.14):
    """Gira durante X segundos (~90 grados con vel=0.5 rad/s)"""
    cmd = Twist()
    cmd.angular.z = 0.5
    for _ in range(int(segundos * 10)):
        pub.publish(cmd)
        rospy.sleep(0.1)

# Configurar Ctrl+C
signal.signal(signal.SIGINT, detener)

# Iniciar ROS
rospy.init_node('simple_cuadrado')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
rospy.sleep(0.5)

print("📐 Dibujando cuadrado...")

# Dibujar 4 lados
for lado in range(4):
    print(f"  Lado {lado+1}/4")
    avanzar(2)      # Avanzar 2 segundos
    rospy.sleep(0.5)
    girar(3.14)     # Girar 90 grados (π/2 rad / 0.5 rad/s = 3.14s)
    rospy.sleep(0.5)

detener()

