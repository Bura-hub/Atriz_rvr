#!/usr/bin/env python3
"""Script ULTRA SIMPLE: Solo avanza el robot"""

import rospy
from geometry_msgs.msg import Twist
import signal, sys

# Variable global para el publicador
pub = None

def detener(signum=None, frame=None):
    """Detiene el robot"""
    cmd = Twist()
    for _ in range(5):
        pub.publish(cmd)
        rospy.sleep(0.1)
    sys.exit(0)

# Configurar Ctrl+C
signal.signal(signal.SIGINT, detener)

# Iniciar ROS
rospy.init_node('simple_avanzar')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
rospy.sleep(0.5)

print("🚀 Robot avanzando...")

# Crear comando de movimiento
cmd = Twist()
cmd.linear.x = 0.3  # Velocidad hacia adelante

# Publicar durante 3 segundos
rate = rospy.Rate(10)
for _ in range(30):  # 30 veces a 10 Hz = 3 segundos
    pub.publish(cmd)
    rate.sleep()

detener()

