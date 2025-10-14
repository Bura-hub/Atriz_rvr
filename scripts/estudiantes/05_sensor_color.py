#!/usr/bin/env python3
"""Script ULTRA SIMPLE: Lee el sensor de color"""

import rospy
from atriz_rvr_msgs.msg import Color
from std_srvs.srv import SetBool
import signal, sys

# Variables globales
ultimo_color = None

def mostrar_color(msg):
    """Callback cuando llega un color"""
    global ultimo_color
    r = int(msg.rgb_color[0])
    g = int(msg.rgb_color[1])
    b = int(msg.rgb_color[2])
    ultimo_color = [r, g, b]
    print(f"Color: R={r:3d}, G={g:3d}, B={b:3d}")

def salir(signum=None, frame=None):
    """Ctrl+C"""
    print("\n👋 Saliendo...")
    sys.exit(0)

# Configurar Ctrl+C
signal.signal(signal.SIGINT, salir)

# Iniciar ROS
rospy.init_node('simple_color')
rospy.Subscriber('/color', Color, mostrar_color)

# Activar sensor
rospy.wait_for_service('/enable_color')
activar = rospy.ServiceProxy('/enable_color', SetBool)
activar(True)

print("🎨 Leyendo colores... (Ctrl+C para salir)")
print()

# Mantener activo
rospy.spin()

