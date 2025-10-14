#!/usr/bin/env python3
"""
TEMPLATE SIMPLE para crear tus propios movimientos
Copia este archivo y modifica la sección "TU CÓDIGO AQUÍ"
"""

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

def mover(lineal=0, angular=0, tiempo=1):
    """
    Mueve el robot
    
    lineal: velocidad adelante/atrás (positivo=adelante, negativo=atrás)
    angular: velocidad de giro (positivo=izquierda, negativo=derecha)
    tiempo: duración en segundos
    """
    cmd = Twist()
    cmd.linear.x = lineal
    cmd.angular.z = angular
    
    for _ in range(int(tiempo * 10)):
        pub.publish(cmd)
        rospy.sleep(0.1)
    
    # Pequeña pausa después del movimiento
    rospy.sleep(0.3)

# Configurar Ctrl+C
signal.signal(signal.SIGINT, detener)

# Iniciar ROS
rospy.init_node('mi_movimiento')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
rospy.sleep(0.5)

# ============================================
# TU CÓDIGO AQUÍ - MODIFICA ESTA SECCIÓN
# ============================================

print("🤖 Iniciando mi secuencia personalizada...")

# Ejemplos de movimientos:

# Avanzar 2 segundos
mover(lineal=0.3, tiempo=2)

# Girar izquierda 1.5 segundos
mover(angular=0.5, tiempo=1.5)

# Avanzar y girar al mismo tiempo (curva)
mover(lineal=0.3, angular=0.3, tiempo=2)

# Retroceder
mover(lineal=-0.3, tiempo=2)

# Girar derecha
mover(angular=-0.5, tiempo=1.5)

# ============================================
# FIN DE TU CÓDIGO
# ============================================

detener()

