#!/usr/bin/env python3
"""
Script ULTRA SIMPLE: Solo avanza el robot

ESTE ES EL PRIMER SCRIPT QUE DEBES EJECUTAR
- Enseña los conceptos básicos de ROS
- Muestra cómo enviar comandos de movimiento
- Incluye manejo seguro de Ctrl+C
"""

# ============================================
# IMPORTACIONES NECESARIAS
# ============================================
import rospy                    # Biblioteca principal de ROS para Python
from geometry_msgs.msg import Twist  # Tipo de mensaje para comandos de velocidad
import signal                   # Para manejar señales del sistema (Ctrl+C)
import sys                      # Para salir del programa

# ============================================
# VARIABLES GLOBALES
# ============================================
# Variable global para el publicador - la usaremos en toda la función
pub = None

# ============================================
# FUNCIÓN DE EMERGENCIA
# ============================================
def detener(signum=None, frame=None):
    """
    Función de emergencia que se ejecuta cuando presionas Ctrl+C
    
    Args:
        signum: Número de la señal recibida (no lo usamos)
        frame: Frame actual de ejecución (no lo usamos)
    """
    print("\n🛑 DETENIENDO ROBOT...")
    
    # Crear comando de parada (velocidades en cero)
    cmd = Twist()
    cmd.linear.x = 0.0   # Sin movimiento hacia adelante/atrás
    cmd.angular.z = 0.0  # Sin rotación
    
    # Enviar comando de parada varias veces para asegurar que se detenga
    for _ in range(5):  # Repetir 5 veces
        pub.publish(cmd)      # Enviar comando al robot
        rospy.sleep(0.1)      # Esperar 0.1 segundos entre envíos
    
    print("✅ Robot detenido correctamente")
    sys.exit(0)  # Salir del programa

# ============================================
# CONFIGURACIÓN DE ROS
# ============================================
# Configurar Ctrl+C para que llame a nuestra función de emergencia
signal.signal(signal.SIGINT, detener)

# Inicializar el nodo ROS con un nombre único
rospy.init_node('simple_avanzar')

# Crear un publicador para enviar comandos al robot
# - '/cmd_vel': nombre del tópico (canal de comunicación)
# - Twist: tipo de mensaje (velocidad lineal + angular)
# - queue_size=10: tamaño de la cola de mensajes
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

# Esperar un momento para que ROS se configure correctamente
rospy.sleep(0.5)

# ============================================
# PROGRAMA PRINCIPAL
# ============================================
print("🚀 Robot avanzando...")

# Crear comando de movimiento
cmd = Twist()
cmd.linear.x = 0.3  # Velocidad hacia adelante (0.3 m/s)
# Nota: cmd.linear.y y cmd.linear.z no se usan en robots terrestres
# cmd.angular.z = 0.0  # Sin rotación (ya está en cero por defecto)

# Publicar comando durante 3 segundos
rate = rospy.Rate(10)  # 10 Hz = 10 mensajes por segundo
for _ in range(30):    # 30 iteraciones × 0.1s = 3 segundos
    pub.publish(cmd)   # Enviar comando al robot
    rate.sleep()       # Esperar 0.1 segundos (mantener 10 Hz)

# Llamar a la función de detención al finalizar
detener()

