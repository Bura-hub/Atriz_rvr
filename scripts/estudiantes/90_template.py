#!/usr/bin/env python3
"""
TEMPLATE SIMPLE para crear tus propios movimientos

ESTE ES UN ARCHIVO PLANTILLA PARA QUE EXPERIMENTES
- Copia este archivo y renómbralo (ej: mi_robot.py)
- Modifica la sección "TU CÓDIGO AQUÍ" con tus propios movimientos
- Usa la función mover() para crear secuencias personalizadas
- ¡Sé creativo y experimenta!
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
# FUNCIÓN DE MOVIMIENTO REUTILIZABLE
# ============================================
def mover(lineal=0, angular=0, tiempo=1):
    """
    Función REUTILIZABLE para mover el robot de forma sencilla.
    
    Esta función es MUY ÚTIL porque:
    - Simplifica el código de movimiento
    - Permite combinar movimiento lineal y angular
    - Es fácil de usar y entender
    - Puedes reutilizarla muchas veces
    
    Args:
        lineal: Velocidad de movimiento hacia adelante/atrás
                - Positivo = adelante (ej: 0.3)
                - Negativo = atrás (ej: -0.3)
                - 0 = sin movimiento lineal
        angular: Velocidad de giro
                 - Positivo = gira a la izquierda (ej: 0.5)
                 - Negativo = gira a la derecha (ej: -0.5)
                 - 0 = sin rotación
        tiempo: Duración del movimiento en segundos
                - 1.0 = 1 segundo
                - 2.5 = 2.5 segundos
    """
    print(f"   → Movimiento: lineal={lineal}, angular={angular}, tiempo={tiempo}s")
    
    # Crear comando de movimiento
    cmd = Twist()
    cmd.linear.x = lineal    # Velocidad hacia adelante/atrás
    cmd.angular.z = angular  # Velocidad de giro
    
    # Enviar comando repetidamente durante el tiempo especificado
    for _ in range(int(tiempo * 10)):  # tiempo × 10 Hz = número de iteraciones
        pub.publish(cmd)      # Enviar comando al robot
        rospy.sleep(0.1)      # Esperar 0.1 segundos (mantener 10 Hz)
    
    # Pequeña pausa después del movimiento para estabilizar
    rospy.sleep(0.3)

# ============================================
# CONFIGURACIÓN DE ROS
# ============================================
# Configurar Ctrl+C para que llame a nuestra función de emergencia
signal.signal(signal.SIGINT, detener)

# Inicializar el nodo ROS con un nombre único
rospy.init_node('mi_movimiento')

# Crear un publicador para enviar comandos al robot
# - '/cmd_vel': nombre del tópico (canal de comunicación)
# - Twist: tipo de mensaje (velocidad lineal + angular)
# - queue_size=10: tamaño de la cola de mensajes
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

# Esperar un momento para que ROS se configure correctamente
rospy.sleep(0.5)

# ============================================
# TU CÓDIGO AQUÍ - MODIFICA ESTA SECCIÓN
# ============================================
# ¡ESTA ES LA SECCIÓN QUE DEBES MODIFICAR!
# Copia este archivo, renómbralo y crea tus propios movimientos

print("🤖 Iniciando mi secuencia personalizada...")

# ============================================
# EJEMPLOS DE MOVIMIENTOS BÁSICOS
# ============================================

# Ejemplo 1: Avanzar 2 segundos
# mover(lineal=0.3, tiempo=2)

# Ejemplo 2: Girar izquierda 1.5 segundos
# mover(angular=0.5, tiempo=1.5)

# Ejemplo 3: Avanzar y girar al mismo tiempo (curva)
# mover(lineal=0.3, angular=0.3, tiempo=2)

# Ejemplo 4: Retroceder
# mover(lineal=-0.3, tiempo=2)

# Ejemplo 5: Girar derecha
# mover(angular=-0.5, tiempo=1.5)

# ============================================
# DESAFÍOS PARA EXPERIMENTAR
# ============================================
# Descomenta y modifica estos ejemplos:

# Desafío 1: Dibujar un triángulo
# for i in range(3):
#     mover(lineal=0.3, tiempo=2)      # Avanzar
#     mover(angular=0.5, tiempo=4.19)  # Girar 120° (2π/3 rad)

# Desafío 2: Dibujar un círculo
# mover(lineal=0.2, angular=0.3, tiempo=10)

# Desafío 3: Patrón en zigzag
# for i in range(5):
#     mover(lineal=0.3, tiempo=1)       # Avanzar
#     mover(angular=0.5, tiempo=1.57)   # Girar 90°
#     mover(lineal=0.3, tiempo=1)       # Avanzar
#     mover(angular=-0.5, tiempo=1.57)  # Girar -90°

# ============================================
# TU CÓDIGO PERSONALIZADO AQUÍ
# ============================================
# ¡Escribe tu propia secuencia de movimientos!

# Ejemplo básico - descomenta y modifica:
mover(lineal=0.3, tiempo=2)    # Avanzar 2 segundos
mover(angular=0.5, tiempo=1.5) # Girar izquierda 1.5 segundos
mover(lineal=0.3, tiempo=2)    # Avanzar 2 segundos más

# ============================================
# FIN DE TU CÓDIGO
# ============================================

print("✅ Secuencia completada!")
detener()

