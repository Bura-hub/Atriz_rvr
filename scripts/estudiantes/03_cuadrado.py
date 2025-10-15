#!/usr/bin/env python3
"""
Script ULTRA SIMPLE: Dibuja un cuadrado

ESTE SCRIPT ENSEÑA PROGRAMACIÓN DE SECUENCIAS
- Combina movimientos lineales y rotacionales
- Introduce el concepto de funciones reutilizables
- Muestra cómo crear patrones geométricos
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
# FUNCIONES DE MOVIMIENTO
# ============================================
def avanzar(segundos=2):
    """
    Función para hacer avanzar el robot durante un tiempo específico
    
    Args:
        segundos: Tiempo de avance en segundos (por defecto 2 segundos)
    """
    print(f"   → Avanzando {segundos} segundos...")
    
    # Crear comando de movimiento hacia adelante
    cmd = Twist()
    cmd.linear.x = 0.3   # Velocidad hacia adelante (0.3 m/s)
    cmd.angular.z = 0.0  # Sin rotación
    
    # Enviar comando repetidamente durante el tiempo especificado
    for _ in range(int(segundos * 10)):  # segundos × 10 Hz = número de iteraciones
        pub.publish(cmd)      # Enviar comando al robot
        rospy.sleep(0.1)      # Esperar 0.1 segundos (mantener 10 Hz)

def girar(segundos=3.14):
    """
    Función para hacer girar el robot durante un tiempo específico
    
    Args:
        segundos: Tiempo de giro en segundos (por defecto 3.14s para ~90°)
    """
    print(f"   → Girando {segundos} segundos...")
    
    # Crear comando de rotación
    cmd = Twist()
    cmd.linear.x = 0.0   # Sin movimiento hacia adelante/atrás
    cmd.angular.z = 0.5  # Velocidad de giro (0.5 rad/s)
    
    # Enviar comando repetidamente durante el tiempo especificado
    for _ in range(int(segundos * 10)):  # segundos × 10 Hz = número de iteraciones
        pub.publish(cmd)      # Enviar comando al robot
        rospy.sleep(0.1)      # Esperar 0.1 segundos (mantener 10 Hz)

# ============================================
# CONFIGURACIÓN DE ROS
# ============================================
# Configurar Ctrl+C para que llame a nuestra función de emergencia
signal.signal(signal.SIGINT, detener)

# Inicializar el nodo ROS con un nombre único
rospy.init_node('simple_cuadrado')

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
print("📐 Dibujando cuadrado...")

# Dibujar 4 lados del cuadrado
for lado in range(4):  # Repetir 4 veces (4 lados del cuadrado)
    print(f"  Lado {lado+1}/4")  # Mostrar progreso (1/4, 2/4, 3/4, 4/4)
    
    # 1. Avanzar para dibujar un lado del cuadrado
    avanzar(2)      # Avanzar durante 2 segundos
    rospy.sleep(0.5)  # Pausa pequeña entre movimientos
    
    # 2. Girar 90 grados para preparar el siguiente lado
    # Cálculo: 90° = π/2 radianes
    # Tiempo = ángulo / velocidad_angular = (π/2) / 0.5 = π ≈ 3.14 segundos
    girar(3.14)     # Girar durante 3.14 segundos (≈90 grados)
    rospy.sleep(0.5)  # Pausa pequeña entre movimientos

# Llamar a la función de detención al finalizar
detener()

