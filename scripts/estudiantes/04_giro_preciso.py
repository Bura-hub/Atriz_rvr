#!/usr/bin/env python3
"""
Script con función mejorada para giros precisos en grados

ESTE SCRIPT ENSEÑA CÁLCULOS PRECISOS DE GIROS
- Convierte grados a radianes automáticamente
- Calcula el tiempo necesario para cada giro
- Permite giros positivos (izquierda) y negativos (derecha)
- Introduce conceptos matemáticos básicos
"""

# ============================================
# IMPORTACIONES NECESARIAS
# ============================================
import rospy                    # Biblioteca principal de ROS para Python
from geometry_msgs.msg import Twist  # Tipo de mensaje para comandos de velocidad
import signal                   # Para manejar señales del sistema (Ctrl+C)
import sys                      # Para salir del programa
import math                     # Para operaciones matemáticas (π, conversiones)

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
# FUNCIÓN DE GIRO PRECISO
# ============================================
def girar_grados(grados, velocidad_angular=0.5):
    """
    Gira el robot un número específico de grados con precisión matemática.
    
    Esta función es MUY ÚTIL porque:
    - Convierte automáticamente grados a radianes
    - Calcula el tiempo exacto necesario
    - Permite giros en ambas direcciones
    - Es reutilizable para cualquier ángulo
    
    Args:
        grados: Ángulo a girar en grados
                - Positivo = gira a la izquierda
                - Negativo = gira a la derecha
        velocidad_angular: Velocidad de rotación en rad/s (por defecto 0.5)
    
    Ejemplos de uso:
        girar_grados(90)     # Gira 90° a la izquierda
        girar_grados(-90)    # Gira 90° a la derecha
        girar_grados(120)    # Gira 120° (útil para triángulos)
        girar_grados(45)     # Gira 45° (útil para octágonos)
    """
    # PASO 1: Convertir grados a radianes
    # Fórmula: radianes = grados × (π / 180)
    # Ejemplo: 90° × (π/180) = π/2 radianes
    radianes = grados * (math.pi / 180)
    
    # PASO 2: Calcular tiempo necesario para el giro
    # Fórmula: tiempo = ángulo / velocidad
    # Ejemplo: (π/2) / 0.5 = π segundos ≈ 3.14 segundos
    tiempo = abs(radianes) / velocidad_angular
    
    # PASO 3: Determinar dirección de giro
    # Si grados > 0: girar a la izquierda (velocidad positiva)
    # Si grados < 0: girar a la derecha (velocidad negativa)
    direccion = velocidad_angular if grados > 0 else -velocidad_angular
    
    # Mostrar información del giro
    print(f"⚙️  Girando {grados}° ({radianes:.3f} rad) en {tiempo:.2f}s")
    
    # PASO 4: Ejecutar el giro
    cmd = Twist()
    cmd.linear.x = 0.0        # Sin movimiento hacia adelante/atrás
    cmd.angular.z = direccion # Velocidad angular calculada
    
    # Enviar comando repetidamente durante el tiempo calculado
    for _ in range(int(tiempo * 10)):  # tiempo × 10 Hz = número de iteraciones
        pub.publish(cmd)      # Enviar comando al robot
        rospy.sleep(0.1)      # Esperar 0.1 segundos (mantener 10 Hz)
    
    # Confirmar que el giro se completó
    print(f"✅ Completado: {grados}°")

# ============================================
# CONFIGURACIÓN DE ROS
# ============================================
# Configurar Ctrl+C para que llame a nuestra función de emergencia
signal.signal(signal.SIGINT, detener)

# Inicializar el nodo ROS con un nombre único
rospy.init_node('giro_preciso')

# Crear un publicador para enviar comandos al robot
# - '/cmd_vel': nombre del tópico (canal de comunicación)
# - Twist: tipo de mensaje (velocidad lineal + angular)
# - queue_size=10: tamaño de la cola de mensajes
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

# Esperar un momento para que ROS se configure correctamente
rospy.sleep(0.5)

# ============================================
# PROGRAMA PRINCIPAL - DEMOSTRACIÓN
# ============================================
print("🎯 Demostrando giros precisos...")
print()

# Demostración de diferentes ángulos para mostrar la versatilidad
print("1️⃣ Giro de 45° (izquierda)")
girar_grados(45)  # Gira 45° a la izquierda
rospy.sleep(1)    # Pausa de 1 segundo entre giros

print("\n2️⃣ Giro de 90° (izquierda)")
girar_grados(90)  # Gira 90° a la izquierda (cuadrado)
rospy.sleep(1)    # Pausa de 1 segundo entre giros

print("\n3️⃣ Giro de -90° (derecha)")
girar_grados(-90) # Gira 90° a la derecha (negativo)
rospy.sleep(1)   # Pausa de 1 segundo entre giros

print("\n4️⃣ Giro de 180° (media vuelta)")
girar_grados(180) # Gira 180° (media vuelta completa)
rospy.sleep(1)    # Pausa de 1 segundo entre giros

print("\n5️⃣ Vuelta completa (360°)")
girar_grados(360) # Gira 360° (vuelta completa)

print("\n✅ Demo completada!")
print("\n💡 CONCEPTOS APRENDIDOS:")
print("   - Conversión de grados a radianes")
print("   - Cálculo de tiempo basado en velocidad angular")
print("   - Giros positivos (izquierda) y negativos (derecha)")
print("   - Uso de funciones reutilizables")

# Llamar a la función de detención al finalizar
detener()

