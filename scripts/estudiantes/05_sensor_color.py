#!/usr/bin/env python3
"""
Script ULTRA SIMPLE: Lee el sensor de color

ESTE SCRIPT ENSEÑA SENSORES Y CALLBACKS
- Introduce el concepto de suscripción a tópicos
- Muestra cómo procesar datos de sensores en tiempo real
- Explica el patrón callback de ROS
- Enseña a activar/desactivar sensores
"""

# ============================================
# IMPORTACIONES NECESARIAS
# ============================================
import rospy                    # Biblioteca principal de ROS para Python
from atriz_rvr_msgs.msg import Color  # Mensaje personalizado para datos de color
from std_srvs.srv import SetBool      # Servicio para activar/desactivar sensores
import signal                   # Para manejar señales del sistema (Ctrl+C)
import sys                      # Para salir del programa

# ============================================
# VARIABLES GLOBALES
# ============================================
# Variable global para almacenar el último color detectado
# La usaremos para procesar los datos que llegan del sensor
ultimo_color = None

# ============================================
# FUNCIÓN CALLBACK DEL SENSOR
# ============================================
def mostrar_color(msg):
    """
    Función callback que se ejecuta automáticamente cada vez que 
    el sensor de color detecta un nuevo color.
    
    Esta función es MUY IMPORTANTE porque:
    - Se ejecuta automáticamente cuando llegan datos del sensor
    - Procesa los datos en tiempo real
    - Es el patrón estándar de ROS para manejar sensores
    
    Args:
        msg: Mensaje de tipo Color que contiene:
            - rgb_color: Lista [R, G, B] con valores 0-255
            - confidence: Confianza de la detección (0-100)
    """
    # Declarar que vamos a usar la variable global
    global ultimo_color
    
    # Extraer valores RGB del mensaje y convertir a enteros
    # Los valores vienen como float, los convertimos a int para mayor claridad
    r = int(msg.rgb_color[0])  # Componente Rojo (0-255)
    g = int(msg.rgb_color[1]) # Componente Verde (0-255)
    b = int(msg.rgb_color[2])  # Componente Azul (0-255)
    
    # Guardar el color en la variable global para uso posterior
    ultimo_color = [r, g, b]
    
    # Mostrar los valores RGB en la consola
    # :3d significa "formato entero con 3 dígitos, alineado a la derecha"
    print(f"Color: R={r:3d}, G={g:3d}, B={b:3d}")

# ============================================
# FUNCIÓN DE EMERGENCIA
# ============================================
def salir(signum=None, frame=None):
    """
    Función de emergencia que se ejecuta cuando presionas Ctrl+C
    
    Args:
        signum: Número de la señal recibida (no lo usamos)
        frame: Frame actual de ejecución (no lo usamos)
    """
    print("\n👋 Saliendo...")
    print("✅ Programa terminado correctamente")
    sys.exit(0)  # Salir del programa

# ============================================
# CONFIGURACIÓN DE ROS
# ============================================
# Configurar Ctrl+C para que llame a nuestra función de emergencia
signal.signal(signal.SIGINT, salir)

# Inicializar el nodo ROS con un nombre único
rospy.init_node('simple_color')

# Crear un suscriptor al tópico de color
# - '/color': nombre del tópico donde se publican los datos del sensor
# - Color: tipo de mensaje que esperamos recibir
# - mostrar_color: función que se ejecutará cada vez que llegue un mensaje
rospy.Subscriber('/color', Color, mostrar_color)

# ============================================
# ACTIVACIÓN DEL SENSOR
# ============================================
# Esperar a que el servicio esté disponible
# Los servicios en ROS permiten activar/desactivar funcionalidades
print("⏳ Esperando servicio /enable_color...")
rospy.wait_for_service('/enable_color')

# Crear un cliente del servicio para poder llamarlo
activar = rospy.ServiceProxy('/enable_color', SetBool)

# Activar el sensor de color
# SetBool es un servicio que recibe un booleano (True/False)
# True = activar sensor, False = desactivar sensor
print("🔧 Activando sensor de color...")
activar(True)

# ============================================
# PROGRAMA PRINCIPAL
# ============================================
print("🎨 Leyendo colores... (Ctrl+C para salir)")
print("💡 Mueve el robot sobre diferentes superficies para ver los colores")
print("📊 Los valores RGB van de 0 (sin color) a 255 (máximo color)")
print()

# Mantener el programa activo
# rospy.spin() hace que el programa se quede esperando mensajes
# Cada vez que llegue un mensaje del sensor, se ejecutará mostrar_color()
rospy.spin()

