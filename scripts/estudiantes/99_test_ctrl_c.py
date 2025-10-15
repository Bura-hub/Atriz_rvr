#!/usr/bin/env python3
"""
Script de prueba para verificar el manejo de Ctrl+C

ESTE SCRIPT ENSEÑA MANEJO DE SEÑALES Y SEGURIDAD
- Demuestra cómo manejar Ctrl+C correctamente
- Muestra el patrón de clase para organizar código
- Enseña buenas prácticas de seguridad
- Es útil para probar que el robot se detiene correctamente
"""

# ============================================
# IMPORTACIONES NECESARIAS
# ============================================
import rospy                    # Biblioteca principal de ROS para Python
from geometry_msgs.msg import Twist  # Tipo de mensaje para comandos de velocidad
import signal                   # Para manejar señales del sistema (Ctrl+C)
import sys                      # Para salir del programa
import time                     # Para medir tiempo transcurrido

# ============================================
# CLASE PRINCIPAL
# ============================================
class TestCtrlC:
    """
    Clase para probar el manejo correcto de Ctrl+C.
    
    Esta clase demuestra:
    - Cómo organizar código usando clases
    - Manejo seguro de señales del sistema
    - Patrones de programación orientada a objetos
    - Buenas prácticas de seguridad
    """
    
    def __init__(self):
        """
        Constructor: Se ejecuta cuando creamos un objeto de esta clase
        """
        # Inicializar el nodo ROS con un nombre único
        rospy.init_node('test_ctrl_c', anonymous=True)
        
        # Crear un publicador para enviar comandos al robot
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Configurar manejadores de señales para diferentes tipos de interrupciones
        signal.signal(signal.SIGINT, self.signal_handler)   # Ctrl+C
        signal.signal(signal.SIGTERM, self.signal_handler)   # Terminación del sistema
        
        # Mostrar información inicial
        rospy.loginfo("=" * 60)
        rospy.loginfo("🧪 TEST DE Ctrl+C")
        rospy.loginfo("=" * 60)
        rospy.loginfo("Este script hará que el robot se mueva en círculo")
        rospy.loginfo("Presiona Ctrl+C en cualquier momento para detenerlo")
        rospy.loginfo("=" * 60)
        
    def signal_handler(self, signum, frame):
        """
        Manejador de señales que se ejecuta cuando presionas Ctrl+C
        
        Esta función es CRÍTICA para la seguridad porque:
        - Se ejecuta inmediatamente cuando detecta Ctrl+C
        - Detiene el robot de forma segura
        - Previene daños al hardware
        - Es un patrón estándar en robótica
        
        Args:
            signum: Número de la señal recibida (SIGINT = 2 para Ctrl+C)
            frame: Frame actual de ejecución (no lo usamos)
        """
        rospy.logwarn("")
        rospy.logwarn("=" * 60)
        rospy.logwarn("✋ Ctrl+C DETECTADO!")
        rospy.logwarn("🛑 Deteniendo robot...")
        rospy.logwarn("=" * 60)
        
        # Detener robot inmediatamente
        cmd = Twist()
        cmd.linear.x = 0.0   # Sin movimiento hacia adelante/atrás
        cmd.angular.z = 0.0  # Sin rotación
        
        # Enviar comando de parada varias veces para asegurar
        for _ in range(10):  # Repetir 10 veces
            self.publisher.publish(cmd)      # Enviar comando al robot
            rospy.sleep(0.1)                  # Esperar 0.1 segundos entre envíos
        
        rospy.loginfo("✅ Robot detenido correctamente")
        rospy.loginfo("✅ Manejo de Ctrl+C funciona perfectamente!")
        rospy.loginfo("👋 Saliendo...")
        sys.exit(0)  # Salir del programa
    
    def run(self):
        """
        Función principal que ejecuta el test de movimiento circular
        
        Esta función:
        - Hace que el robot se mueva en círculo indefinidamente
        - Muestra el tiempo transcurrido cada 5 segundos
        - Demuestra que Ctrl+C funciona correctamente
        - Es útil para probar la seguridad del sistema
        """
        rospy.loginfo("")
        rospy.loginfo("🔄 Iniciando movimiento en círculo...")
        rospy.loginfo("   (El robot girará indefinidamente hasta que presiones Ctrl+C)")
        rospy.loginfo("")
        
        # Configurar frecuencia de envío de comandos
        rate = rospy.Rate(10)  # 10 Hz = 10 mensajes por segundo
        contador = 0           # Contador para medir tiempo transcurrido
        
        # Bucle principal - se ejecuta hasta que se presione Ctrl+C
        while not rospy.is_shutdown():
            # Crear comando de movimiento circular
            cmd = Twist()
            cmd.linear.x = 0.2   # Velocidad hacia adelante (0.2 m/s)
            cmd.angular.z = 0.3  # Velocidad de giro (0.3 rad/s)
            
            # Enviar comando al robot
            self.publisher.publish(cmd)
            
            # Incrementar contador
            contador += 1
            
            # Mostrar tiempo transcurrido cada 5 segundos
            # contador / 10 = segundos (porque enviamos 10 mensajes por segundo)
            if contador % 50 == 0:  # Cada 50 iteraciones = 5 segundos
                tiempo_segundos = contador / 10
                rospy.loginfo(f"⏱️  Tiempo transcurrido: {tiempo_segundos:.1f} segundos - Presiona Ctrl+C para detener")
            
            # Esperar según la frecuencia configurada
            rate.sleep()

# ============================================
# FUNCIÓN PRINCIPAL
# ============================================
if __name__ == '__main__':
    """
    Punto de entrada del programa.
    
    Esta sección:
    - Crea una instancia de la clase TestCtrlC
    - Ejecuta el test
    - Maneja excepciones de forma segura
    """
    try:
        # Crear objeto de la clase TestCtrlC
        test = TestCtrlC()
        
        # Ejecutar el test
        test.run()
        
    except rospy.ROSInterruptException:
        # Esta excepción se lanza cuando ROS se interrumpe
        rospy.loginfo("ROSInterruptException - Programa interrumpido por ROS")
        
    except Exception as e:
        # Capturar cualquier otro error inesperado
        rospy.logerr(f"Error inesperado: {e}")
        
    finally:
        # Esta sección siempre se ejecuta, incluso si hay errores
        rospy.loginfo("🧪 Test de Ctrl+C finalizado")

