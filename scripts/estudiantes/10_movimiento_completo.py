#!/usr/bin/env python3
"""
Script 1: Control de Movimiento del Sphero RVR
Este script enseña cómo mover el robot en traslación y rotación

ESTE SCRIPT ENSEÑA PROGRAMACIÓN ORIENTADA A OBJETOS
- Demuestra el uso de clases para organizar código
- Enseña métodos de movimiento reutilizables
- Muestra buenas prácticas de programación
- Introduce conceptos de encapsulación

INSTRUCCIONES:
1. Asegúrate de que el robot esté conectado y ROS esté corriendo
2. Ejecuta: python3 10_movimiento_completo.py
3. Observa cómo se mueve el robot
4. Modifica los valores y experimenta
"""

# ============================================
# IMPORTACIONES NECESARIAS
# ============================================
import rospy                      # Biblioteca principal de ROS para Python
from geometry_msgs.msg import Twist  # Mensaje para enviar comandos de velocidad
import time                       # Para controlar tiempos de espera
import signal                     # Para manejar señales del sistema (Ctrl+C)
import sys                        # Para salir del programa

# ============================================
# CLASE PRINCIPAL
# ============================================
class ControladorMovimiento:
    """
    Esta clase controla el movimiento del robot Sphero RVR.
    
    CONCEPTOS DE PROGRAMACIÓN ORIENTADA A OBJETOS:
    - Encapsulación: Todos los métodos de movimiento están en una clase
    - Reutilización: Los métodos pueden ser llamados múltiples veces
    - Organización: El código está bien estructurado y es fácil de mantener
    - Abstracción: Ocultamos la complejidad de ROS detrás de métodos simples
    """
    
    def __init__(self):
        """
        Constructor: Se ejecuta cuando creamos un objeto de esta clase.
        
        Esta función:
        - Configura la conexión con ROS
        - Crea el publicador para enviar comandos
        - Configura el manejo de señales de emergencia
        - Inicializa variables importantes
        """
        # Inicializar el nodo ROS con un nombre único
        # anonymous=True permite múltiples instancias del mismo nodo
        rospy.init_node('controlador_movimiento', anonymous=True)
        
        # Crear un publicador para enviar comandos al robot
        # - '/cmd_vel': nombre del tópico (canal de comunicación)
        # - Twist: tipo de mensaje (velocidad lineal + angular)
        # - queue_size=10: tamaño de la cola de mensajes
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Frecuencia de envío: 10 Hz = 10 mensajes por segundo
        # Esto asegura que el robot reciba comandos de forma constante
        self.rate = rospy.Rate(10)
        
        # Configurar manejadores de señales para Ctrl+C y terminación del sistema
        signal.signal(signal.SIGINT, self.signal_handler)   # Ctrl+C
        signal.signal(signal.SIGTERM, self.signal_handler)   # Terminación del sistema
        
        # Mostrar información de inicialización
        rospy.loginfo("=" * 60)
        rospy.loginfo("✅ Controlador de movimiento iniciado")
        rospy.loginfo("📡 Publicando en: /cmd_vel")
        rospy.loginfo("⚠️  Presiona Ctrl+C para detener el robot y salir")
        rospy.loginfo("=" * 60)
        
    def crear_comando_velocidad(self, vel_lineal, vel_angular):
        """
        Crea un mensaje de tipo Twist con las velocidades deseadas.
        
        Esta función es MUY IMPORTANTE porque:
        - Centraliza la creación de comandos de movimiento
        - Evita duplicación de código
        - Hace el código más mantenible
        - Enseña la estructura del mensaje Twist
        
        Args:
            vel_lineal: Velocidad de traslación en m/s
                       - Positivo = adelante (ej: 0.3)
                       - Negativo = atrás (ej: -0.3)
                       - 0 = sin movimiento lineal
            vel_angular: Velocidad de rotación en rad/s
                        - Positivo = izquierda (ej: 0.5)
                        - Negativo = derecha (ej: -0.5)
                        - 0 = sin rotación
        
        Returns:
            Objeto Twist con las velocidades configuradas
        """
        # Crear un mensaje vacío de tipo Twist
        cmd = Twist()
        
        # ============================================
        # VELOCIDAD LINEAL (Traslación)
        # ============================================
        # Solo usamos X porque el robot se mueve en 2D (plano horizontal)
        cmd.linear.x = vel_lineal    # Adelante/Atrás (eje X)
        cmd.linear.y = 0.0           # Lateral (eje Y) - no usado en RVR
        cmd.linear.z = 0.0           # Vertical (eje Z) - no usado en RVR
        
        # ============================================
        # VELOCIDAD ANGULAR (Rotación)
        # ============================================
        # Solo usamos Z porque el robot gira sobre su eje vertical
        cmd.angular.x = 0.0          # Rotación sobre eje X - no usado
        cmd.angular.y = 0.0          # Rotación sobre eje Y - no usado
        cmd.angular.z = vel_angular  # Rotación sobre eje Z (giro izquierda/derecha)
        
        return cmd
    
    def mover_adelante(self, velocidad=0.3, duracion=2.0):
        """
        Mueve el robot hacia adelante.
        
        Este método demuestra:
        - Cómo crear movimientos controlados por tiempo
        - Uso de bucles while con condiciones de salida
        - Control de frecuencia con rospy.Rate
        - Manejo seguro de interrupciones
        
        Args:
            velocidad: Rapidez del movimiento en m/s
                      - Rango recomendado: 0.0 - 1.0
                      - 0.3 m/s es una velocidad moderada
                      - 0.5 m/s es rápida
            duracion: Tiempo de movimiento en segundos
                     - 1.0 = 1 segundo
                     - 2.0 = 2 segundos
        """
        rospy.loginfo(f"🔵 Moviendo ADELANTE - Velocidad: {velocidad} m/s por {duracion}s")
        
        # Crear comando: velocidad lineal positiva, sin rotación
        cmd = self.crear_comando_velocidad(velocidad, 0.0)
        
        # Enviar comando repetidamente durante el tiempo especificado
        start_time = time.time()  # Marcar tiempo de inicio
        while (time.time() - start_time) < duracion and not rospy.is_shutdown():
            # Enviar comando al robot
            self.publisher.publish(cmd)
            # Esperar según la frecuencia configurada (10 Hz)
            self.rate.sleep()
        
        # Detener el robot al finalizar
        self.detener()
    
    def mover_atras(self, velocidad=0.3, duracion=2.0):
        """
        Mueve el robot hacia atrás
        
        Args:
            velocidad: Rapidez del movimiento (m/s). Rango: 0.0 - 1.0
            duracion: Tiempo de movimiento (segundos)
        """
        rospy.loginfo(f"🔴 Moviendo ATRÁS - Velocidad: {velocidad} m/s por {duracion}s")
        
        # Crear comando: velocidad lineal NEGATIVA, sin rotación
        cmd = self.crear_comando_velocidad(-velocidad, 0.0)
        
        start_time = time.time()
        while (time.time() - start_time) < duracion and not rospy.is_shutdown():
            self.publisher.publish(cmd)
            self.rate.sleep()
        
        self.detener()
    
    def girar_izquierda(self, velocidad_angular=0.5, duracion=2.0):
        """
        Gira el robot hacia la izquierda (sobre su propio eje)
        
        Args:
            velocidad_angular: Rapidez de rotación (rad/s)
            duracion: Tiempo de rotación (segundos)
        """
        rospy.loginfo(f"⬅️  Girando IZQUIERDA - Vel. Angular: {velocidad_angular} rad/s por {duracion}s")
        
        # Crear comando: sin traslación, velocidad angular POSITIVA
        cmd = self.crear_comando_velocidad(0.0, velocidad_angular)
        
        start_time = time.time()
        while (time.time() - start_time) < duracion and not rospy.is_shutdown():
            self.publisher.publish(cmd)
            self.rate.sleep()
        
        self.detener()
    
    def girar_derecha(self, velocidad_angular=0.5, duracion=2.0):
        """
        Gira el robot hacia la derecha (sobre su propio eje)
        
        Args:
            velocidad_angular: Rapidez de rotación (rad/s)
            duracion: Tiempo de rotación (segundos)
        """
        rospy.loginfo(f"➡️  Girando DERECHA - Vel. Angular: {velocidad_angular} rad/s por {duracion}s")
        
        # Crear comando: sin traslación, velocidad angular NEGATIVA
        cmd = self.crear_comando_velocidad(0.0, -velocidad_angular)
        
        start_time = time.time()
        while (time.time() - start_time) < duracion and not rospy.is_shutdown():
            self.publisher.publish(cmd)
            self.rate.sleep()
        
        self.detener()
    
    def girar_avanzando(self, vel_lineal=0.3, vel_angular=0.3, duracion=2.0):
        """
        Mueve el robot en curva (combina traslación y rotación)
        
        Args:
            vel_lineal: Velocidad de avance (m/s)
            vel_angular: Velocidad de giro (rad/s)
            duracion: Tiempo de movimiento (segundos)
        """
        rospy.loginfo(f"🔄 Movimiento en CURVA - Lin: {vel_lineal} m/s, Ang: {vel_angular} rad/s")
        
        # Crear comando: COMBINACIÓN de velocidad lineal y angular
        cmd = self.crear_comando_velocidad(vel_lineal, vel_angular)
        
        start_time = time.time()
        while (time.time() - start_time) < duracion and not rospy.is_shutdown():
            self.publisher.publish(cmd)
            self.rate.sleep()
        
        self.detener()
    
    def detener(self):
        """
        Detiene completamente el robot.
        
        Esta función es CRÍTICA para la seguridad porque:
        - Asegura que el robot se detenga completamente
        - Envía múltiples comandos de parada para garantizar la detención
        - Es llamada automáticamente al final de cada movimiento
        - Previene que el robot siga moviéndose inesperadamente
        """
        rospy.loginfo("🛑 DETENIENDO robot")
        
        # Crear comando con velocidades en cero
        cmd = self.crear_comando_velocidad(0.0, 0.0)
        
        # Enviar comando de detención varias veces para asegurar
        # Repetir 10 veces porque a veces un solo comando no es suficiente
        for _ in range(10):
            self.publisher.publish(cmd)  # Enviar comando de parada
            self.rate.sleep()           # Esperar según la frecuencia
    
    def signal_handler(self, signum, frame):
        """
        Manejador de señales para Ctrl+C y otras interrupciones
        
        Args:
            signum: Número de la señal recibida
            frame: Frame actual de ejecución
        """
        rospy.logwarn("")
        rospy.logwarn("=" * 60)
        rospy.logwarn("⚠️  SEÑAL DE INTERRUPCIÓN RECIBIDA (Ctrl+C)")
        rospy.logwarn("🛑 Deteniendo robot de emergencia...")
        rospy.logwarn("=" * 60)
        
        # Detener el robot inmediatamente
        self.detener()
        
        rospy.loginfo("✅ Robot detenido correctamente")
        rospy.loginfo("👋 Saliendo del programa...")
        
        # Salir del programa
        sys.exit(0)
    
    def ejecutar_secuencia_demo(self):
        """
        Ejecuta una secuencia de demostración de movimientos
        """
        rospy.loginfo("=" * 60)
        rospy.loginfo("🚀 INICIANDO SECUENCIA DE DEMOSTRACIÓN")
        rospy.loginfo("=" * 60)
        
        # Esperar un momento antes de empezar
        rospy.sleep(1.0)
        
        # 1. Avanzar
        self.mover_adelante(velocidad=0.3, duracion=3.0)
        rospy.sleep(1.0)
        
        # 2. Retroceder
        self.mover_atras(velocidad=0.3, duracion=3.0)
        rospy.sleep(1.0)
        
        # 3. Girar a la izquierda
        self.girar_izquierda(velocidad_angular=0.5, duracion=2.0)
        rospy.sleep(1.0)
        
        # 4. Girar a la derecha
        self.girar_derecha(velocidad_angular=0.5, duracion=2.0)
        rospy.sleep(1.0)
        
        # 5. Movimiento en curva
        self.girar_avanzando(vel_lineal=0.3, vel_angular=0.3, duracion=3.0)
        rospy.sleep(1.0)
        
        # 6. Dibujar un cuadrado
        rospy.loginfo("📐 Dibujando un CUADRADO")
        for i in range(4):
            rospy.loginfo(f"   Lado {i+1}/4")
            self.mover_adelante(velocidad=0.3, duracion=2.0)
            self.girar_izquierda(velocidad_angular=0.5, duracion=3.14)  # ~90 grados
        #1.57
        rospy.loginfo("=" * 60)
        rospy.loginfo("✅ SECUENCIA COMPLETADA")
        rospy.loginfo("=" * 60)
        rospy.loginfo("")
        rospy.loginfo("💡 SUGERENCIAS PARA EXPERIMENTAR:")
        rospy.loginfo("   1. Modifica las velocidades (líneas con 'velocidad=')")
        rospy.loginfo("   2. Modifica las duraciones (líneas con 'duracion=')")
        rospy.loginfo("   3. Crea tu propia secuencia de movimientos")
        rospy.loginfo("   4. Intenta dibujar un triángulo o un círculo")
        rospy.loginfo("=" * 60)

# ============================================
# FUNCIÓN PRINCIPAL
# ============================================
def main():
    """
    Función principal que se ejecuta al iniciar el script.
    
    Esta función demuestra:
    - Manejo de excepciones en Python
    - Uso de try/except/finally
    - Creación de objetos de clase
    - Buenas prácticas de programación
    """
    try:
        # Crear objeto controlador
        # Esto ejecuta el constructor __init__ de la clase
        controlador = ControladorMovimiento()
        
        # Ejecutar secuencia de demostración
        # Esto llama al método que ejecuta todos los movimientos
        controlador.ejecutar_secuencia_demo()
        
    except rospy.ROSInterruptException:
        # Esta excepción se lanza cuando ROS se interrumpe (Ctrl+C)
        rospy.loginfo("❌ Programa interrumpido por el usuario")
    except Exception as e:
        # Capturar cualquier otro error inesperado
        rospy.logerr(f"❌ Error: {e}")
    finally:
        # Esta sección siempre se ejecuta, incluso si hay errores
        rospy.loginfo("👋 Finalizando programa")

# ============================================
# PUNTO DE ENTRADA
# ============================================
if __name__ == '__main__':
    """
    Esta línea es MUY IMPORTANTE en Python:
    - Solo ejecuta main() si el archivo se ejecuta directamente
    - Permite importar este archivo como módulo sin ejecutar el código
    - Es una buena práctica de programación
    """
    main()

