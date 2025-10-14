#!/usr/bin/env python3
"""
Script 2: Sensor de Color del Sphero RVR
Este script enseña cómo leer el sensor de color y tomar decisiones

INSTRUCCIONES:
1. Asegúrate de que el robot esté conectado y ROS esté corriendo
2. Ejecuta: python3 2_sensor_color.py
3. Selecciona el modo que deseas probar
4. Experimenta con diferentes colores
"""

# ============================================
# IMPORTACIONES
# ============================================
import rospy
from atriz_rvr_msgs.msg import Color  # Mensaje personalizado para color
from geometry_msgs.msg import Twist   # Para controlar movimiento
from std_srvs.srv import SetBool     # Servicio para activar/desactivar sensor
import time
import signal                         # Para manejar señales del sistema (Ctrl+C)
import sys                            # Para salir del programa

# ============================================
# CLASE PRINCIPAL
# ============================================
class DetectorColor:
    """
    Esta clase lee el sensor de color y controla el robot según el color detectado
    """
    
    def __init__(self):
        """
        Constructor: Inicializa el detector de color
        """
        rospy.init_node('detector_color', anonymous=True)
        
        # Variables para almacenar información
        self.ultimo_color = None        # Último color detectado
        self.confianza = 0              # Confianza de la detección (0-100)
        self.contador_lecturas = 0      # Contador de lecturas
        
        # Crear suscriptor al tópico de color
        # Cada vez que llegue un mensaje, se llamará a self.callback_color
        rospy.Subscriber('/color', Color, self.callback_color)
        
        # Crear publicador para controlar el robot
        self.publisher_movimiento = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Esperar a que el servicio esté disponible
        rospy.loginfo("⏳ Esperando servicio /enable_color...")
        rospy.wait_for_service('/enable_color')
        
        # Crear cliente del servicio para habilitar el sensor
        self.servicio_color = rospy.ServiceProxy('/enable_color', SetBool)
        
        # Configurar manejador de señales para Ctrl+C
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("✅ Detector de color iniciado")
        rospy.loginfo("📡 Suscrito a: /color")
        rospy.loginfo("⚠️  Presiona Ctrl+C para detener el robot y salir")
        rospy.loginfo("=" * 60)
        
    def callback_color(self, msg):
        """
        Esta función se llama automáticamente cada vez que llega un mensaje de color
        
        Args:
            msg: Mensaje de tipo Color que contiene:
                - rgb_color: Lista [R, G, B] con valores 0-255
                - confidence: Confianza de la detección 0-100
        """
        # Extraer valores RGB del mensaje y convertir a enteros
        r = int(msg.rgb_color[0])
        g = int(msg.rgb_color[1])
        b = int(msg.rgb_color[2])
        
        # Guardar información
        self.ultimo_color = [r, g, b]
        self.confianza = int(msg.confidence)  # Convertir a entero
        self.contador_lecturas += 1
        
        # Identificar el color dominante
        color_nombre = self.identificar_color(r, g, b)
        
        # Mostrar información cada 10 lecturas para no saturar la pantalla
        if self.contador_lecturas % 10 == 0:
            rospy.loginfo(f"🎨 Color: {color_nombre:12s} | RGB: ({r:3d}, {g:3d}, {b:3d}) | Confianza: {self.confianza:3d}%")
    
    def identificar_color(self, r, g, b):
        """
        Identifica el color dominante basándose en los valores RGB
        
        Args:
            r, g, b: Valores de Rojo, Verde y Azul (0-255)
        
        Returns:
            String con el nombre del color detectado
        """
        # IMPORTANTE: Estos umbrales son aproximados
        # Cada grupo deberá ajustarlos según su pista
        
        # Calcular cuál componente es mayor
        max_val = max(r, g, b)
        min_val = min(r, g, b)
        
        # Si todos los valores son similares
        if max_val - min_val < 30:
            if max_val > 200:
                return "BLANCO"
            elif max_val < 50:
                return "NEGRO"
            else:
                return "GRIS"
        
        # Identificar color dominante
        if r > g and r > b:
            if r > 150:
                return "ROJO"
        elif g > r and g > b:
            if g > 150:
                return "VERDE"
        elif b > r and b > g:
            if b > 150:
                return "AZUL"
        
        # Colores secundarios
        if r > 150 and g > 150 and b < 100:
            return "AMARILLO"
        elif r > 150 and b > 150 and g < 100:
            return "MAGENTA"
        elif g > 150 and b > 150 and r < 100:
            return "CYAN"
        
        return "DESCONOCIDO"
    
    def habilitar_sensor(self, estado=True):
        """
        Activa o desactiva el sensor de color
        
        Args:
            estado: True para activar, False para desactivar
        """
        try:
            respuesta = self.servicio_color(estado)
            if respuesta.success:
                accion = "activado" if estado else "desactivado"
                rospy.loginfo(f"✅ Sensor de color {accion}")
            else:
                rospy.logwarn(f"⚠️  No se pudo cambiar estado del sensor")
        except rospy.ServiceException as e:
            rospy.logerr(f"❌ Error al llamar servicio: {e}")
    
    def reaccionar_a_color(self, color_nombre):
        """
        Controla el robot según el color detectado
        
        Args:
            color_nombre: Nombre del color detectado
        """
        cmd = Twist()
        
        # REGLAS DE COMPORTAMIENTO (puedes modificarlas)
        if color_nombre == "ROJO":
            # Si detecta ROJO: Detenerse
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            rospy.loginfo("🔴 ROJO detectado - DETENIENDO")
            
        elif color_nombre == "VERDE":
            # Si detecta VERDE: Avanzar
            cmd.linear.x = 0.3
            cmd.angular.z = 0.0
            rospy.loginfo("🟢 VERDE detectado - AVANZANDO")
            
        elif color_nombre == "AZUL":
            # Si detecta AZUL: Girar a la izquierda
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5
            rospy.loginfo("🔵 AZUL detectado - GIRANDO IZQUIERDA")
            
        elif color_nombre == "AMARILLO":
            # Si detecta AMARILLO: Girar a la derecha
            cmd.linear.x = 0.0
            cmd.angular.z = -0.5
            rospy.loginfo("🟡 AMARILLO detectado - GIRANDO DERECHA")
            
        elif color_nombre == "NEGRO":
            # NEGRO es típico de una línea - útil para seguidor
            cmd.linear.x = 0.2
            cmd.angular.z = 0.0
            rospy.loginfo("⚫ NEGRO detectado - AVANCE LENTO")
            
        elif color_nombre == "BLANCO":
            # BLANCO es el fondo típico
            cmd.linear.x = 0.3
            cmd.angular.z = 0.0
            rospy.loginfo("⚪ BLANCO detectado - AVANCE NORMAL")
            
        # Enviar comando al robot
        self.publisher_movimiento.publish(cmd)
    
    def modo_monitoreo(self, duracion=30.0):
        """
        Modo que solo monitorea y muestra los colores detectados
        
        Args:
            duracion: Tiempo de monitoreo en segundos
        """
        rospy.loginfo("")
        rospy.loginfo("=" * 60)
        rospy.loginfo("🔍 MODO MONITOREO DE COLORES")
        rospy.loginfo(f"📊 Monitoreando durante {duracion} segundos")
        rospy.loginfo("💡 Mueve el robot sobre diferentes superficies")
        rospy.loginfo("=" * 60)
        rospy.loginfo("")
        
        # Activar sensor
        self.habilitar_sensor(True)
        
        # Esperar el tiempo especificado
        rospy.sleep(duracion)
        
        # Desactivar sensor
        self.habilitar_sensor(False)
        
        rospy.loginfo("")
        rospy.loginfo("=" * 60)
        rospy.loginfo(f"✅ Monitoreo completado - Total lecturas: {self.contador_lecturas}")
        rospy.loginfo("=" * 60)
    
    def modo_reaccion(self, duracion=60.0):
        """
        Modo que hace reaccionar al robot según el color detectado
        
        Args:
            duracion: Tiempo de reacción en segundos
        """
        rospy.loginfo("")
        rospy.loginfo("=" * 60)
        rospy.loginfo("🤖 MODO REACCIÓN A COLORES")
        rospy.loginfo(f"⏱️  Reaccionando durante {duracion} segundos")
        rospy.loginfo("⚠️  ASEGÚRATE DE QUE HAY ESPACIO LIBRE")
        rospy.loginfo("=" * 60)
        rospy.loginfo("")
        
        # Esperar confirmación
        input("Presiona ENTER para comenzar...")
        
        # Activar sensor
        self.habilitar_sensor(True)
        
        rate = rospy.Rate(10)  # 10 Hz
        start_time = time.time()
        
        while (time.time() - start_time) < duracion and not rospy.is_shutdown():
            # Si tenemos un color detectado, reaccionar
            if self.ultimo_color is not None:
                r, g, b = self.ultimo_color
                color_nombre = self.identificar_color(r, g, b)
                self.reaccionar_a_color(color_nombre)
            
            rate.sleep()
        
        # Detener robot y desactivar sensor
        self.detener_robot()
        self.habilitar_sensor(False)
        
        rospy.loginfo("")
        rospy.loginfo("=" * 60)
        rospy.loginfo("✅ Modo reacción completado")
        rospy.loginfo("=" * 60)
    
    def calibrar_colores(self, duracion_por_color=5.0):
        """
        Modo de calibración: ayuda a identificar los valores RGB de cada color
        
        Args:
            duracion_por_color: Tiempo para medir cada color
        """
        colores_calibrar = ["BLANCO", "NEGRO", "ROJO", "VERDE", "AZUL"]
        resultados = {}
        
        rospy.loginfo("")
        rospy.loginfo("=" * 60)
        rospy.loginfo("🎯 MODO CALIBRACIÓN DE COLORES")
        rospy.loginfo("=" * 60)
        rospy.loginfo("📋 Instrucciones:")
        rospy.loginfo("   1. Prepara papeles o superficies de cada color")
        rospy.loginfo("   2. Cuando se indique, coloca el robot sobre ese color")
        rospy.loginfo("   3. Presiona ENTER para iniciar la medición")
        rospy.loginfo("   4. Anota los valores RGB obtenidos")
        rospy.loginfo("=" * 60)
        rospy.loginfo("")
        
        # Activar sensor
        self.habilitar_sensor(True)
        rospy.sleep(1.0)
        
        for color in colores_calibrar:
            rospy.loginfo("")
            rospy.loginfo(f"📏 COLOR A CALIBRAR: {color}")
            rospy.loginfo(f"   Coloca el robot sobre una superficie de color {color}")
            input(f"   Presiona ENTER cuando esté listo...")
            
            # Reiniciar mediciones
            mediciones_r = []
            mediciones_g = []
            mediciones_b = []
            
            # Tomar mediciones
            rospy.loginfo(f"   ⏳ Tomando mediciones durante {duracion_por_color} segundos...")
            start_time = time.time()
            while (time.time() - start_time) < duracion_por_color:
                if self.ultimo_color is not None:
                    mediciones_r.append(self.ultimo_color[0])
                    mediciones_g.append(self.ultimo_color[1])
                    mediciones_b.append(self.ultimo_color[2])
                rospy.sleep(0.1)
            
            # Calcular promedios
            if len(mediciones_r) > 0:
                r_avg = sum(mediciones_r) / len(mediciones_r)
                g_avg = sum(mediciones_g) / len(mediciones_g)
                b_avg = sum(mediciones_b) / len(mediciones_b)
                
                resultados[color] = {
                    'R': r_avg,
                    'G': g_avg,
                    'B': b_avg,
                    'muestras': len(mediciones_r)
                }
                
                rospy.loginfo(f"   ✅ Resultado: R={r_avg:.1f}, G={g_avg:.1f}, B={b_avg:.1f} ({len(mediciones_r)} muestras)")
        
        # Mostrar resumen
        rospy.loginfo("")
        rospy.loginfo("=" * 60)
        rospy.loginfo("📊 RESUMEN DE CALIBRACIÓN")
        rospy.loginfo("=" * 60)
        rospy.loginfo("")
        rospy.loginfo("Color      |    R    |    G    |    B    ")
        rospy.loginfo("-" * 60)
        for color, valores in resultados.items():
            rospy.loginfo(f"{color:10s} | {valores['R']:6.1f} | {valores['G']:6.1f} | {valores['B']:6.1f}")
        rospy.loginfo("=" * 60)
        rospy.loginfo("")
        rospy.loginfo("💡 Usa estos valores para ajustar la función 'identificar_color()'")
        rospy.loginfo("=" * 60)
        
        # Desactivar sensor
        self.habilitar_sensor(False)
    
    def detener_robot(self):
        """
        Detiene el robot completamente
        """
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        
        for _ in range(10):
            self.publisher_movimiento.publish(cmd)
            rospy.sleep(0.1)
    
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
        rospy.logwarn("🛑 Deteniendo robot y desactivando sensor...")
        rospy.logwarn("=" * 60)
        
        # Detener el robot inmediatamente
        self.detener_robot()
        
        # Desactivar el sensor de color
        try:
            self.habilitar_sensor(False)
        except:
            pass
        
        rospy.loginfo("✅ Robot detenido y sensor desactivado")
        rospy.loginfo("👋 Saliendo del programa...")
        
        # Salir del programa
        sys.exit(0)

# ============================================
# FUNCIÓN PRINCIPAL
# ============================================
def main():
    """
    Función principal
    """
    try:
        # Crear detector
        detector = DetectorColor()
        
        # MENÚ DE OPCIONES
        print("")
        print("=" * 60)
        print("🎨 SENSOR DE COLOR - MENÚ DE OPCIONES")
        print("=" * 60)
        print("")
        print("1. Modo Calibración (recomendado primero)")
        print("   → Obtén los valores RGB reales de tu pista")
        print("")
        print("2. Modo Monitoreo (solo observar colores)")
        print("   → El robot NO se moverá, solo mostrará colores detectados")
        print("")
        print("3. Modo Reacción (robot reacciona a colores)")
        print("   → El robot SE MOVERÁ según el color detectado")
        print("")
        print("=" * 60)
        print("")
        
        opcion = input("Seleccione una opción (1-3): ")
        
        if opcion == "1":
            detector.calibrar_colores(duracion_por_color=5.0)
        elif opcion == "2":
            detector.modo_monitoreo(duracion=30.0)
        elif opcion == "3":
            detector.modo_reaccion(duracion=60.0)
        else:
            rospy.logwarn("❌ Opción no válida")
        
    except rospy.ROSInterruptException:
        rospy.loginfo("❌ Programa interrumpido")
    except KeyboardInterrupt:
        rospy.loginfo("❌ Programa interrumpido por teclado (Ctrl+C)")
    except Exception as e:
        rospy.logerr(f"❌ Error inesperado: {e}")
    finally:
        rospy.loginfo("")
        rospy.loginfo("👋 Finalizando programa")
        rospy.loginfo("")

# ============================================
# PUNTO DE ENTRADA
# ============================================
if __name__ == '__main__':
    main()

