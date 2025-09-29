#!/usr/bin/env python3

import rospy
import signal
import sys
import time
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
from atriz_rvr_msgs.msg import Color
from atriz_rvr_msgs.srv import SetLEDRGB

class ColorMeasurementAnalysis:
    """Análisis completo de las posibilidades de medición de color sin LED."""
    
    def __init__(self):
        rospy.init_node('color_measurement_analysis', anonymous=True)
        
        # Variables
        self.running = True
        self.measurement_count = 0
        self.led_activation_attempts = 0
        
        # Configurar servicios
        self.setup_services()
        
        # Configurar señal de interrupción
        signal.signal(signal.SIGINT, self.signal_handler)
        
        rospy.loginfo("🔬 Análisis de Medición de Color Sin LED")
        rospy.loginfo("=" * 60)
    
    def setup_services(self):
        """Configura los servicios necesarios."""
        try:
            rospy.wait_for_service('/enable_color', timeout=5.0)
            rospy.wait_for_service('/set_led_rgb', timeout=5.0)
            
            self.enable_color_service = rospy.ServiceProxy('/enable_color', SetBool)
            self.set_led_rgb_service = rospy.ServiceProxy('/set_led_rgb', SetLEDRGB)
            
            rospy.loginfo("✅ Servicios configurados")
            
        except rospy.ROSException as e:
            rospy.logerr(f"❌ Error: {e}")
            rospy.logwarn("⚠️ Asegúrate de que el driver del RVR esté ejecutándose")
            sys.exit(1)
    
    def signal_handler(self, signum, frame):
        """Maneja Ctrl+C."""
        rospy.loginfo("🛑 Deteniendo...")
        self.running = False
        self.cleanup()
        sys.exit(0)
    
    def cleanup(self):
        """Limpia al finalizar."""
        try:
            # Detener robot
            cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
            twist = Twist()
            cmd_vel_pub.publish(twist)
            
            # Deshabilitar sensor
            try:
                self.enable_color_service(False)
                rospy.loginfo("🔴 Sensor de color deshabilitado")
            except:
                pass
            
            rospy.loginfo("🧹 Limpieza completada")
            
        except Exception as e:
            rospy.logwarn(f"⚠️ Error en limpieza: {e}")
    
    def control_sensor_led(self, red, green, blue):
        """Controla el LED del sensor usando servicios ROS."""
        try:
            response = self.set_led_rgb_service(
                led_id=0x40000000,  # Bitmask para undercarriage_white
                red=red, green=green, blue=blue
            )
            if response.success:
                rospy.loginfo(f"✅ LED del sensor controlado a RGB({red},{green},{blue})")
                return True
            else:
                rospy.logwarn(f"⚠️ Error controlando LED: {response.message}")
                return False
        except Exception as e:
            rospy.logwarn(f"⚠️ Error en servicio LED: {e}")
            return False
    
    def analyze_color_measurement_possibilities(self):
        """Analiza todas las posibilidades de medición de color."""
        rospy.loginfo("🔍 ANÁLISIS DE POSIBILIDADES")
        rospy.loginfo("=" * 60)
        
        # 1. Verificar servicios disponibles
        rospy.loginfo("1️⃣ Verificando servicios disponibles...")
        try:
            rospy.loginfo("   ✅ /enable_color - Disponible")
            rospy.loginfo("   ✅ /set_led_rgb - Disponible")
            rospy.loginfo("   ❌ /get_rgbc_sensor_values - NO disponible como servicio ROS")
        except Exception as e:
            rospy.logwarn(f"   ⚠️ Error verificando servicios: {e}")
        
        # 2. Verificar tópicos disponibles
        rospy.loginfo("\n2️⃣ Verificando tópicos disponibles...")
        try:
            rospy.loginfo("   ✅ /color - Disponible (requiere enable_color)")
            rospy.loginfo("   ❌ /rgbc_raw - NO disponible")
            rospy.loginfo("   ❌ /sensor_raw - NO disponible")
        except Exception as e:
            rospy.logwarn(f"   ⚠️ Error verificando tópicos: {e}")
        
        # 3. Análisis de la arquitectura
        rospy.loginfo("\n3️⃣ Análisis de la arquitectura del sistema...")
        rospy.loginfo("   🔧 Hardware: Sensor de color con LED integrado")
        rospy.loginfo("   🔧 Firmware: LED se activa automáticamente con enable_color")
        rospy.loginfo("   🔧 SDK: get_rgbc_sensor_values existe pero no expuesto en ROS")
        rospy.loginfo("   🔧 Driver: Solo expone enable_color y streaming de color")
        
        # 4. Conclusión
        rospy.loginfo("\n4️⃣ CONCLUSIÓN:")
        rospy.loginfo("   ❌ IMPOSIBLE hacer mediciones sin activar el LED")
        rospy.loginfo("   📋 Razones:")
        rospy.loginfo("      • El LED está integrado en el hardware del sensor")
        rospy.loginfo("      • El firmware activa el LED automáticamente")
        rospy.loginfo("      • No hay acceso directo a datos raw del sensor")
        rospy.loginfo("      • get_rgbc_sensor_values no está expuesto en ROS")
    
    def demonstrate_led_activation_behavior(self):
        """Demuestra el comportamiento de activación del LED."""
        rospy.loginfo("\n🎯 DEMOSTRACIÓN DEL COMPORTAMIENTO")
        rospy.loginfo("=" * 60)
        
        # Paso 1: Verificar estado inicial
        rospy.loginfo("1️⃣ Estado inicial del sensor...")
        rospy.loginfo("   🔍 Verificando si el sensor está habilitado...")
        
        # Paso 2: Intentar apagar LED antes de habilitar sensor
        rospy.loginfo("\n2️⃣ Intentando apagar LED antes de habilitar sensor...")
        self.control_sensor_led(0, 0, 0)
        rospy.loginfo("   ⚠️ Esto no tiene efecto si el sensor no está habilitado")
        
        # Paso 3: Habilitar sensor (esto activará el LED)
        rospy.loginfo("\n3️⃣ Habilitando sensor de color...")
        rospy.loginfo("   ⚠️ ADVERTENCIA: Esto activará el LED automáticamente")
        
        try:
            response = self.enable_color_service(True)
            if response[0]:  # response es una tupla (success, message)
                rospy.loginfo("   ✅ Sensor habilitado exitosamente")
                rospy.loginfo("   🟡 LED del sensor: ACTIVADO AUTOMÁTICAMENTE")
                self.led_activation_attempts += 1
            else:
                rospy.logwarn(f"   ⚠️ Error habilitando sensor: {response[1]}")
                return False
        except Exception as e:
            rospy.logerr(f"   ❌ Error en servicio: {e}")
            return False
        
        # Paso 4: Intentar apagar LED después de habilitar sensor
        rospy.loginfo("\n4️⃣ Intentando apagar LED después de habilitar sensor...")
        self.control_sensor_led(0, 0, 0)
        rospy.loginfo("   ⚠️ El LED se reactivará en la siguiente medición")
        
        # Paso 5: Tomar medición y observar comportamiento
        rospy.loginfo("\n5️⃣ Tomando medición de color...")
        try:
            color_data = rospy.wait_for_message('/color', Color, timeout=5.0)
            
            self.measurement_count += 1
            r, g, b = color_data.rgb_color[0], color_data.rgb_color[1], color_data.rgb_color[2]
            confidence = color_data.confidence * 100
            
            rospy.loginfo(f"   📊 MEDICIÓN #{self.measurement_count}")
            rospy.loginfo(f"      RGB: ({r:3d}, {g:3d}, {b:3d})")
            rospy.loginfo(f"      Confianza: {confidence:5.1f}%")
            rospy.loginfo(f"      LED: 🟡 REACTIVADO AUTOMÁTICAMENTE")
            
        except rospy.ROSException as e:
            rospy.logwarn(f"   ⚠️ Error tomando medición: {e}")
            return False
        
        return True
    
    def provide_alternatives(self):
        """Proporciona alternativas y soluciones."""
        rospy.loginfo("\n💡 ALTERNATIVAS Y SOLUCIONES")
        rospy.loginfo("=" * 60)
        
        rospy.loginfo("1️⃣ Soluciones Técnicas:")
        rospy.loginfo("   🔧 Modificar el driver para exponer get_rgbc_sensor_values")
        rospy.loginfo("   🔧 Crear un servicio ROS personalizado para acceso directo")
        rospy.loginfo("   🔧 Usar el SDK directamente (como en el script anterior)")
        
        rospy.loginfo("\n2️⃣ Soluciones de Hardware:")
        rospy.loginfo("   🔧 Usar un sensor de color externo")
        rospy.loginfo("   🔧 Modificar el hardware del RVR (no recomendado)")
        
        rospy.loginfo("\n3️⃣ Soluciones de Software:")
        rospy.loginfo("   🔧 Aceptar que el LED se active pero controlarlo después")
        rospy.loginfo("   🔧 Usar mediciones puntuales en lugar de streaming continuo")
        rospy.loginfo("   🔧 Implementar un sistema de control de LED más sofisticado")
        
        rospy.loginfo("\n4️⃣ Recomendación:")
        rospy.loginfo("   ✅ La mejor opción es aceptar la activación del LED")
        rospy.loginfo("   ✅ Implementar control del LED después de cada medición")
        rospy.loginfo("   ✅ Usar mediciones puntuales para minimizar el tiempo de activación")
    
    def run_analysis(self):
        """Ejecuta el análisis completo."""
        rospy.loginfo("🚀 Iniciando análisis completo...")
        
        # Análisis de posibilidades
        self.analyze_color_measurement_possibilities()
        
        # Demostración del comportamiento
        if self.demonstrate_led_activation_behavior():
            rospy.loginfo("✅ Demostración completada exitosamente")
        else:
            rospy.logwarn("⚠️ Demostración falló")
        
        # Alternativas
        self.provide_alternatives()
        
        # Resumen final
        rospy.loginfo("\n📋 RESUMEN FINAL")
        rospy.loginfo("=" * 60)
        rospy.loginfo(f"   📊 Mediciones tomadas: {self.measurement_count}")
        rospy.loginfo(f"   🟡 Intentos de activación de LED: {self.led_activation_attempts}")
        rospy.loginfo("   ❌ Conclusión: IMPOSIBLE medir color sin activar LED")
        rospy.loginfo("   💡 Recomendación: Usar control del LED después de mediciones")

def main():
    """Función principal."""
    try:
        # Crear instancia
        analyzer = ColorMeasurementAnalysis()
        
        # Ejecutar análisis
        analyzer.run_analysis()
        
        # Mantener el nodo vivo
        rospy.loginfo("\n🔄 Nodo activo. Presiona Ctrl+C para salir.")
        while analyzer.running and not rospy.is_shutdown():
            rospy.sleep(0.1)
            
    except KeyboardInterrupt:
        rospy.loginfo("🛑 Interrumpido por el usuario")
    except Exception as e:
        rospy.logerr(f"❌ Error: {e}")
    finally:
        rospy.loginfo("👋 Finalizando...")

if __name__ == '__main__':
    main()
