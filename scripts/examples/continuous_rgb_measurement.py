#!/usr/bin/env python3

"""
Script para comparación de mediciones RGB con LED encendido y apagado.
Usa el tópico /color para obtener datos consistentes del sensor de color.
Compara el impacto del LED en las mediciones de color.
"""

import rospy
import time
import signal
import sys
from atriz_rvr_msgs.srv import GetRGBCSensorValues
from atriz_rvr_msgs.msg import Color
from std_srvs.srv import SetBool

class ContinuousRGBMeasurement:
    """Clase para comparación de mediciones RGB con LED encendido y apagado."""
    
    def __init__(self):
        rospy.init_node('continuous_rgb_measurement', anonymous=True)
        
        # Configurar manejo de señales
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        # Variables de control
        self.running = True
        self.measurement_count = 0
        self.start_time = time.time()
        self.led_enabled = False
        
        # Almacenar mediciones para comparación
        self.measurements_led_off = []
        self.measurements_led_on = []
        self.color_topic_data = None
        
        # Configurar servicios y subscribers
        self.setup_services()
        self.setup_subscribers()
        
        # Configurar logging
        rospy.loginfo("=" * 70)
        rospy.loginfo("🎨 COMPARACIÓN DE MEDICIONES RGB: LED ENCENDIDO vs APAGADO")
        rospy.loginfo("=" * 70)
        rospy.loginfo("📊 Configuración:")
        rospy.loginfo("   • Tópico: /color")
        rospy.loginfo("   • Modo: Comparación LED ON/OFF")
        rospy.loginfo("   • Frecuencia: 1 Hz por modo")
        rospy.loginfo("   • Formato: Valores RGB (0-255)")
        rospy.loginfo("=" * 70)
        
    def setup_services(self):
        """Configura los servicios ROS necesarios."""
        try:
            # Esperar servicio de medición RGB
            rospy.loginfo("🔍 Esperando servicio /get_rgbc_sensor_values...")
            rospy.wait_for_service('/get_rgbc_sensor_values', timeout=10)
            self.rgbc_service = rospy.ServiceProxy('/get_rgbc_sensor_values', GetRGBCSensorValues)
            rospy.loginfo("✅ Servicio RGB disponible")
            
            # Configurar servicio de control de LED
            rospy.loginfo("🔍 Esperando servicio /enable_color...")
            rospy.wait_for_service('/enable_color', timeout=10)
            self.enable_color_service = rospy.ServiceProxy('/enable_color', SetBool)
            rospy.loginfo("✅ Servicio de control de LED disponible")
                
        except rospy.ROSException as e:
            rospy.logerr(f"❌ Error configurando servicios: {e}")
            sys.exit(1)
    
    def setup_subscribers(self):
        """Configura los subscribers ROS necesarios."""
        try:
            # Subscriber para el tópico /color
            rospy.loginfo("🔍 Configurando subscriber para /color...")
            self.color_sub = rospy.Subscriber('/color', Color, self.color_callback)
            rospy.loginfo("✅ Subscriber configurado")
        except Exception as e:
            rospy.logerr(f"❌ Error configurando subscribers: {e}")
            sys.exit(1)
    
    def color_callback(self, msg):
        """Callback para recibir datos del tópico /color."""
        self.color_topic_data = {
            'red': msg.rgb_color[0],
            'green': msg.rgb_color[1], 
            'blue': msg.rgb_color[2],
            'confidence': msg.confidence,
            'timestamp': time.time(),
            'led_enabled': self.led_enabled
        }
    
    def signal_handler(self, signum, frame):
        """Maneja las señales del sistema para parada segura."""
        rospy.loginfo(f"\n🛑 Señal {signum} recibida - Deteniendo mediciones...")
        self.running = False
        self.print_comparison_summary()
        sys.exit(0)
    
    def set_led_state(self, enabled):
        """Controla el estado del LED del sensor."""
        try:
            response = self.enable_color_service(enabled)
            if response.success:
                self.led_enabled = enabled
                status = "ENCENDIDO" if enabled else "APAGADO"
                rospy.loginfo(f"💡 LED del sensor: {status}")
                return True
            else:
                rospy.logwarn(f"⚠️ Error controlando LED: {response.message}")
                return False
        except Exception as e:
            rospy.logwarn(f"⚠️ Error llamando servicio LED: {e}")
            return False
    
    def measure_rgb(self):
        """Realiza una medición de RGB usando el tópico /color."""
        # Esperar a recibir datos del tópico
        timeout = 5.0  # 5 segundos de timeout
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            if self.color_topic_data is not None:
                # Crear copia de los datos
                measurement = self.color_topic_data.copy()
                
                # Almacenar según el estado del LED
                if self.led_enabled:
                    self.measurements_led_on.append(measurement)
                else:
                    self.measurements_led_off.append(measurement)
                
                return measurement
            
            rospy.sleep(0.1)  # Esperar 100ms
        
        rospy.logwarn("⚠️ Timeout esperando datos del tópico /color")
        return None
    
    def calculate_percentages(self, rgb_data):
        """Calcula porcentajes de los valores RGB."""
        if not rgb_data:
            return None
            
        max_value = 65535  # uint16_t máximo
        
        return {
            'red_pct': (rgb_data['red'] / max_value) * 100,
            'green_pct': (rgb_data['green'] / max_value) * 100,
            'blue_pct': (rgb_data['blue'] / max_value) * 100,
            'clear_pct': (rgb_data['clear'] / max_value) * 100
        }
    
    def calculate_normalized_rgb(self, rgb_data):
        """Calcula valores RGB normalizados (0-255)."""
        if not rgb_data:
            return None
            
        max_value = 65535
        
        return {
            'red_norm': int((rgb_data['red'] * 255) // max_value),
            'green_norm': int((rgb_data['green'] * 255) // max_value),
            'blue_norm': int((rgb_data['blue'] * 255) // max_value)
        }
    
    def process_like_color_topic(self, rgb_data):
        """
        Procesa los valores RGB de la misma manera que el tópico /color.
        
        El tópico /color usa directamente los valores de ColorDetection['R'], ['G'], ['B']
        sin conversión adicional, que son valores 8-bit (0-255).
        """
        if not rgb_data:
            return None
            
        # Simular el procesamiento del tópico /color
        # Los valores vienen directamente de color_data['ColorDetection']['R/G/B']
        # que son valores 8-bit (0-255) según el driver
        
        # Para simular esto, usamos los valores raw pero los limitamos a 8-bit
        # ya que el tópico /color espera valores 0-255
        return {
            'red_topic': min(rgb_data['red'], 255),
            'green_topic': min(rgb_data['green'], 255), 
            'blue_topic': min(rgb_data['blue'], 255),
            'confidence': 0.0  # El tópico /color siempre muestra confianza 0.0
        }
    
    def analyze_color(self, rgb_data):
        """Analiza el color detectado."""
        if not rgb_data:
            return "Desconocido"
            
        # Encontrar color dominante
        colors = {
            'Red': rgb_data['red'],
            'Green': rgb_data['green'],
            'Blue': rgb_data['blue']
        }
        
        dominant_color = max(colors, key=colors.get)
        dominant_value = colors[dominant_color]
        
        # Calcular luminancia total
        total_luminance = rgb_data['clear']
        luminance_pct = (total_luminance / 65535) * 100
        
        # Determinar tipo de ambiente
        if luminance_pct < 10:
            env_type = "Muy oscuro"
        elif luminance_pct < 30:
            env_type = "Oscuro"
        elif luminance_pct < 60:
            env_type = "Normal"
        elif luminance_pct < 85:
            env_type = "Brillante"
        else:
            env_type = "Muy brillante"
        
        return f"{dominant_color} dominante ({dominant_value}), {env_type} ({luminance_pct:.1f}%)"
    
    def analyze_color_topic(self, rgb_data):
        """Analiza el color detectado desde el tópico /color."""
        if not rgb_data:
            return "Desconocido"
            
        # Encontrar color dominante
        colors = {
            'Red': rgb_data['red'],
            'Green': rgb_data['green'],
            'Blue': rgb_data['blue']
        }
        
        dominant_color = max(colors, key=colors.get)
        dominant_value = colors[dominant_color]
        
        # Determinar nivel de confianza
        confidence = rgb_data['confidence']
        if confidence < 0.1:
            conf_level = "Baja"
        elif confidence < 0.5:
            conf_level = "Media"
        else:
            conf_level = "Alta"
        
        return f"{dominant_color} (Red={rgb_data['red']}), Confianza: {conf_level}"
    
    def calculate_averages(self, measurements):
        """Calcula promedios de una lista de mediciones."""
        if not measurements:
            return None
            
        avg_red = sum(m['red'] for m in measurements) / len(measurements)
        avg_green = sum(m['green'] for m in measurements) / len(measurements)
        avg_blue = sum(m['blue'] for m in measurements) / len(measurements)
        avg_clear = sum(m['clear'] for m in measurements) / len(measurements)
        
        return {
            'red': avg_red,
            'green': avg_green,
            'blue': avg_blue,
            'clear': avg_clear,
            'count': len(measurements)
        }
    
    def calculate_differences(self, avg_led_off, avg_led_on):
        """Calcula las diferencias entre mediciones con LED apagado y encendido."""
        if not avg_led_off or not avg_led_on:
            return None
            
        return {
            'red_diff': avg_led_on['red'] - avg_led_off['red'],
            'green_diff': avg_led_on['green'] - avg_led_off['green'],
            'blue_diff': avg_led_on['blue'] - avg_led_off['blue'],
            'clear_diff': avg_led_on['clear'] - avg_led_off['clear'],
            'red_pct_diff': ((avg_led_on['red'] - avg_led_off['red']) / avg_led_off['red']) * 100 if avg_led_off['red'] > 0 else 0,
            'green_pct_diff': ((avg_led_on['green'] - avg_led_off['green']) / avg_led_off['green']) * 100 if avg_led_off['green'] > 0 else 0,
            'blue_pct_diff': ((avg_led_on['blue'] - avg_led_off['blue']) / avg_led_off['blue']) * 100 if avg_led_off['blue'] > 0 else 0,
            'clear_pct_diff': ((avg_led_on['clear'] - avg_led_off['clear']) / avg_led_off['clear']) * 100 if avg_led_off['clear'] > 0 else 0
        }
    
    def print_measurement(self, rgb_data, analysis):
        """Imprime una medición formateada."""
        if not rgb_data:
            return
            
        self.measurement_count += 1
        elapsed_time = time.time() - self.start_time
        led_status = "💡 LED ON" if rgb_data['led_enabled'] else "🔌 LED OFF"
        
        print(f"\n📊 Medición #{self.measurement_count} (T+{elapsed_time:.1f}s) - {led_status}")
        print(f"   RGB:        R={rgb_data['red']:3d}, G={rgb_data['green']:3d}, B={rgb_data['blue']:3d}")
        print(f"   Confianza:  {rgb_data['confidence']:.2f}")
        print(f"   Análisis:   {analysis}")
    
    def print_comparison_summary(self):
        """Imprime resumen de comparación entre LED encendido y apagado."""
        elapsed_time = time.time() - self.start_time
        
        print(f"\n📈 RESUMEN DE COMPARACIÓN LED ON vs OFF")
        print(f"   • Tiempo total: {elapsed_time:.1f} segundos")
        print(f"   • Mediciones LED OFF: {len(self.measurements_led_off)}")
        print(f"   • Mediciones LED ON: {len(self.measurements_led_on)}")
        
        if len(self.measurements_led_off) > 0 and len(self.measurements_led_on) > 0:
            # Calcular promedios
            avg_led_off = self.calculate_averages(self.measurements_led_off)
            avg_led_on = self.calculate_averages(self.measurements_led_on)
            differences = self.calculate_differences(avg_led_off, avg_led_on)
            
            print(f"\n📊 PROMEDIOS:")
            print(f"   LED OFF: R={avg_led_off['red']:6.0f}, G={avg_led_off['green']:6.0f}, B={avg_led_off['blue']:6.0f}, Clear={avg_led_off['clear']:6.0f}")
            print(f"   LED ON:  R={avg_led_on['red']:6.0f}, G={avg_led_on['green']:6.0f}, B={avg_led_on['blue']:6.0f}, Clear={avg_led_on['clear']:6.0f}")
            
            print(f"\n🔍 DIFERENCIAS (LED ON - LED OFF):")
            print(f"   Absolutas: R={differences['red_diff']:+6.0f}, G={differences['green_diff']:+6.0f}, B={differences['blue_diff']:+6.0f}, Clear={differences['clear_diff']:+6.0f}")
            print(f"   Porcentuales: R={differences['red_pct_diff']:+6.1f}%, G={differences['green_pct_diff']:+6.1f}%, B={differences['blue_pct_diff']:+6.1f}%, Clear={differences['clear_pct_diff']:+6.1f}%")
            
            # Análisis del impacto del LED
            print(f"\n💡 IMPACTO DEL LED:")
            if abs(differences['clear_pct_diff']) < 5:
                print(f"   • Impacto MÍNIMO: El LED apenas afecta la medición ({differences['clear_pct_diff']:+.1f}%)")
            elif abs(differences['clear_pct_diff']) < 20:
                print(f"   • Impacto MODERADO: El LED afecta moderadamente la medición ({differences['clear_pct_diff']:+.1f}%)")
            else:
                print(f"   • Impacto ALTO: El LED afecta significativamente la medición ({differences['clear_pct_diff']:+.1f}%)")
                
            # Recomendación
            if abs(differences['clear_pct_diff']) < 10:
                print(f"   • ✅ RECOMENDACIÓN: Usar mediciones sin LED para detección de obstáculos")
            else:
                print(f"   • ⚠️ RECOMENDACIÓN: Considerar el impacto del LED en las mediciones")
        else:
            print(f"   ⚠️ No hay suficientes datos para comparación")
    
    def print_summary(self):
        """Imprime resumen de las mediciones."""
        elapsed_time = time.time() - self.start_time
        avg_freq = self.measurement_count / elapsed_time if elapsed_time > 0 else 0
        
        print(f"\n📈 RESUMEN DE MEDICIONES")
        print(f"   • Total de mediciones: {self.measurement_count}")
        print(f"   • Tiempo transcurrido: {elapsed_time:.1f} segundos")
        print(f"   • Frecuencia promedio: {avg_freq:.2f} Hz")
        print(f"   • LED del sensor: APAGADO durante toda la medición")
    
    def run_comparison_measurement(self, measurements_per_mode=5, frequency=1.0):
        """Ejecuta medición comparativa entre LED encendido y apagado."""
        rospy.loginfo(f"🚀 Iniciando medición comparativa...")
        rospy.loginfo(f"📊 {measurements_per_mode} mediciones por modo a {frequency} Hz")
        rospy.loginfo("🛑 Presiona Ctrl+C para detener")
        
        rate = rospy.Rate(frequency)
        
        # Fase 1: Mediciones con LED APAGADO
        rospy.loginfo(f"\n🔌 FASE 1: Mediciones con LED APAGADO ({measurements_per_mode} mediciones)")
        self.set_led_state(False)
        rospy.sleep(1)  # Esperar estabilización
        
        for i in range(measurements_per_mode):
            if rospy.is_shutdown() or not self.running:
                break
                
            rgb_data = self.measure_rgb()
            if rgb_data:
                analysis = self.analyze_color_topic(rgb_data)
                self.print_measurement(rgb_data, analysis)
            else:
                rospy.logwarn("⚠️ No se pudo obtener medición")
            
            rate.sleep()
        
        # Fase 2: Mediciones con LED ENCENDIDO
        rospy.loginfo(f"\n💡 FASE 2: Mediciones con LED ENCENDIDO ({measurements_per_mode} mediciones)")
        self.set_led_state(True)
        rospy.sleep(1)  # Esperar estabilización
        
        for i in range(measurements_per_mode):
            if rospy.is_shutdown() or not self.running:
                break
                
            rgb_data = self.measure_rgb()
            if rgb_data:
                analysis = self.analyze_color_topic(rgb_data)
                self.print_measurement(rgb_data, analysis)
            else:
                rospy.logwarn("⚠️ No se pudo obtener medición")
            
            rate.sleep()
        
        # Apagar LED al finalizar
        self.set_led_state(False)
        rospy.loginfo("🏁 Medición comparativa completada")

def main():
    """Función principal."""
    try:
        # Crear instancia del medidor
        measurer = ContinuousRGBMeasurement()
        
        # Ejecutar medición comparativa
        measurer.run_comparison_measurement(measurements_per_mode=5, frequency=1.0)
        
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 Interrumpido por el usuario")
    except Exception as e:
        rospy.logerr(f"❌ Error inesperado: {e}")
    finally:
        rospy.loginfo("🏁 Script finalizado")

if __name__ == '__main__':
    main()
