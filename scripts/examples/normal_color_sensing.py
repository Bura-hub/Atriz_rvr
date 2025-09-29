#!/usr/bin/env python3

"""
Script para probar el sensado normal de colores del Sphero RVR.
Usa el tópico /color que es el sistema estándar de detección de colores.
"""

import rospy
import time
import signal
import sys
from atriz_rvr_msgs.msg import Color
from std_srvs.srv import SetBool

class NormalColorSensing:
    """Clase para probar el sensado normal de colores."""
    
    def __init__(self):
        rospy.init_node('normal_color_sensing', anonymous=True)
        
        # Configurar manejo de señales
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        # Variables de control
        self.running = True
        self.measurement_count = 0
        self.start_time = time.time()
        self.color_enabled = False
        
        # Almacenar mediciones
        self.measurements = []
        
        # Configurar servicios y subscribers
        self.setup_ros_interface()
        
        # Configurar logging
        rospy.loginfo("=" * 60)
        rospy.loginfo("🎨 SENSADO NORMAL DE COLORES - SPHERO RVR")
        rospy.loginfo("=" * 60)
        rospy.loginfo("📊 Configuración:")
        rospy.loginfo("   • Tópico: /color")
        rospy.loginfo("   • Servicio: /enable_color")
        rospy.loginfo("   • Frecuencia: Variable (depende del sensor)")
        rospy.loginfo("   • Formato: Valores RGB normalizados (0-255)")
        rospy.loginfo("=" * 60)
        
    def setup_ros_interface(self):
        """Configura la interfaz ROS."""
        try:
            # Esperar servicio de control de color
            rospy.loginfo("🔍 Esperando servicio /enable_color...")
            rospy.wait_for_service('/enable_color', timeout=10)
            self.enable_color_service = rospy.ServiceProxy('/enable_color', SetBool)
            rospy.loginfo("✅ Servicio de control de color disponible")
            
            # Configurar subscriber para datos de color
            rospy.loginfo("🔍 Configurando subscriber para /color...")
            self.color_sub = rospy.Subscriber('/color', Color, self.color_callback)
            rospy.loginfo("✅ Subscriber configurado")
            
        except rospy.ROSException as e:
            rospy.logerr(f"❌ Error configurando interfaz ROS: {e}")
            sys.exit(1)
    
    def signal_handler(self, signum, frame):
        """Maneja las señales del sistema para parada segura."""
        rospy.loginfo(f"\n🛑 Señal {signum} recibida - Deteniendo sensado...")
        self.running = False
        self.print_summary()
        sys.exit(0)
    
    def enable_color_sensing(self):
        """Habilita el sensado de colores."""
        try:
            response = self.enable_color_service(True)
            if response.success:
                self.color_enabled = True
                rospy.loginfo("✅ Sensado de colores habilitado")
                return True
            else:
                rospy.logerr(f"❌ Error habilitando sensado: {response.message}")
                return False
        except Exception as e:
            rospy.logerr(f"❌ Error llamando servicio: {e}")
            return False
    
    def disable_color_sensing(self):
        """Deshabilita el sensado de colores."""
        try:
            response = self.enable_color_service(False)
            if response.success:
                self.color_enabled = False
                rospy.loginfo("✅ Sensado de colores deshabilitado")
                return True
            else:
                rospy.logwarn(f"⚠️ Error deshabilitando sensado: {response.message}")
                return False
        except Exception as e:
            rospy.logwarn(f"⚠️ Error llamando servicio: {e}")
            return False
    
    def color_callback(self, data):
        """Callback para datos de color recibidos."""
        if not self.running:
            return
            
        self.measurement_count += 1
        elapsed_time = time.time() - self.start_time
        
        # Extraer datos RGB
        rgb_values = data.rgb_color
        confidence = data.confidence
        
        # Crear medición
        measurement = {
            'red': rgb_values[0],
            'green': rgb_values[1], 
            'blue': rgb_values[2],
            'confidence': confidence,
            'timestamp': time.time(),
            'elapsed_time': elapsed_time
        }
        
        # Almacenar medición
        self.measurements.append(measurement)
        
        # Mostrar medición
        self.print_color_measurement(measurement)
    
    def print_color_measurement(self, measurement):
        """Imprime una medición de color formateada."""
        print(f"\n🎨 Medición #{self.measurement_count} (T+{measurement['elapsed_time']:.1f}s)")
        print(f"   RGB:        R={measurement['red']:3d}, G={measurement['green']:3d}, B={measurement['blue']:3d}")
        print(f"   Confianza:  {measurement['confidence']:.2f}")
        
        # Análisis del color
        analysis = self.analyze_color(measurement)
        print(f"   Análisis:   {analysis}")
    
    def analyze_color(self, measurement):
        """Analiza el color detectado."""
        red = measurement['red']
        green = measurement['green']
        blue = measurement['blue']
        confidence = measurement['confidence']
        
        # Encontrar color dominante
        colors = {'Red': red, 'Green': green, 'Blue': blue}
        dominant_color = max(colors, key=colors.get)
        dominant_value = colors[dominant_color]
        
        # Determinar tipo de color
        if red > 200 and green > 200 and blue > 200:
            color_type = "Blanco"
        elif red < 50 and green < 50 and blue < 50:
            color_type = "Negro/Oscuro"
        elif red > green and red > blue:
            color_type = "Rojo"
        elif green > red and green > blue:
            color_type = "Verde"
        elif blue > red and blue > green:
            color_type = "Azul"
        elif red > 200 and green > 200 and blue < 100:
            color_type = "Amarillo"
        elif red > 200 and green < 100 and blue > 200:
            color_type = "Magenta"
        elif red < 100 and green > 200 and blue > 200:
            color_type = "Cian"
        else:
            color_type = "Mixto"
        
        # Evaluar confianza
        if confidence > 0.8:
            conf_level = "Alta"
        elif confidence > 0.5:
            conf_level = "Media"
        else:
            conf_level = "Baja"
        
        return f"{color_type} ({dominant_color}={dominant_value}), Confianza: {conf_level}"
    
    def calculate_statistics(self):
        """Calcula estadísticas de las mediciones."""
        if not self.measurements:
            return None
            
        # Promedios
        avg_red = sum(m['red'] for m in self.measurements) / len(self.measurements)
        avg_green = sum(m['green'] for m in self.measurements) / len(self.measurements)
        avg_blue = sum(m['blue'] for m in self.measurements) / len(self.measurements)
        avg_confidence = sum(m['confidence'] for m in self.measurements) / len(self.measurements)
        
        # Valores máximos y mínimos
        max_red = max(m['red'] for m in self.measurements)
        min_red = min(m['red'] for m in self.measurements)
        max_green = max(m['green'] for m in self.measurements)
        min_green = min(m['green'] for m in self.measurements)
        max_blue = max(m['blue'] for m in self.measurements)
        min_blue = min(m['blue'] for m in self.measurements)
        
        return {
            'count': len(self.measurements),
            'avg_red': avg_red,
            'avg_green': avg_green,
            'avg_blue': avg_blue,
            'avg_confidence': avg_confidence,
            'max_red': max_red,
            'min_red': min_red,
            'max_green': max_green,
            'min_green': min_green,
            'max_blue': max_blue,
            'min_blue': min_blue
        }
    
    def print_summary(self):
        """Imprime resumen de las mediciones."""
        elapsed_time = time.time() - self.start_time
        stats = self.calculate_statistics()
        
        print(f"\n📈 RESUMEN DEL SENSADO DE COLORES")
        print(f"   • Tiempo total: {elapsed_time:.1f} segundos")
        print(f"   • Total de mediciones: {self.measurement_count}")
        print(f"   • Frecuencia promedio: {self.measurement_count/elapsed_time:.2f} Hz")
        
        if stats:
            print(f"\n📊 ESTADÍSTICAS:")
            print(f"   Promedios:    R={stats['avg_red']:6.1f}, G={stats['avg_green']:6.1f}, B={stats['avg_blue']:6.1f}")
            print(f"   Rango R:      {stats['min_red']:3d} - {stats['max_red']:3d}")
            print(f"   Rango G:      {stats['min_green']:3d} - {stats['max_green']:3d}")
            print(f"   Rango B:      {stats['min_blue']:3d} - {stats['max_blue']:3d}")
            print(f"   Confianza:    {stats['avg_confidence']:.2f}")
            
            # Análisis de estabilidad
            red_range = stats['max_red'] - stats['min_red']
            green_range = stats['max_green'] - stats['min_green']
            blue_range = stats['max_blue'] - stats['min_blue']
            
            print(f"\n🔍 ANÁLISIS DE ESTABILIDAD:")
            print(f"   Variación R:  {red_range:3d} ({'Estable' if red_range < 50 else 'Variable'})")
            print(f"   Variación G:  {green_range:3d} ({'Estable' if green_range < 50 else 'Variable'})")
            print(f"   Variación B:  {blue_range:3d} ({'Estable' if blue_range < 50 else 'Variable'})")
    
    def run_color_sensing(self, duration=30):
        """Ejecuta el sensado de colores por un tiempo determinado."""
        rospy.loginfo(f"🚀 Iniciando sensado de colores por {duration} segundos...")
        
        # Habilitar sensado de colores
        if not self.enable_color_sensing():
            rospy.logerr("❌ No se pudo habilitar el sensado de colores")
            return
        
        rospy.loginfo("🎨 Sensado de colores activo - Coloca objetos de diferentes colores cerca del sensor")
        rospy.loginfo("🛑 Presiona Ctrl+C para detener")
        
        # Esperar por el tiempo especificado
        start_time = time.time()
        while not rospy.is_shutdown() and self.running:
            if time.time() - start_time >= duration:
                break
            rospy.sleep(0.1)
        
        # Deshabilitar sensado de colores
        self.disable_color_sensing()
        rospy.loginfo("🏁 Sensado de colores completado")

def main():
    """Función principal."""
    try:
        # Crear instancia del sensor
        sensor = NormalColorSensing()
        
        # Ejecutar sensado de colores
        sensor.run_color_sensing(duration=30)  # 30 segundos
        
        # Mostrar resumen
        sensor.print_summary()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 Interrumpido por el usuario")
    except Exception as e:
        rospy.logerr(f"❌ Error inesperado: {e}")
    finally:
        rospy.loginfo("🏁 Script finalizado")

if __name__ == '__main__':
    main()
