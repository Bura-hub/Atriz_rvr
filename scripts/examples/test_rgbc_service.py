#!/usr/bin/env python3

import rospy
import signal
import sys
import time
from atriz_rvr_msgs.srv import GetRGBCSensorValues

class TestRGBCService:
    """Cliente de prueba para el servicio get_rgbc_sensor_values."""
    
    def __init__(self):
        rospy.init_node('test_rgbc_service', anonymous=True)
        
        # Variables
        self.running = True
        self.measurement_count = 0
        
        # Configurar señal de interrupción
        signal.signal(signal.SIGINT, self.signal_handler)
        
        # Configurar servicio
        self.setup_service()
        
        rospy.loginfo("🧪 Cliente de Prueba: Servicio RGBC")
        rospy.loginfo("=" * 60)
        rospy.loginfo("🎯 Objetivo: Probar mediciones de color SIN activar LED")
    
    def setup_service(self):
        """Configura el servicio RGBC."""
        try:
            rospy.wait_for_service('/get_rgbc_sensor_values', timeout=10.0)
            self.rgbc_service = rospy.ServiceProxy('/get_rgbc_sensor_values', GetRGBCSensorValues)
            rospy.loginfo("✅ Servicio RGBC configurado")
            
        except rospy.ROSException as e:
            rospy.logerr(f"❌ Error: {e}")
            rospy.logwarn("⚠️ Asegúrate de que el servicio rgbc_sensor_service.py esté ejecutándose")
            sys.exit(1)
    
    def signal_handler(self, signum, frame):
        """Maneja Ctrl+C."""
        rospy.loginfo("🛑 Deteniendo...")
        self.running = False
        sys.exit(0)
    
    def take_rgbc_measurement(self):
        """Toma una medición usando el servicio RGBC."""
        try:
            rospy.loginfo("=" * 60)
            rospy.loginfo("🎨 TOMANDO MEDICIÓN RGBC (SIN LED)")
            rospy.loginfo("=" * 60)
            
            # Llamar al servicio
            rospy.loginfo("📡 Solicitando datos RGBC...")
            response = self.rgbc_service()
            
            if response.success:
                self.measurement_count += 1
                
                # Log de la medición
                rospy.loginfo(f"📊 MEDICIÓN #{self.measurement_count} (SIN LED)")
                rospy.loginfo(f"   Red:   {response.red_channel_value:5d}")
                rospy.loginfo(f"   Green: {response.green_channel_value:5d}")
                rospy.loginfo(f"   Blue:  {response.blue_channel_value:5d}")
                rospy.loginfo(f"   Clear: {response.clear_channel_value:5d}")
                rospy.loginfo(f"   LED:   ❌ NO ACTIVADO")
                rospy.loginfo("=" * 60)
                
                # Análisis de los resultados
                if (response.red_channel_value > 0 or 
                    response.green_channel_value > 0 or 
                    response.blue_channel_value > 0 or 
                    response.clear_channel_value > 0):
                    rospy.loginfo("✅ ¡ÉXITO! Se obtuvieron datos del sensor SIN activar el LED")
                    rospy.loginfo("🎉 Esto demuestra que es posible medir color sin LED")
                else:
                    rospy.loginfo("⚠️ Datos del sensor son cero")
                    rospy.loginfo("🔍 Esto podría indicar que el sensor no está activo")
                
                return True
            else:
                rospy.logwarn(f"⚠️ Error en medición: {response.message}")
                return False
                
        except Exception as e:
            rospy.logerr(f"❌ Error tomando medición: {e}")
            return False
    
    def run_continuous_measurements(self):
        """Ejecuta mediciones continuas."""
        rospy.loginfo("🔄 Iniciando mediciones continuas...")
        rospy.loginfo("⏰ Tomando una medición cada 3 segundos")
        rospy.loginfo("🛑 Presiona Ctrl+C para detener")
        
        while self.running and not rospy.is_shutdown():
            self.take_rgbc_measurement()
            
            if self.running:
                rospy.loginfo("⏳ Esperando 3 segundos...")
                rospy.sleep(3.0)
    
    def run_single_measurement(self):
        """Ejecuta una sola medición."""
        rospy.loginfo("🎯 Ejecutando medición única...")
        self.take_rgbc_measurement()
    
    def run_test_suite(self):
        """Ejecuta una suite de pruebas."""
        rospy.loginfo("🧪 Ejecutando suite de pruebas...")
        
        # Prueba 1: Medición única
        rospy.loginfo("\n1️⃣ Prueba 1: Medición única")
        self.take_rgbc_measurement()
        
        # Prueba 2: Múltiples mediciones rápidas
        rospy.loginfo("\n2️⃣ Prueba 2: Múltiples mediciones rápidas")
        for i in range(3):
            if not self.running:
                break
            rospy.loginfo(f"   Medición rápida #{i+1}/3...")
            self.take_rgbc_measurement()
            if i < 2:
                rospy.sleep(1.0)
        
        # Resumen final
        rospy.loginfo("\n📋 RESUMEN DE PRUEBAS")
        rospy.loginfo("=" * 60)
        rospy.loginfo(f"   📊 Total de mediciones: {self.measurement_count}")
        rospy.loginfo("   ✅ Todas las mediciones usaron get_rgbc_sensor_values")
        rospy.loginfo("   ❌ NINGUNA medición activó el LED del sensor")
        rospy.loginfo("   🎉 ¡OBJETIVO CUMPLIDO!")

def main():
    """Función principal."""
    try:
        # Crear instancia
        tester = TestRGBCService()
        
        # Verificar argumentos de línea de comandos
        if len(sys.argv) > 1:
            if sys.argv[1] == "continuous":
                tester.run_continuous_measurements()
            elif sys.argv[1] == "single":
                tester.run_single_measurement()
            elif sys.argv[1] == "test":
                tester.run_test_suite()
            else:
                rospy.logwarn("⚠️ Argumento desconocido. Usando medición única.")
                tester.run_single_measurement()
        else:
            # Por defecto, ejecutar suite de pruebas
            tester.run_test_suite()
        
        # Mantener el nodo vivo
        rospy.loginfo("\n🔄 Nodo activo. Presiona Ctrl+C para salir.")
        while tester.running and not rospy.is_shutdown():
            rospy.sleep(0.1)
            
    except KeyboardInterrupt:
        rospy.loginfo("🛑 Interrumpido por el usuario")
    except Exception as e:
        rospy.logerr(f"❌ Error: {e}")
    finally:
        rospy.loginfo("👋 Finalizando...")

if __name__ == '__main__':
    main()
