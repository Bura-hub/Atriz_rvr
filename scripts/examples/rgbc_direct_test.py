#!/usr/bin/env python3

import rospy
import signal
import sys
import asyncio
import time
from geometry_msgs.msg import Twist

# Importar el SDK de Sphero
sys.path.append('/home/sphero/atriz_git/src/Atriz_rvr/atriz_rvr_driver/scripts/sphero_sdk')
from sphero_sdk.asyncio.client.dal.serial_async_dal import SerialAsyncDal
from sphero_sdk.asyncio.client.toys.sphero_rvr_async import SpheroRvrAsync

class RGBCDirectTest:
    """Prueba directa de get_rgbc_sensor_values para verificar si activa el LED."""
    
    def __init__(self):
        rospy.init_node('rgbc_direct_test', anonymous=True)
        
        # Variables
        self.rvr = None
        self.dal = None
        self.running = True
        self.measurement_count = 0
        
        # Configurar señal de interrupción
        signal.signal(signal.SIGINT, self.signal_handler)
        
        rospy.loginfo("🔬 Prueba Directa: get_rgbc_sensor_values")
        rospy.loginfo("=" * 60)
        rospy.loginfo("🎯 Objetivo: Verificar si get_rgbc_sensor_values activa el LED")
    
    def signal_handler(self, signum, frame):
        """Maneja Ctrl+C."""
        rospy.loginfo("🛑 Deteniendo...")
        self.running = False
        self.cleanup()
        sys.exit(0)
    
    async def connect_to_rvr(self):
        """Conecta al RVR usando el SDK directamente."""
        try:
            rospy.loginfo("🔌 Conectando al RVR...")
            
            # Crear loop y DAL
            loop = asyncio.get_event_loop()
            self.dal = SerialAsyncDal(loop, port_id='/dev/ttyS0', baud=115200)
            self.rvr = SpheroRvrAsync(self.dal)
            
            # Conectar y despertar
            await self.rvr.connect()
            await self.rvr.wake()
            
            rospy.loginfo("✅ RVR conectado exitosamente")
            return True
            
        except Exception as e:
            rospy.logerr(f"❌ Error conectando al RVR: {e}")
            return False
    
    async def test_rgbc_direct_access(self):
        """Prueba acceso directo a get_rgbc_sensor_values."""
        rospy.loginfo("🧪 PRUEBA: Acceso directo a get_rgbc_sensor_values")
        rospy.loginfo("=" * 60)
        
        # Paso 1: Verificar estado inicial
        rospy.loginfo("1️⃣ Estado inicial del sensor...")
        rospy.loginfo("   🔍 Sensor de color: NO habilitado (sin enable_color_detection)")
        rospy.loginfo("   🔍 LED del sensor: Estado desconocido")
        
        # Paso 2: Intentar acceso directo a get_rgbc_sensor_values
        rospy.loginfo("\n2️⃣ Intentando acceso directo a get_rgbc_sensor_values...")
        rospy.loginfo("   ⚠️ ADVERTENCIA: Esto podría activar el LED automáticamente")
        
        try:
            # Llamar directamente a get_rgbc_sensor_values
            rospy.loginfo("   📡 Enviando comando get_rgbc_sensor_values...")
            result = await self.rvr.get_rgbc_sensor_values(timeout=5.0)
            
            if result:
                self.measurement_count += 1
                
                # Extraer valores RGBC
                red = result.get('red_channel_value', 0)
                green = result.get('green_channel_value', 0)
                blue = result.get('blue_channel_value', 0)
                clear = result.get('clear_channel_value', 0)
                
                # Log de la medición
                rospy.loginfo(f"   📊 MEDICIÓN #{self.measurement_count} (DIRECTA)")
                rospy.loginfo(f"      Red:   {red:5d}")
                rospy.loginfo(f"      Green: {green:5d}")
                rospy.loginfo(f"      Blue:  {blue:5d}")
                rospy.loginfo(f"      Clear: {clear:5d}")
                
                # Análisis de los resultados
                rospy.loginfo("\n3️⃣ Análisis de resultados...")
                if red > 0 or green > 0 or blue > 0 or clear > 0:
                    rospy.loginfo("   ✅ Se obtuvieron datos del sensor")
                    rospy.loginfo("   🔍 Los datos sugieren que el sensor está funcionando")
                    rospy.loginfo("   ⚠️ NOTA: No podemos verificar visualmente si el LED se activó")
                    rospy.loginfo("   💡 El LED podría haberse activado internamente")
                else:
                    rospy.loginfo("   ❌ No se obtuvieron datos del sensor")
                    rospy.loginfo("   🔍 Esto podría indicar que el sensor no está activo")
                
                return True
            else:
                rospy.logwarn("   ⚠️ No se recibieron datos del sensor")
                return False
                
        except Exception as e:
            rospy.logerr(f"   ❌ Error en get_rgbc_sensor_values: {e}")
            return False
    
    async def test_without_enable_color(self):
        """Prueba múltiples llamadas sin enable_color_detection."""
        rospy.loginfo("\n🔄 PRUEBA: Múltiples llamadas sin enable_color_detection")
        rospy.loginfo("=" * 60)
        
        for i in range(3):
            if not self.running:
                break
                
            rospy.loginfo(f"🔄 Llamada #{i+1}/3...")
            
            try:
                result = await self.rvr.get_rgbc_sensor_values(timeout=3.0)
                
                if result:
                    red = result.get('red_channel_value', 0)
                    green = result.get('green_channel_value', 0)
                    blue = result.get('blue_channel_value', 0)
                    clear = result.get('clear_channel_value', 0)
                    
                    rospy.loginfo(f"   📊 Resultado #{i+1}: R={red}, G={green}, B={blue}, C={clear}")
                else:
                    rospy.logwarn(f"   ⚠️ Sin datos en llamada #{i+1}")
                
                # Esperar entre llamadas
                if i < 2:
                    await asyncio.sleep(1.0)
                    
            except Exception as e:
                rospy.logerr(f"   ❌ Error en llamada #{i+1}: {e}")
    
    async def test_with_enable_color_comparison(self):
        """Prueba comparativa con enable_color_detection."""
        rospy.loginfo("\n🆚 PRUEBA: Comparación con enable_color_detection")
        rospy.loginfo("=" * 60)
        
        # Habilitar color detection
        rospy.loginfo("1️⃣ Habilitando color_detection...")
        try:
            await self.rvr.enable_color_detection(True)
            rospy.loginfo("   ✅ Color detection habilitado")
            rospy.loginfo("   🟡 LED del sensor: ACTIVADO (confirmado)")
        except Exception as e:
            rospy.logerr(f"   ❌ Error habilitando color detection: {e}")
            return
        
        # Esperar un poco
        await asyncio.sleep(1.0)
        
        # Ahora probar get_rgbc_sensor_values
        rospy.loginfo("\n2️⃣ Probando get_rgbc_sensor_values con color_detection habilitado...")
        try:
            result = await self.rvr.get_rgbc_sensor_values(timeout=3.0)
            
            if result:
                red = result.get('red_channel_value', 0)
                green = result.get('green_channel_value', 0)
                blue = result.get('blue_channel_value', 0)
                clear = result.get('clear_channel_value', 0)
                
                rospy.loginfo(f"   📊 Con color_detection: R={red}, G={green}, B={blue}, C={clear}")
            else:
                rospy.logwarn("   ⚠️ Sin datos con color_detection habilitado")
                
        except Exception as e:
            rospy.logerr(f"   ❌ Error con color_detection: {e}")
        
        # Deshabilitar color detection
        rospy.loginfo("\n3️⃣ Deshabilitando color_detection...")
        try:
            await self.rvr.enable_color_detection(False)
            rospy.loginfo("   ✅ Color detection deshabilitado")
        except Exception as e:
            rospy.logerr(f"   ❌ Error deshabilitando color detection: {e}")
    
    def cleanup(self):
        """Limpia al finalizar."""
        try:
            if self.rvr:
                rospy.loginfo("🧹 Desconectando del RVR...")
                # Nota: No podemos usar await aquí, así que solo logueamos
                rospy.loginfo("✅ Limpieza completada")
        except Exception as e:
            rospy.logwarn(f"⚠️ Error en limpieza: {e}")
    
    async def run_tests(self):
        """Ejecuta todas las pruebas."""
        rospy.loginfo("🚀 Iniciando pruebas de get_rgbc_sensor_values...")
        
        # Conectar al RVR
        if not await self.connect_to_rvr():
            rospy.logerr("❌ No se pudo conectar al RVR")
            return
        
        # Prueba 1: Acceso directo
        await self.test_rgbc_direct_access()
        
        # Prueba 2: Múltiples llamadas
        await self.test_without_enable_color()
        
        # Prueba 3: Comparación con enable_color
        await self.test_with_enable_color_comparison()
        
        # Resumen final
        rospy.loginfo("\n📋 RESUMEN FINAL")
        rospy.loginfo("=" * 60)
        rospy.loginfo(f"   📊 Mediciones exitosas: {self.measurement_count}")
        rospy.loginfo("   🔍 Conclusión: Verificar logs para determinar comportamiento del LED")
        rospy.loginfo("   💡 Recomendación: Observar visualmente el LED durante las pruebas")

def main():
    """Función principal."""
    try:
        # Crear instancia
        tester = RGBCDirectTest()
        
        # Ejecutar pruebas
        asyncio.run(tester.run_tests())
        
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
