#!/usr/bin/env python3

import rospy
import asyncio
import signal
import sys
from atriz_rvr_msgs.srv import GetRGBCSensorValues, GetRGBCSensorValuesResponse

# Importar el SDK de Sphero
sys.path.append('/home/sphero/atriz_git/src/ros_sphero_rvr/atriz_rvr_driver/scripts/sphero_sdk')
from sphero_sdk.asyncio.client.dal.serial_async_dal import SerialAsyncDal
from sphero_sdk.asyncio.client.toys.sphero_rvr_async import SpheroRvrAsync

class RGBCSensorService:
    """Servicio ROS para acceder a get_rgbc_sensor_values sin activar el LED."""
    
    def __init__(self):
        rospy.init_node('rgbc_sensor_service', anonymous=True)
        
        # Variables
        self.rvr = None
        self.dal = None
        self.connected = False
        self.running = True
        
        # Configurar señal de interrupción
        signal.signal(signal.SIGINT, self.signal_handler)
        
        # Crear servicio
        self.service = rospy.Service('/get_rgbc_sensor_values', GetRGBCSensorValues, self.handle_rgbc_request)
        
        # Inicializar conexión en un hilo separado
        import threading
        self.connection_thread = threading.Thread(target=self.run_async_initialization)
        self.connection_thread.daemon = True
        self.connection_thread.start()
        
        rospy.loginfo("🎨 Servicio RGBC Sensor iniciado")
        rospy.loginfo("🔧 Usando get_rgbc_sensor_values (sin activar LED)")
        rospy.loginfo("📡 Servicio disponible en: /get_rgbc_sensor_values")
    
    def run_async_initialization(self):
        """Ejecuta la inicialización asíncrona en un hilo separado."""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self.initialize_connection())
    
    def signal_handler(self, signum, frame):
        """Maneja Ctrl+C."""
        rospy.loginfo("🛑 Deteniendo servicio...")
        self.running = False
        sys.exit(0)
    
    async def initialize_connection(self):
        """Inicializa la conexión con el RVR."""
        try:
            rospy.loginfo("🔌 Conectando al RVR...")
            
            # Crear loop y DAL
            loop = asyncio.get_event_loop()
            self.dal = SerialAsyncDal(loop, port_id='/dev/ttyS0', baud=115200)
            self.rvr = SpheroRvrAsync(self.dal)
            
            # Conectar y despertar
            await self.rvr.connect()
            await self.rvr.wake()
            
            self.connected = True
            rospy.loginfo("✅ RVR conectado exitosamente")
            rospy.loginfo("🎯 Servicio listo para recibir solicitudes")
            
        except Exception as e:
            rospy.logerr(f"❌ Error conectando al RVR: {e}")
            self.connected = False
    
    def handle_rgbc_request(self, req):
        """Maneja las solicitudes del servicio get_rgbc_sensor_values."""
        response = GetRGBCSensorValuesResponse()
        
        if not self.connected:
            response.success = False
            response.message = "RVR no conectado"
            rospy.logwarn("⚠️ Solicitud rechazada: RVR no conectado")
            return response
        
        try:
            rospy.loginfo("📡 Solicitud de datos RGBC recibida")
            
            # Ejecutar get_rgbc_sensor_values de forma asíncrona
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            
            try:
                result = loop.run_until_complete(self.rvr.get_rgbc_sensor_values(timeout=5.0))
                
                if result:
                    # Extraer valores RGBC
                    response.red_channel_value = result.get('red_channel_value', 0)
                    response.green_channel_value = result.get('green_channel_value', 0)
                    response.blue_channel_value = result.get('blue_channel_value', 0)
                    response.clear_channel_value = result.get('clear_channel_value', 0)
                    response.success = True
                    response.message = "Datos RGBC obtenidos exitosamente"
                    
                    rospy.loginfo(f"✅ Datos RGBC: R={response.red_channel_value}, G={response.green_channel_value}, B={response.blue_channel_value}, C={response.clear_channel_value}")
                    
                else:
                    response.success = False
                    response.message = "No se recibieron datos del sensor"
                    rospy.logwarn("⚠️ No se recibieron datos del sensor")
                    
            finally:
                loop.close()
                
        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"
            rospy.logerr(f"❌ Error en servicio RGBC: {e}")
        
        return response
    
    async def cleanup(self):
        """Limpia al finalizar."""
        try:
            if self.rvr and self.connected:
                rospy.loginfo("🧹 Desconectando del RVR...")
                await self.rvr.sleep()
                await self.rvr.disconnect()
                self.connected = False
                rospy.loginfo("✅ Limpieza completada")
        except Exception as e:
            rospy.logwarn(f"⚠️ Error en limpieza: {e}")
    
    def run(self):
        """Ejecuta el servicio."""
        rospy.loginfo("🔄 Servicio activo. Presiona Ctrl+C para salir.")
        
        while self.running and not rospy.is_shutdown():
            rospy.sleep(0.1)

def main():
    """Función principal."""
    try:
        # Crear instancia del servicio
        service = RGBCSensorService()
        
        # Ejecutar servicio
        service.run()
        
    except KeyboardInterrupt:
        rospy.loginfo("🛑 Interrumpido por el usuario")
    except Exception as e:
        rospy.logerr(f"❌ Error: {e}")
    finally:
        rospy.loginfo("👋 Finalizando...")

if __name__ == '__main__':
    main()
