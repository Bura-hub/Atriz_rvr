#!/usr/bin/env python3

"""
Script completo para probar todas las funcionalidades del driver Atriz RVR
Incluye diagnóstico, pruebas automáticas y pruebas interactivas
"""

import rospy
import time
import json
import subprocess
import sys
import os
from datetime import datetime
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Illuminance
from std_msgs.msg import String, Bool, Empty
from sphero_rvr_msgs.msg import Color, DegreesTwist
from sphero_rvr_msgs.srv import SetIRMode, BatteryState
from std_srvs.srv import SetBool, Empty as EmptySrv

class CompleteTester:
    """Clase para ejecutar todas las pruebas del driver Atriz RVR"""
    
    def __init__(self):
        """Inicializa el tester completo"""
        rospy.init_node('complete_tester', anonymous=True)
        
        self.test_results = {
            'timestamp': datetime.now().isoformat(),
            'diagnostic': {},
            'publisher_tests': {},
            'subscriber_tests': {},
            'service_tests': {},
            'movement_tests': {},
            'sensor_tests': {},
            'ir_tests': {},
            'emergency_stop_tests': {},
            'overall_success': False
        }
        
        self.received_data = {}
        
        # Configurar publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.cmd_degrees_pub = rospy.Publisher('/cmd_degrees', DegreesTwist, queue_size=1)
        self.emergency_stop_pub = rospy.Publisher('/is_emergency_stop', Empty, queue_size=1)
        
        # Configurar subscribers
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
        self.color_sub = rospy.Subscriber('/color', Color, self.color_callback)
        self.illuminance_sub = rospy.Subscriber('/ambient_light', Illuminance, self.illuminance_callback)
        self.ir_messages_sub = rospy.Subscriber('/ir_messages', String, self.ir_messages_callback)
        
        # Configurar services
        self.setup_services()
        
        rospy.loginfo("Complete Tester inicializado")
    
    def setup_services(self):
        """Configura los servicios"""
        try:
            rospy.wait_for_service('/enable_color', timeout=5.0)
            rospy.wait_for_service('/battery_state', timeout=5.0)
            rospy.wait_for_service('/reset_odom', timeout=5.0)
            rospy.wait_for_service('/release_emergency_stop', timeout=5.0)
            rospy.wait_for_service('/ir_mode', timeout=5.0)
            
            self.enable_color_srv = rospy.ServiceProxy('/enable_color', SetBool)
            self.battery_state_srv = rospy.ServiceProxy('/battery_state', BatteryState)
            self.reset_odom_srv = rospy.ServiceProxy('/reset_odom', EmptySrv)
            self.release_emergency_stop_srv = rospy.ServiceProxy('/release_emergency_stop', EmptySrv)
            self.ir_mode_srv = rospy.ServiceProxy('/ir_mode', SetIRMode)
            
            rospy.loginfo("Servicios configurados correctamente")
            return True
        except rospy.ROSException as e:
            rospy.logerr(f"Error configurando servicios: {e}")
            return False
    
    # Callbacks
    def odom_callback(self, data):
        self.received_data['odom'] = data
    
    def imu_callback(self, data):
        self.received_data['imu'] = data
    
    def color_callback(self, data):
        self.received_data['color'] = data
    
    def illuminance_callback(self, data):
        self.received_data['illuminance'] = data
    
    def ir_messages_callback(self, data):
        self.received_data['ir_messages'] = data
    
    def run_diagnostic(self):
        """Ejecuta diagnóstico del sistema"""
        rospy.loginfo("=== EJECUTANDO DIAGNÓSTICO ===")
        
        diagnostic_results = {
            'ros_master': False,
            'driver_running': False,
            'topics_available': {},
            'services_available': {},
            'sensor_data': {},
            'battery_status': {},
            'emergency_stop_status': False
        }
        
        # Verificar ROS master
        try:
            topics = rospy.get_published_topics()
            diagnostic_results['ros_master'] = True
            rospy.loginfo("✓ ROS master funcionando")
        except Exception as e:
            rospy.logerr(f"✗ ROS master no disponible: {e}")
        
        # Verificar driver
        try:
            result = subprocess.run(['pgrep', '-f', 'Atriz_rvr_node.py'], 
                                  capture_output=True, text=True)
            diagnostic_results['driver_running'] = result.returncode == 0
            if diagnostic_results['driver_running']:
                rospy.loginfo("✓ Driver Atriz RVR ejecutándose")
            else:
                rospy.logerr("✗ Driver Atriz RVR no ejecutándose")
        except Exception as e:
            rospy.logerr(f"✗ Error verificando driver: {e}")
        
        # Verificar tópicos
        expected_topics = ['/odom', '/imu', '/color', '/ambient_light', '/ir_messages']
        try:
            published_topics = [topic[0] for topic in rospy.get_published_topics()]
            for topic in expected_topics:
                diagnostic_results['topics_available'][topic] = topic in published_topics
        except Exception as e:
            rospy.logerr(f"✗ Error verificando tópicos: {e}")
        
        # Verificar servicios
        expected_services = ['/enable_color', '/battery_state', '/reset_odom', 
                           '/release_emergency_stop', '/ir_mode']
        for service in expected_services:
            try:
                rospy.wait_for_service(service, timeout=2.0)
                diagnostic_results['services_available'][service] = True
            except rospy.ROSException:
                diagnostic_results['services_available'][service] = False
        
        # Verificar datos de sensores
        rospy.loginfo("Esperando datos de sensores...")
        rospy.sleep(3.0)
        
        for sensor in ['odom', 'imu', 'illuminance']:
            if sensor in self.received_data and self.received_data[sensor] is not None:
                diagnostic_results['sensor_data'][sensor] = True
                rospy.loginfo(f"✓ Datos de {sensor} recibidos")
            else:
                diagnostic_results['sensor_data'][sensor] = False
                rospy.logwarn(f"✗ No se recibieron datos de {sensor}")
        
        # Verificar batería
        try:
            response = self.battery_state_srv()
            diagnostic_results['battery_status'] = {
                'percentage': response.battery_percentage,
                'voltage_state': response.voltage_state
            }
            rospy.loginfo(f"✓ Batería: {response.battery_percentage}%")
        except Exception as e:
            rospy.logerr(f"✗ Error obteniendo estado de batería: {e}")
            diagnostic_results['battery_status'] = None
        
        # Verificar parada de emergencia
        try:
            emergency_stop = rospy.get_param('/emergency_stop', False)
            diagnostic_results['emergency_stop_status'] = emergency_stop
            if emergency_stop:
                rospy.logwarn("⚠ Parada de emergencia activa")
            else:
                rospy.loginfo("✓ Parada de emergencia inactiva")
        except Exception as e:
            rospy.logerr(f"✗ Error verificando parada de emergencia: {e}")
        
        self.test_results['diagnostic'] = diagnostic_results
        return diagnostic_results
    
    def test_publishers(self):
        """Prueba todos los publishers"""
        rospy.loginfo("=== PROBANDO PUBLISHERS ===")
        
        publisher_results = {}
        
        # Test cmd_vel
        try:
            twist = Twist()
            twist.linear.x = 0.1
            twist.angular.z = 0.2
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(0.5)
            publisher_results['cmd_vel'] = True
            rospy.loginfo("✓ cmd_vel funcionando")
        except Exception as e:
            publisher_results['cmd_vel'] = False
            rospy.logerr(f"✗ Error en cmd_vel: {e}")
        
        # Test cmd_degrees
        try:
            degrees_twist = DegreesTwist()
            degrees_twist.linear_x = 0.1
            degrees_twist.angular_z = 20.0
            self.cmd_degrees_pub.publish(degrees_twist)
            rospy.sleep(0.5)
            publisher_results['cmd_degrees'] = True
            rospy.loginfo("✓ cmd_degrees funcionando")
        except Exception as e:
            publisher_results['cmd_degrees'] = False
            rospy.logerr(f"✗ Error en cmd_degrees: {e}")
        
        # Test emergency stop
        try:
            self.emergency_stop_pub.publish(Empty())
            rospy.sleep(1.0)
            publisher_results['emergency_stop'] = True
            rospy.loginfo("✓ emergency_stop funcionando")
        except Exception as e:
            publisher_results['emergency_stop'] = False
            rospy.logerr(f"✗ Error en emergency_stop: {e}")
        
        self.test_results['publisher_tests'] = publisher_results
        return publisher_results
    
    def test_subscribers(self):
        """Prueba todos los subscribers"""
        rospy.loginfo("=== PROBANDO SUBSCRIBERS ===")
        
        subscriber_results = {}
        
        # Esperar datos de sensores
        rospy.loginfo("Esperando datos de sensores...")
        rospy.sleep(5.0)
        
        for sensor in ['odom', 'imu', 'illuminance']:
            if sensor in self.received_data and self.received_data[sensor] is not None:
                subscriber_results[sensor] = True
                rospy.loginfo(f"✓ {sensor} recibiendo datos")
            else:
                subscriber_results[sensor] = False
                rospy.logwarn(f"✗ {sensor} no recibiendo datos")
        
        self.test_results['subscriber_tests'] = subscriber_results
        return subscriber_results
    
    def test_services(self):
        """Prueba todos los servicios"""
        rospy.loginfo("=== PROBANDO SERVICIOS ===")
        
        service_results = {}
        
        # Test enable_color
        try:
            response = self.enable_color_srv(True)
            service_results['enable_color'] = True
            rospy.loginfo("✓ enable_color funcionando")
        except Exception as e:
            service_results['enable_color'] = False
            rospy.logerr(f"✗ Error en enable_color: {e}")
        
        # Test battery_state
        try:
            response = self.battery_state_srv()
            service_results['battery_state'] = True
            rospy.loginfo(f"✓ battery_state funcionando: {response.battery_percentage}%")
        except Exception as e:
            service_results['battery_state'] = False
            rospy.logerr(f"✗ Error en battery_state: {e}")
        
        # Test reset_odom
        try:
            response = self.reset_odom_srv()
            service_results['reset_odom'] = True
            rospy.loginfo("✓ reset_odom funcionando")
        except Exception as e:
            service_results['reset_odom'] = False
            rospy.logerr(f"✗ Error en reset_odom: {e}")
        
        # Test release_emergency_stop
        try:
            response = self.release_emergency_stop_srv()
            service_results['release_emergency_stop'] = True
            rospy.loginfo("✓ release_emergency_stop funcionando")
        except Exception as e:
            service_results['release_emergency_stop'] = False
            rospy.logerr(f"✗ Error en release_emergency_stop: {e}")
        
        self.test_results['service_tests'] = service_results
        return service_results
    
    def test_movement(self):
        """Prueba las funciones de movimiento"""
        rospy.loginfo("=== PROBANDO MOVIMIENTO ===")
        
        movement_results = {}
        
        # Test movimiento hacia adelante
        try:
            twist = Twist()
            twist.linear.x = 0.2
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(2.0)
            movement_results['forward'] = True
            rospy.loginfo("✓ Movimiento hacia adelante")
        except Exception as e:
            movement_results['forward'] = False
            rospy.logerr(f"✗ Error en movimiento hacia adelante: {e}")
        
        # Test giro
        try:
            twist.linear.x = 0.0
            twist.angular.z = 0.5
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(2.0)
            movement_results['turn'] = True
            rospy.loginfo("✓ Giro")
        except Exception as e:
            movement_results['turn'] = False
            rospy.logerr(f"✗ Error en giro: {e}")
        
        # Parar
        try:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            movement_results['stop'] = True
            rospy.loginfo("✓ Parada")
        except Exception as e:
            movement_results['stop'] = False
            rospy.logerr(f"✗ Error en parada: {e}")
        
        self.test_results['movement_tests'] = movement_results
        return movement_results
    
    def test_ir_functionality(self):
        """Prueba la funcionalidad IR"""
        rospy.loginfo("=== PROBANDO FUNCIONALIDAD IR ===")
        
        ir_results = {}
        
        # Test IR broadcast
        try:
            response = self.ir_mode_srv('broadcast', 1, 2)
            ir_results['broadcast'] = response.success
            rospy.loginfo("✓ IR broadcast")
        except Exception as e:
            ir_results['broadcast'] = False
            rospy.logerr(f"✗ Error en IR broadcast: {e}")
        
        rospy.sleep(2.0)
        
        # Test IR following
        try:
            response = self.ir_mode_srv('following', 3, 4)
            ir_results['following'] = response.success
            rospy.loginfo("✓ IR following")
        except Exception as e:
            ir_results['following'] = False
            rospy.logerr(f"✗ Error en IR following: {e}")
        
        rospy.sleep(2.0)
        
        # Test IR off
        try:
            response = self.ir_mode_srv('off', 0, 0)
            ir_results['off'] = response.success
            rospy.loginfo("✓ IR off")
        except Exception as e:
            ir_results['off'] = False
            rospy.logerr(f"✗ Error en IR off: {e}")
        
        self.test_results['ir_tests'] = ir_results
        return ir_results
    
    def test_emergency_stop_system(self):
        """Prueba el sistema de parada de emergencia"""
        rospy.loginfo("=== PROBANDO SISTEMA DE PARADA DE EMERGENCIA ===")
        
        emergency_results = {}
        
        # Iniciar movimiento
        try:
            twist = Twist()
            twist.linear.x = 0.3
            twist.angular.z = 0.2
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(1.0)
            emergency_results['start_movement'] = True
            rospy.loginfo("✓ Movimiento iniciado")
        except Exception as e:
            emergency_results['start_movement'] = False
            rospy.logerr(f"✗ Error iniciando movimiento: {e}")
        
        # Activar parada de emergencia
        try:
            self.emergency_stop_pub.publish(Empty())
            rospy.sleep(2.0)
            emergency_results['activate_emergency'] = True
            rospy.loginfo("✓ Parada de emergencia activada")
        except Exception as e:
            emergency_results['activate_emergency'] = False
            rospy.logerr(f"✗ Error activando parada de emergencia: {e}")
        
        # Liberar parada de emergencia
        try:
            self.release_emergency_stop_srv()
            rospy.sleep(1.0)
            emergency_results['release_emergency'] = True
            rospy.loginfo("✓ Parada de emergencia liberada")
        except Exception as e:
            emergency_results['release_emergency'] = False
            rospy.logerr(f"✗ Error liberando parada de emergencia: {e}")
        
        self.test_results['emergency_stop_tests'] = emergency_results
        return emergency_results
    
    def generate_final_report(self):
        """Genera el reporte final"""
        rospy.loginfo("=== GENERANDO REPORTE FINAL ===")
        
        # Calcular estadísticas
        total_tests = 0
        passed_tests = 0
        
        for category, tests in self.test_results.items():
            if isinstance(tests, dict) and category != 'timestamp':
                for test_name, result in tests.items():
                    if isinstance(result, bool):
                        total_tests += 1
                        if result:
                            passed_tests += 1
        
        success_rate = (passed_tests / total_tests * 100) if total_tests > 0 else 0
        
        self.test_results['overall_success'] = success_rate >= 80.0
        
        rospy.loginfo(f"Total de pruebas: {total_tests}")
        rospy.loginfo(f"Pruebas exitosas: {passed_tests}")
        rospy.loginfo(f"Pruebas fallidas: {total_tests - passed_tests}")
        rospy.loginfo(f"Porcentaje de éxito: {success_rate:.1f}%")
        
        if self.test_results['overall_success']:
            rospy.loginfo("✓ SISTEMA FUNCIONANDO CORRECTAMENTE")
        else:
            rospy.logwarn("⚠ SISTEMA CON PROBLEMAS - REVISAR RESULTADOS")
        
        # Guardar reporte
        try:
            with open('/tmp/atriz_rvr_complete_test_report.json', 'w') as f:
                json.dump(self.test_results, f, indent=2, default=str)
            rospy.loginfo("Reporte guardado en /tmp/atriz_rvr_complete_test_report.json")
        except Exception as e:
            rospy.logerr(f"Error guardando reporte: {e}")
    
    def run_complete_test_suite(self):
        """Ejecuta la suite completa de pruebas"""
        rospy.loginfo("Iniciando suite completa de pruebas...")
        
        try:
            # Esperar a que el sistema esté listo
            rospy.loginfo("Esperando a que el sistema esté listo...")
            rospy.sleep(5.0)
            
            # Ejecutar diagnóstico
            self.run_diagnostic()
            rospy.sleep(2.0)
            
            # Ejecutar pruebas de publishers
            self.test_publishers()
            rospy.sleep(2.0)
            
            # Ejecutar pruebas de subscribers
            self.test_subscribers()
            rospy.sleep(2.0)
            
            # Ejecutar pruebas de servicios
            self.test_services()
            rospy.sleep(2.0)
            
            # Ejecutar pruebas de movimiento
            self.test_movement()
            rospy.sleep(2.0)
            
            # Ejecutar pruebas de IR
            self.test_ir_functionality()
            rospy.sleep(2.0)
            
            # Ejecutar pruebas de parada de emergencia
            self.test_emergency_stop_system()
            rospy.sleep(2.0)
            
            # Generar reporte final
            self.generate_final_report()
            
            rospy.loginfo("Suite completa de pruebas finalizada")
            
        except rospy.ROSInterruptException:
            rospy.loginfo("Pruebas interrumpidas por el usuario")
        except Exception as e:
            rospy.logerr(f"Error durante las pruebas: {e}")
            import traceback
            traceback.print_exc()

def main():
    """Función principal"""
    try:
        tester = CompleteTester()
        tester.run_complete_test_suite()
    except rospy.ROSInterruptException:
        rospy.loginfo("Programa interrumpido")
    except Exception as e:
        rospy.logerr(f"Error en el programa principal: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()
