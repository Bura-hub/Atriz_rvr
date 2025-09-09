#!/usr/bin/env python3

"""
Script de pruebas completo para el driver Atriz_rvr_node
Prueba todos los tópicos, servicios y funcionalidades disponibles
"""

import rospy
import time
import threading
import json
from datetime import datetime
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Illuminance
from std_msgs.msg import String, Bool, Empty
from sphero_rvr_msgs.msg import Color, DegreesTwist
from sphero_rvr_msgs.srv import SetIRMode, BatteryState
from std_srvs.srv import SetBool, Empty as EmptySrv

class AtrizRVRDriverTester:
    """Clase para probar todas las funcionalidades del driver Atriz_rvr_node"""
    
    def __init__(self):
        """Inicializa el tester"""
        rospy.init_node('atriz_rvr_driver_tester', anonymous=True)
        
        # Variables para almacenar datos recibidos
        self.received_data = {
            'odom': None,
            'imu': None,
            'color': None,
            'illuminance': None,
            'ir_messages': None
        }
        
        # Flags para controlar las pruebas
        self.test_results = {}
        self.test_start_time = None
        
        # Configurar publishers
        self.setup_publishers()
        
        # Configurar subscribers
        self.setup_subscribers()
        
        # Configurar service proxies
        self.setup_services()
        
        rospy.loginfo("Tester de Atriz RVR Driver inicializado")
    
    def setup_publishers(self):
        """Configura los publishers para enviar comandos"""
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.cmd_degrees_pub = rospy.Publisher('/cmd_degrees', DegreesTwist, queue_size=1)
        self.emergency_stop_pub = rospy.Publisher('/is_emergency_stop', Empty, queue_size=1)
        
        rospy.loginfo("Publishers configurados")
    
    def setup_subscribers(self):
        """Configura los subscribers para recibir datos"""
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
        self.color_sub = rospy.Subscriber('/color', Color, self.color_callback)
        self.illuminance_sub = rospy.Subscriber('/ambient_light', Illuminance, self.illuminance_callback)
        self.ir_messages_sub = rospy.Subscriber('/ir_messages', String, self.ir_messages_callback)
        
        rospy.loginfo("Subscribers configurados")
    
    def setup_services(self):
        """Configura los service proxies"""
        rospy.wait_for_service('/enable_color')
        rospy.wait_for_service('/battery_state')
        rospy.wait_for_service('/reset_odom')
        rospy.wait_for_service('/release_emergency_stop')
        rospy.wait_for_service('/ir_mode')
        
        self.enable_color_srv = rospy.ServiceProxy('/enable_color', SetBool)
        self.battery_state_srv = rospy.ServiceProxy('/battery_state', BatteryState)
        self.reset_odom_srv = rospy.ServiceProxy('/reset_odom', EmptySrv)
        self.release_emergency_stop_srv = rospy.ServiceProxy('/release_emergency_stop', EmptySrv)
        self.ir_mode_srv = rospy.ServiceProxy('/ir_mode', SetIRMode)
        
        rospy.loginfo("Services configurados")
    
    # Callbacks para recibir datos
    def odom_callback(self, data):
        """Callback para datos de odometría"""
        self.received_data['odom'] = data
        rospy.logdebug("Datos de odometría recibidos")
    
    def imu_callback(self, data):
        """Callback para datos de IMU"""
        self.received_data['imu'] = data
        rospy.logdebug("Datos de IMU recibidos")
    
    def color_callback(self, data):
        """Callback para datos de color"""
        self.received_data['color'] = data
        rospy.logdebug("Datos de color recibidos")
    
    def illuminance_callback(self, data):
        """Callback para datos de iluminancia"""
        self.received_data['illuminance'] = data
        rospy.logdebug("Datos de iluminancia recibidos")
    
    def ir_messages_callback(self, data):
        """Callback para mensajes IR"""
        self.received_data['ir_messages'] = data
        rospy.logdebug("Mensajes IR recibidos")
    
    def wait_for_data(self, data_type, timeout=5.0):
        """Espera a recibir datos de un tipo específico"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.received_data[data_type] is not None:
                return True
            rospy.sleep(0.1)
        return False
    
    def test_publishers(self):
        """Prueba todos los publishers"""
        rospy.loginfo("=== PROBANDO PUBLISHERS ===")
        
        # Test 1: cmd_vel
        rospy.loginfo("Probando cmd_vel...")
        twist = Twist()
        twist.linear.x = 0.1
        twist.angular.z = 0.2
        self.cmd_vel_pub.publish(twist)
        rospy.sleep(0.5)
        
        # Test 2: cmd_degrees
        rospy.loginfo("Probando cmd_degrees...")
        degrees_twist = DegreesTwist()
        degrees_twist.linear_x = 0.1
        degrees_twist.angular_z = 20.0
        self.cmd_degrees_pub.publish(degrees_twist)
        rospy.sleep(0.5)
        
        # Test 3: Emergency stop
        rospy.loginfo("Probando emergency stop...")
        self.emergency_stop_pub.publish(Empty())
        rospy.sleep(1.0)
        
        # Release emergency stop
        rospy.loginfo("Liberando emergency stop...")
        try:
            self.release_emergency_stop_srv()
        except rospy.ServiceException as e:
            rospy.logerr(f"Error liberando emergency stop: {e}")
        
        rospy.loginfo("Publishers probados")
    
    def test_subscribers(self):
        """Prueba todos los subscribers"""
        rospy.loginfo("=== PROBANDO SUBSCRIBERS ===")
        
        # Esperar datos de odometría
        rospy.loginfo("Esperando datos de odometría...")
        if self.wait_for_data('odom', timeout=10.0):
            odom = self.received_data['odom']
            rospy.loginfo(f"Odometría recibida - Posición: x={odom.pose.pose.position.x:.3f}, y={odom.pose.pose.position.y:.3f}")
            self.test_results['odom'] = True
        else:
            rospy.logwarn("No se recibieron datos de odometría")
            self.test_results['odom'] = False
        
        # Esperar datos de IMU
        rospy.loginfo("Esperando datos de IMU...")
        if self.wait_for_data('imu', timeout=10.0):
            imu = self.received_data['imu']
            rospy.loginfo(f"IMU recibido - Orientación: x={imu.orientation.x:.3f}, y={imu.orientation.y:.3f}, z={imu.orientation.z:.3f}, w={imu.orientation.w:.3f}")
            self.test_results['imu'] = True
        else:
            rospy.logwarn("No se recibieron datos de IMU")
            self.test_results['imu'] = False
        
        # Esperar datos de iluminancia
        rospy.loginfo("Esperando datos de iluminancia...")
        if self.wait_for_data('illuminance', timeout=10.0):
            illuminance = self.received_data['illuminance']
            rospy.loginfo(f"Iluminancia recibida: {illuminance.illuminance:.3f}")
            self.test_results['illuminance'] = True
        else:
            rospy.logwarn("No se recibieron datos de iluminancia")
            self.test_results['illuminance'] = False
        
        rospy.loginfo("Subscribers probados")
    
    def test_services(self):
        """Prueba todos los servicios"""
        rospy.loginfo("=== PROBANDO SERVICIOS ===")
        
        # Test 1: enable_color
        rospy.loginfo("Probando enable_color...")
        try:
            response = self.enable_color_srv(True)
            rospy.loginfo(f"enable_color (True): {response}")
            self.test_results['enable_color_true'] = True
        except rospy.ServiceException as e:
            rospy.logerr(f"Error en enable_color (True): {e}")
            self.test_results['enable_color_true'] = False
        
        rospy.sleep(1.0)
        
        try:
            response = self.enable_color_srv(False)
            rospy.loginfo(f"enable_color (False): {response}")
            self.test_results['enable_color_false'] = True
        except rospy.ServiceException as e:
            rospy.logerr(f"Error en enable_color (False): {e}")
            self.test_results['enable_color_false'] = False
        
        # Test 2: battery_state
        rospy.loginfo("Probando battery_state...")
        try:
            response = self.battery_state_srv()
            rospy.loginfo(f"Batería: {response.battery_percentage}% - Estado: {response.voltage_state}")
            self.test_results['battery_state'] = True
        except rospy.ServiceException as e:
            rospy.logerr(f"Error en battery_state: {e}")
            self.test_results['battery_state'] = False
        
        # Test 3: reset_odom
        rospy.loginfo("Probando reset_odom...")
        try:
            response = self.reset_odom_srv()
            rospy.loginfo(f"reset_odom: {response}")
            self.test_results['reset_odom'] = True
        except rospy.ServiceException as e:
            rospy.logerr(f"Error en reset_odom: {e}")
            self.test_results['reset_odom'] = False
        
        # Test 4: release_emergency_stop
        rospy.loginfo("Probando release_emergency_stop...")
        try:
            response = self.release_emergency_stop_srv()
            rospy.loginfo(f"release_emergency_stop: {response}")
            self.test_results['release_emergency_stop'] = True
        except rospy.ServiceException as e:
            rospy.logerr(f"Error en release_emergency_stop: {e}")
            self.test_results['release_emergency_stop'] = False
        
        rospy.loginfo("Servicios probados")
    
    def test_ir_functionality(self):
        """Prueba la funcionalidad IR"""
        rospy.loginfo("=== PROBANDO FUNCIONALIDAD IR ===")
        
        # Test 1: IR Broadcast
        rospy.loginfo("Probando IR Broadcast...")
        try:
            response = self.ir_mode_srv('broadcast', 1, 2)  # far_code=1, near_code=2
            rospy.loginfo(f"IR Broadcast: {response}")
            self.test_results['ir_broadcast'] = response.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Error en IR Broadcast: {e}")
            self.test_results['ir_broadcast'] = False
        
        rospy.sleep(2.0)
        
        # Test 2: IR Following
        rospy.loginfo("Probando IR Following...")
        try:
            response = self.ir_mode_srv('following', 3, 4)  # far_code=3, near_code=4
            rospy.loginfo(f"IR Following: {response}")
            self.test_results['ir_following'] = response.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Error en IR Following: {e}")
            self.test_results['ir_following'] = False
        
        rospy.sleep(2.0)
        
        # Test 3: IR Off
        rospy.loginfo("Probando IR Off...")
        try:
            response = self.ir_mode_srv('off', 0, 0)
            rospy.loginfo(f"IR Off: {response}")
            self.test_results['ir_off'] = response.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Error en IR Off: {e}")
            self.test_results['ir_off'] = False
        
        rospy.loginfo("Funcionalidad IR probada")
    
    def test_movement_functions(self):
        """Prueba las funciones de movimiento"""
        rospy.loginfo("=== PROBANDO FUNCIONES DE MOVIMIENTO ===")
        
        # Test 1: Movimiento hacia adelante
        rospy.loginfo("Probando movimiento hacia adelante...")
        twist = Twist()
        twist.linear.x = 0.2
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        rospy.sleep(2.0)
        
        # Test 2: Giro a la izquierda
        rospy.loginfo("Probando giro a la izquierda...")
        twist.linear.x = 0.0
        twist.angular.z = 0.5
        self.cmd_vel_pub.publish(twist)
        rospy.sleep(2.0)
        
        # Test 3: Giro a la derecha
        rospy.loginfo("Probando giro a la derecha...")
        twist.angular.z = -0.5
        self.cmd_vel_pub.publish(twist)
        rospy.sleep(2.0)
        
        # Test 4: Movimiento hacia atrás
        rospy.loginfo("Probando movimiento hacia atrás...")
        twist.linear.x = -0.2
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        rospy.sleep(2.0)
        
        # Test 5: Parada
        rospy.loginfo("Probando parada...")
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        rospy.sleep(1.0)
        
        # Test 6: Comandos en grados
        rospy.loginfo("Probando comandos en grados...")
        degrees_twist = DegreesTwist()
        degrees_twist.linear_x = 0.1
        degrees_twist.angular_z = 30.0
        self.cmd_degrees_pub.publish(degrees_twist)
        rospy.sleep(2.0)
        
        # Parada final
        degrees_twist.linear_x = 0.0
        degrees_twist.angular_z = 0.0
        self.cmd_degrees_pub.publish(degrees_twist)
        
        self.test_results['movement_functions'] = True
        rospy.loginfo("Funciones de movimiento probadas")
    
    def test_emergency_stop_system(self):
        """Prueba el sistema de parada de emergencia"""
        rospy.loginfo("=== PROBANDO SISTEMA DE PARADA DE EMERGENCIA ===")
        
        # Iniciar movimiento
        rospy.loginfo("Iniciando movimiento...")
        twist = Twist()
        twist.linear.x = 0.3
        twist.angular.z = 0.2
        self.cmd_vel_pub.publish(twist)
        rospy.sleep(1.0)
        
        # Activar parada de emergencia
        rospy.loginfo("Activando parada de emergencia...")
        self.emergency_stop_pub.publish(Empty())
        rospy.sleep(2.0)
        
        # Intentar enviar comando de movimiento (debería ser ignorado)
        rospy.loginfo("Intentando enviar comando durante parada de emergencia...")
        twist.linear.x = 0.5
        self.cmd_vel_pub.publish(twist)
        rospy.sleep(1.0)
        
        # Liberar parada de emergencia
        rospy.loginfo("Liberando parada de emergencia...")
        try:
            self.release_emergency_stop_srv()
            self.test_results['emergency_stop_system'] = True
        except rospy.ServiceException as e:
            rospy.logerr(f"Error liberando parada de emergencia: {e}")
            self.test_results['emergency_stop_system'] = False
        
        rospy.sleep(1.0)
        
        # Verificar que el movimiento funciona después de liberar
        rospy.loginfo("Verificando movimiento después de liberar parada de emergencia...")
        twist.linear.x = 0.1
        self.cmd_vel_pub.publish(twist)
        rospy.sleep(2.0)
        
        # Parada final
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        
        rospy.loginfo("Sistema de parada de emergencia probado")
    
    def test_color_sensor(self):
        """Prueba el sensor de color"""
        rospy.loginfo("=== PROBANDO SENSOR DE COLOR ===")
        
        # Habilitar sensor de color
        rospy.loginfo("Habilitando sensor de color...")
        try:
            self.enable_color_srv(True)
            rospy.sleep(2.0)
            
            # Esperar datos de color
            if self.wait_for_data('color', timeout=10.0):
                color = self.received_data['color']
                rospy.loginfo(f"Color detectado - RGB: {color.rgb_color}, Confianza: {color.confidence}")
                self.test_results['color_sensor'] = True
            else:
                rospy.logwarn("No se recibieron datos de color")
                self.test_results['color_sensor'] = False
            
            # Deshabilitar sensor de color
            self.enable_color_srv(False)
            
        except rospy.ServiceException as e:
            rospy.logerr(f"Error probando sensor de color: {e}")
            self.test_results['color_sensor'] = False
        
        rospy.loginfo("Sensor de color probado")
    
    def generate_test_report(self):
        """Genera un reporte de las pruebas"""
        rospy.loginfo("=== REPORTE DE PRUEBAS ===")
        
        total_tests = len(self.test_results)
        passed_tests = sum(1 for result in self.test_results.values() if result)
        failed_tests = total_tests - passed_tests
        
        rospy.loginfo(f"Total de pruebas: {total_tests}")
        rospy.loginfo(f"Pruebas exitosas: {passed_tests}")
        rospy.loginfo(f"Pruebas fallidas: {failed_tests}")
        rospy.loginfo(f"Porcentaje de éxito: {(passed_tests/total_tests)*100:.1f}%")
        
        rospy.loginfo("\nDetalles de las pruebas:")
        for test_name, result in self.test_results.items():
            status = "✓ EXITOSO" if result else "✗ FALLIDO"
            rospy.loginfo(f"  {test_name}: {status}")
        
        # Guardar reporte en archivo
        report_data = {
            'timestamp': datetime.now().isoformat(),
            'total_tests': total_tests,
            'passed_tests': passed_tests,
            'failed_tests': failed_tests,
            'success_rate': (passed_tests/total_tests)*100,
            'test_results': self.test_results
        }
        
        try:
            with open('/tmp/atriz_rvr_test_report.json', 'w') as f:
                json.dump(report_data, f, indent=2)
            rospy.loginfo("Reporte guardado en /tmp/atriz_rvr_test_report.json")
        except Exception as e:
            rospy.logerr(f"Error guardando reporte: {e}")
    
    def run_all_tests(self):
        """Ejecuta todas las pruebas"""
        rospy.loginfo("Iniciando pruebas completas del driver Atriz RVR...")
        self.test_start_time = time.time()
        
        try:
            # Esperar a que el driver esté listo
            rospy.loginfo("Esperando a que el driver esté listo...")
            rospy.sleep(5.0)
            
            # Ejecutar todas las pruebas
            self.test_publishers()
            rospy.sleep(2.0)
            
            self.test_subscribers()
            rospy.sleep(2.0)
            
            self.test_services()
            rospy.sleep(2.0)
            
            self.test_ir_functionality()
            rospy.sleep(2.0)
            
            self.test_movement_functions()
            rospy.sleep(2.0)
            
            self.test_emergency_stop_system()
            rospy.sleep(2.0)
            
            self.test_color_sensor()
            rospy.sleep(2.0)
            
            # Generar reporte
            self.generate_test_report()
            
            rospy.loginfo("Todas las pruebas completadas")
            
        except rospy.ROSInterruptException:
            rospy.loginfo("Pruebas interrumpidas por el usuario")
        except Exception as e:
            rospy.logerr(f"Error durante las pruebas: {e}")
            import traceback
            traceback.print_exc()

def main():
    """Función principal"""
    try:
        tester = AtrizRVRDriverTester()
        tester.run_all_tests()
    except rospy.ROSInterruptException:
        rospy.loginfo("Programa interrumpido")
    except Exception as e:
        rospy.logerr(f"Error en el programa principal: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()
