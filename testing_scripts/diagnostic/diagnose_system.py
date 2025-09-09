#!/usr/bin/env python3

"""
Script de diagnóstico para el sistema Atriz RVR
Verifica el estado del driver, sensores y funcionalidades
"""

import rospy
import time
import subprocess
import json
from datetime import datetime
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Illuminance
from std_msgs.msg import String, Bool, Empty
from sphero_rvr_msgs.msg import Color, DegreesTwist
from sphero_rvr_msgs.srv import SetIRMode, BatteryState
from std_srvs.srv import SetBool, Empty as EmptySrv

class SystemDiagnostic:
    """Clase para diagnosticar el sistema Atriz RVR"""
    
    def __init__(self):
        """Inicializa el diagnosticador"""
        rospy.init_node('system_diagnostic', anonymous=True)
        
        self.diagnostic_results = {
            'timestamp': datetime.now().isoformat(),
            'ros_master': False,
            'driver_running': False,
            'topics_available': {},
            'services_available': {},
            'sensor_data': {},
            'battery_status': {},
            'emergency_stop_status': False,
            'recommendations': []
        }
        
        rospy.loginfo("Sistema de diagnóstico inicializado")
    
    def check_ros_master(self):
        """Verifica si el master de ROS está ejecutándose"""
        rospy.loginfo("Verificando master de ROS...")
        
        try:
            # Intentar obtener la lista de nodos
            nodes = rospy.get_published_topics()
            self.diagnostic_results['ros_master'] = True
            rospy.loginfo("✓ Master de ROS está ejecutándose")
            return True
        except Exception as e:
            self.diagnostic_results['ros_master'] = False
            rospy.logerr(f"✗ Master de ROS no está ejecutándose: {e}")
            self.diagnostic_results['recommendations'].append("Iniciar roscore")
            return False
    
    def check_driver_running(self):
        """Verifica si el driver Atriz RVR está ejecutándose"""
        rospy.loginfo("Verificando driver Atriz RVR...")
        
        try:
            # Verificar si el proceso está ejecutándose
            result = subprocess.run(['pgrep', '-f', 'Atriz_rvr_node.py'], 
                                  capture_output=True, text=True)
            
            if result.returncode == 0:
                self.diagnostic_results['driver_running'] = True
                rospy.loginfo("✓ Driver Atriz RVR está ejecutándose")
                return True
            else:
                self.diagnostic_results['driver_running'] = False
                rospy.logerr("✗ Driver Atriz RVR no está ejecutándose")
                self.diagnostic_results['recommendations'].append("Iniciar el driver Atriz RVR")
                return False
        except Exception as e:
            rospy.logerr(f"✗ Error verificando driver: {e}")
            return False
    
    def check_topics(self):
        """Verifica la disponibilidad de tópicos"""
        rospy.loginfo("Verificando tópicos...")
        
        expected_topics = [
            '/odom',
            '/imu',
            '/color',
            '/ambient_light',
            '/ir_messages',
            '/cmd_vel',
            '/cmd_degrees',
            '/is_emergency_stop'
        ]
        
        available_topics = []
        try:
            # Usar rostopic list para obtener todos los tópicos (incluyendo los sin publicadores)
            import subprocess
            result = subprocess.run(['rostopic', 'list'], capture_output=True, text=True)
            if result.returncode == 0:
                topic_names = result.stdout.strip().split('\n')
            else:
                # Fallback a rospy si rostopic no funciona
                published_topics = rospy.get_published_topics()
                topic_names = [topic[0] for topic in published_topics]
            
            for topic in expected_topics:
                if topic in topic_names:
                    available_topics.append(topic)
                    self.diagnostic_results['topics_available'][topic] = True
                    rospy.loginfo(f"✓ Tópico {topic} disponible")
                else:
                    self.diagnostic_results['topics_available'][topic] = False
                    # Solo mostrar warning si el driver debería estar ejecutándose
                    if self.diagnostic_results['driver_running']:
                        rospy.logwarn(f"✗ Tópico {topic} no disponible (driver ejecutándose)")
                    else:
                        rospy.loginfo(f"ℹ Tópico {topic} no disponible (driver no ejecutándose)")
            
            return len(available_topics) == len(expected_topics)
            
        except Exception as e:
            rospy.logerr(f"✗ Error verificando tópicos: {e}")
            return False
    
    def check_services(self):
        """Verifica la disponibilidad de servicios"""
        rospy.loginfo("Verificando servicios...")
        
        expected_services = [
            '/enable_color',
            '/battery_state',
            '/reset_odom',
            '/release_emergency_stop',
            '/ir_mode'
        ]
        
        available_services = []
        
        for service in expected_services:
            try:
                rospy.wait_for_service(service, timeout=2.0)
                available_services.append(service)
                self.diagnostic_results['services_available'][service] = True
                rospy.loginfo(f"✓ Servicio {service} disponible")
            except rospy.ROSException:
                self.diagnostic_results['services_available'][service] = False
                # Solo mostrar warning si el driver debería estar ejecutándose
                if self.diagnostic_results['driver_running']:
                    rospy.logwarn(f"✗ Servicio {service} no disponible (driver ejecutándose)")
                else:
                    rospy.loginfo(f"ℹ Servicio {service} no disponible (driver no ejecutándose)")
        
        return len(available_services) == len(expected_services)
    
    def check_sensor_data(self):
        """Verifica si se están recibiendo datos de sensores"""
        rospy.loginfo("Verificando datos de sensores...")
        
        sensor_data = {}
        timeout = 5.0
        
        # Verificar odometría
        try:
            odom_msg = rospy.wait_for_message('/odom', Odometry, timeout=timeout)
            sensor_data['odom'] = {
                'position': [odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y],
                'velocity': [odom_msg.twist.twist.linear.x, odom_msg.twist.twist.angular.z],
                'timestamp': odom_msg.header.stamp.to_sec()
            }
            rospy.loginfo("✓ Datos de odometría recibidos")
        except rospy.ROSException:
            rospy.logwarn("✗ No se recibieron datos de odometría")
            sensor_data['odom'] = None
        
        # Verificar IMU
        try:
            imu_msg = rospy.wait_for_message('/imu', Imu, timeout=timeout)
            sensor_data['imu'] = {
                'orientation': [imu_msg.orientation.x, imu_msg.orientation.y, 
                               imu_msg.orientation.z, imu_msg.orientation.w],
                'angular_velocity': [imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, 
                                   imu_msg.angular_velocity.z],
                'timestamp': imu_msg.header.stamp.to_sec()
            }
            rospy.loginfo("✓ Datos de IMU recibidos")
        except rospy.ROSException:
            rospy.logwarn("✗ No se recibieron datos de IMU")
            sensor_data['imu'] = None
        
        # Verificar iluminancia
        try:
            illuminance_msg = rospy.wait_for_message('/ambient_light', Illuminance, timeout=timeout)
            sensor_data['illuminance'] = {
                'value': illuminance_msg.illuminance,
                'timestamp': illuminance_msg.header.stamp.to_sec()
            }
            rospy.loginfo("✓ Datos de iluminancia recibidos")
        except rospy.ROSException:
            rospy.logwarn("✗ No se recibieron datos de iluminancia")
            sensor_data['illuminance'] = None
        
        self.diagnostic_results['sensor_data'] = sensor_data
        return len([s for s in sensor_data.values() if s is not None]) > 0
    
    def check_battery_status(self):
        """Verifica el estado de la batería"""
        rospy.loginfo("Verificando estado de batería...")
        
        try:
            battery_srv = rospy.ServiceProxy('/battery_state', BatteryState)
            response = battery_srv()
            
            self.diagnostic_results['battery_status'] = {
                'percentage': response.battery_percentage,
                'voltage_state': response.voltage_state,
                'timestamp': time.time()
            }
            
            rospy.loginfo(f"✓ Batería: {response.battery_percentage}% - Estado: {response.voltage_state}")
            
            # Verificar si la batería está baja
            if response.battery_percentage < 20:
                self.diagnostic_results['recommendations'].append("Batería baja - considerar cargar el robot")
            
            return True
            
        except rospy.ServiceException as e:
            rospy.logerr(f"✗ Error obteniendo estado de batería: {e}")
            return False
    
    def check_emergency_stop_status(self):
        """Verifica el estado de la parada de emergencia"""
        rospy.loginfo("Verificando estado de parada de emergencia...")
        
        try:
            # Verificar parámetro de parada de emergencia
            emergency_stop = rospy.get_param('/emergency_stop', False)
            self.diagnostic_results['emergency_stop_status'] = emergency_stop
            
            if emergency_stop:
                rospy.logwarn("⚠ Parada de emergencia activa")
                self.diagnostic_results['recommendations'].append("Liberar parada de emergencia para permitir movimiento")
            else:
                rospy.loginfo("✓ Parada de emergencia inactiva")
            
            return True
            
        except Exception as e:
            rospy.logerr(f"✗ Error verificando parada de emergencia: {e}")
            return False
    
    def test_basic_movement(self):
        """Prueba el movimiento básico del robot"""
        rospy.loginfo("Probando movimiento básico...")
        
        try:
            # Crear publisher
            cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
            rospy.sleep(1.0)  # Esperar a que el publisher se conecte
            
            # Enviar comando de movimiento suave
            twist = Twist()
            twist.linear.x = 0.1
            twist.angular.z = 0.0
            cmd_vel_pub.publish(twist)
            rospy.sleep(1.0)
            
            # Parar
            twist.linear.x = 0.0
            cmd_vel_pub.publish(twist)
            
            rospy.loginfo("✓ Movimiento básico probado")
            return True
            
        except Exception as e:
            rospy.logerr(f"✗ Error probando movimiento: {e}")
            return False
    
    def generate_report(self):
        """Genera un reporte de diagnóstico"""
        rospy.loginfo("=== REPORTE DE DIAGNÓSTICO ===")
        
        # Resumen general
        total_checks = 6
        passed_checks = 0
        
        if self.diagnostic_results['ros_master']:
            passed_checks += 1
        if self.diagnostic_results['driver_running']:
            passed_checks += 1
        if all(self.diagnostic_results['topics_available'].values()):
            passed_checks += 1
        if all(self.diagnostic_results['services_available'].values()):
            passed_checks += 1
        if len([s for s in self.diagnostic_results['sensor_data'].values() if s is not None]) > 0:
            passed_checks += 1
        if self.diagnostic_results['battery_status']:
            passed_checks += 1
        
        rospy.loginfo(f"Checks pasados: {passed_checks}/{total_checks}")
        rospy.loginfo(f"Porcentaje de salud: {(passed_checks/total_checks)*100:.1f}%")
        
        # Mostrar recomendaciones
        if self.diagnostic_results['recommendations']:
            rospy.loginfo("\nRecomendaciones:")
            for i, rec in enumerate(self.diagnostic_results['recommendations'], 1):
                rospy.loginfo(f"  {i}. {rec}")
        
        # Guardar reporte
        try:
            with open('/tmp/atriz_rvr_diagnostic_report.json', 'w') as f:
                json.dump(self.diagnostic_results, f, indent=2, default=str)
            rospy.loginfo("Reporte guardado en /tmp/atriz_rvr_diagnostic_report.json")
        except Exception as e:
            rospy.logerr(f"Error guardando reporte: {e}")
    
    def run_full_diagnostic(self):
        """Ejecuta el diagnóstico completo"""
        rospy.loginfo("🔍 Iniciando diagnóstico completo del sistema Atriz RVR...")
        rospy.loginfo("=" * 60)
        
        # Verificar ROS master
        rospy.loginfo("1️⃣ Verificando master de ROS...")
        self.check_ros_master()
        rospy.sleep(1.0)
        
        # Verificar driver
        rospy.loginfo("2️⃣ Verificando driver Atriz RVR...")
        self.check_driver_running()
        rospy.sleep(1.0)
        
        # Verificar tópicos
        rospy.loginfo("3️⃣ Verificando tópicos...")
        self.check_topics()
        rospy.sleep(1.0)
        
        # Verificar servicios
        rospy.loginfo("4️⃣ Verificando servicios...")
        self.check_services()
        rospy.sleep(1.0)
        
        # Solo verificar datos de sensores si el driver está ejecutándose
        if self.diagnostic_results['driver_running']:
            rospy.loginfo("5️⃣ Verificando datos de sensores...")
            self.check_sensor_data()
            rospy.sleep(1.0)
            
            rospy.loginfo("6️⃣ Verificando batería...")
            self.check_battery_status()
            rospy.sleep(1.0)
            
            rospy.loginfo("7️⃣ Verificando parada de emergencia...")
            self.check_emergency_stop_status()
            rospy.sleep(1.0)
            
            rospy.loginfo("8️⃣ Probando movimiento básico...")
            self.test_basic_movement()
        else:
            rospy.loginfo("ℹ Saltando verificación de sensores (driver no ejecutándose)")
        
        # Generar reporte
        rospy.loginfo("9️⃣ Generando reporte...")
        self.generate_report()
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("✅ Diagnóstico completo finalizado")

def main():
    """Función principal"""
    try:
        diagnostic = SystemDiagnostic()
        diagnostic.run_full_diagnostic()
    except rospy.ROSInterruptException:
        rospy.loginfo("Diagnóstico interrumpido")
    except Exception as e:
        rospy.logerr(f"Error en el diagnóstico: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()
