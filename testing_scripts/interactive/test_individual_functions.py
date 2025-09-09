#!/usr/bin/env python3

"""
Script para probar funcionalidades individuales del driver Atriz_rvr_node
Permite probar cada función por separado para debugging detallado
"""

import rospy
import time
import json
from datetime import datetime
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Illuminance
from std_msgs.msg import String, Bool, Empty
from sphero_rvr_msgs.msg import Color, DegreesTwist
from sphero_rvr_msgs.srv import SetIRMode, BatteryState
from std_srvs.srv import SetBool, Empty as EmptySrv

class IndividualFunctionTester:
    """Clase para probar funciones individuales del driver"""
    
    def __init__(self):
        """Inicializa el tester individual"""
        rospy.init_node('individual_function_tester', anonymous=True)
        
        # Variables para almacenar datos
        self.received_data = {}
        self.test_results = {}
        
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
        
        rospy.loginfo("Individual Function Tester inicializado")
    
    def odom_callback(self, data):
        """Callback para odometría"""
        self.received_data['odom'] = data
        # Solo log debug para no interferir con el menú
        rospy.logdebug(f"Odometría: Pos=({data.pose.pose.position.x:.3f}, {data.pose.pose.position.y:.3f}), Vel=({data.twist.twist.linear.x:.3f}, {data.twist.twist.angular.z:.3f})")
    
    def imu_callback(self, data):
        """Callback para IMU"""
        self.received_data['imu'] = data
        # Solo log debug para no interferir con el menú
        rospy.logdebug(f"IMU: Orientación=({data.orientation.x:.3f}, {data.orientation.y:.3f}, {data.orientation.z:.3f}, {data.orientation.w:.3f})")
    
    def color_callback(self, data):
        """Callback para color"""
        self.received_data['color'] = data
        # Solo log debug para no interferir con el menú
        rospy.logdebug(f"Color: RGB={data.rgb_color}, Confianza={data.confidence}")
    
    def illuminance_callback(self, data):
        """Callback para iluminancia"""
        self.received_data['illuminance'] = data
        # Solo log debug para no interferir con el menú
        rospy.logdebug(f"Iluminancia: {data.illuminance:.3f}")
    
    def ir_messages_callback(self, data):
        """Callback para mensajes IR"""
        self.received_data['ir_messages'] = data
        # Solo log debug para no interferir con el menú
        rospy.logdebug(f"Mensaje IR: {data.data}")
    
    def test_cmd_vel_forward(self):
        """Prueba movimiento hacia adelante con cmd_vel"""
        rospy.loginfo("=== PROBANDO MOVIMIENTO HACIA ADELANTE (cmd_vel) ===")
        
        twist = Twist()
        twist.linear.x = 0.2
        twist.angular.z = 0.0
        
        rospy.loginfo("Enviando comando: linear.x=0.2, angular.z=0.0")
        self.cmd_vel_pub.publish(twist)
        
        rospy.sleep(3.0)
        
        # Parar
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo("Movimiento hacia adelante completado")
    
    def test_cmd_vel_backward(self):
        """Prueba movimiento hacia atrás con cmd_vel"""
        rospy.loginfo("=== PROBANDO MOVIMIENTO HACIA ATRÁS (cmd_vel) ===")
        
        twist = Twist()
        twist.linear.x = -0.2
        twist.angular.z = 0.0
        
        rospy.loginfo("Enviando comando: linear.x=-0.2, angular.z=0.0")
        self.cmd_vel_pub.publish(twist)
        
        rospy.sleep(3.0)
        
        # Parar
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo("Movimiento hacia atrás completado")
    
    def test_cmd_vel_turn_left(self):
        """Prueba giro a la izquierda con cmd_vel"""
        rospy.loginfo("=== PROBANDO GIRO A LA IZQUIERDA (cmd_vel) ===")
        
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.5
        
        rospy.loginfo("Enviando comando: linear.x=0.0, angular.z=0.5")
        self.cmd_vel_pub.publish(twist)
        
        rospy.sleep(3.0)
        
        # Parar
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo("Giro a la izquierda completado")
    
    def test_cmd_vel_turn_right(self):
        """Prueba giro a la derecha con cmd_vel"""
        rospy.loginfo("=== PROBANDO GIRO A LA DERECHA (cmd_vel) ===")
        
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = -0.5
        
        rospy.loginfo("Enviando comando: linear.x=0.0, angular.z=-0.5")
        self.cmd_vel_pub.publish(twist)
        
        rospy.sleep(3.0)
        
        # Parar
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo("Giro a la derecha completado")
    
    def test_cmd_degrees_forward(self):
        """Prueba movimiento hacia adelante con cmd_degrees"""
        rospy.loginfo("=== PROBANDO MOVIMIENTO HACIA ADELANTE (cmd_degrees) ===")
        
        degrees_twist = DegreesTwist()
        degrees_twist.linear_x = 0.2
        degrees_twist.angular_z = 0.0
        
        rospy.loginfo("Enviando comando: linear_x=0.2, angular_z=0.0")
        self.cmd_degrees_pub.publish(degrees_twist)
        
        rospy.sleep(3.0)
        
        # Parar
        degrees_twist.linear_x = 0.0
        self.cmd_degrees_pub.publish(degrees_twist)
        rospy.loginfo("Movimiento hacia adelante (grados) completado")
    
    def test_cmd_degrees_turn(self):
        """Prueba giro con cmd_degrees"""
        rospy.loginfo("=== PROBANDO GIRO (cmd_degrees) ===")
        
        degrees_twist = DegreesTwist()
        degrees_twist.linear_x = 0.0
        degrees_twist.angular_z = 30.0
        
        rospy.loginfo("Enviando comando: linear_x=0.0, angular_z=30.0")
        self.cmd_degrees_pub.publish(degrees_twist)
        
        rospy.sleep(3.0)
        
        # Parar
        degrees_twist.angular_z = 0.0
        self.cmd_degrees_pub.publish(degrees_twist)
        rospy.loginfo("Giro (grados) completado")
    
    def test_emergency_stop(self):
        """Prueba el sistema de parada de emergencia"""
        rospy.loginfo("=== PROBANDO PARADA DE EMERGENCIA ===")
        
        # Iniciar movimiento
        twist = Twist()
        twist.linear.x = 0.3
        twist.angular.z = 0.2
        rospy.loginfo("Iniciando movimiento...")
        self.cmd_vel_pub.publish(twist)
        rospy.sleep(1.0)
        
        # Activar parada de emergencia
        rospy.loginfo("Activando parada de emergencia...")
        self.emergency_stop_pub.publish(Empty())
        rospy.sleep(2.0)
        
        # Intentar enviar comando (debería ser ignorado)
        rospy.loginfo("Intentando enviar comando durante parada de emergencia...")
        twist.linear.x = 0.5
        self.cmd_vel_pub.publish(twist)
        rospy.sleep(1.0)
        
        # Liberar parada de emergencia
        rospy.loginfo("Liberando parada de emergencia...")
        try:
            self.release_emergency_stop_srv()
            rospy.loginfo("Parada de emergencia liberada exitosamente")
        except rospy.ServiceException as e:
            rospy.logerr(f"Error liberando parada de emergencia: {e}")
        
        rospy.sleep(1.0)
        rospy.loginfo("Prueba de parada de emergencia completada")
    
    def test_battery_state(self):
        """Prueba el servicio de estado de batería"""
        rospy.loginfo("=== PROBANDO ESTADO DE BATERÍA ===")
        
        try:
            response = self.battery_state_srv()
            rospy.loginfo(f"Batería: {response.battery_percentage}%")
            rospy.loginfo(f"Estado de voltaje: {response.voltage_state}")
            return True
        except rospy.ServiceException as e:
            rospy.logerr(f"Error obteniendo estado de batería: {e}")
            return False
    
    def test_enable_color(self):
        """Prueba el servicio de habilitación de color"""
        rospy.loginfo("=== PROBANDO HABILITACIÓN DE COLOR ===")
        
        # Habilitar
        try:
            response = self.enable_color_srv(True)
            rospy.loginfo(f"Habilitar color: {response}")
            rospy.sleep(2.0)
        except rospy.ServiceException as e:
            rospy.logerr(f"Error habilitando color: {e}")
            return False
        
        # Deshabilitar
        try:
            response = self.enable_color_srv(False)
            rospy.loginfo(f"Deshabilitar color: {response}")
            return True
        except rospy.ServiceException as e:
            rospy.logerr(f"Error deshabilitando color: {e}")
            return False
    
    def test_reset_odom(self):
        """Prueba el servicio de reinicio de odometría"""
        rospy.loginfo("=== PROBANDO REINICIO DE ODOMETRÍA ===")
        
        try:
            response = self.reset_odom_srv()
            rospy.loginfo(f"Reinicio de odometría: {response}")
            return True
        except rospy.ServiceException as e:
            rospy.logerr(f"Error reiniciando odometría: {e}")
            return False
    
    def test_ir_broadcast(self):
        """Prueba el modo IR broadcast"""
        rospy.loginfo("=== PROBANDO IR BROADCAST ===")
        
        try:
            response = self.ir_mode_srv('broadcast', 1, 2)
            rospy.loginfo(f"IR Broadcast: {response}")
            rospy.sleep(3.0)
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Error en IR Broadcast: {e}")
            return False
    
    def test_ir_following(self):
        """Prueba el modo IR following"""
        rospy.loginfo("=== PROBANDO IR FOLLOWING ===")
        
        try:
            response = self.ir_mode_srv('following', 3, 4)
            rospy.loginfo(f"IR Following: {response}")
            rospy.sleep(3.0)
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Error en IR Following: {e}")
            return False
    
    def test_ir_off(self):
        """Prueba el modo IR off"""
        rospy.loginfo("=== PROBANDO IR OFF ===")
        
        try:
            response = self.ir_mode_srv('off', 0, 0)
            rospy.loginfo(f"IR Off: {response}")
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Error en IR Off: {e}")
            return False
    
    def test_sensor_data_streaming(self):
        """Prueba el streaming de datos de sensores"""
        rospy.loginfo("=== PROBANDO STREAMING DE SENSORES ===")
        
        rospy.loginfo("Esperando datos de sensores por 10 segundos...")
        start_time = time.time()
        
        while time.time() - start_time < 10.0:
            if 'odom' in self.received_data:
                odom = self.received_data['odom']
                rospy.loginfo(f"Odometría: Pos=({odom.pose.pose.position.x:.3f}, {odom.pose.pose.position.y:.3f})")
            
            if 'imu' in self.received_data:
                imu = self.received_data['imu']
                rospy.loginfo(f"IMU: Orientación=({imu.orientation.x:.3f}, {imu.orientation.y:.3f}, {imu.orientation.z:.3f}, {imu.orientation.w:.3f})")
            
            if 'illuminance' in self.received_data:
                illuminance = self.received_data['illuminance']
                rospy.loginfo(f"Iluminancia: {illuminance.illuminance:.3f}")
            
            if 'color' in self.received_data:
                color = self.received_data['color']
                rospy.loginfo(f"Color: RGB={color.rgb_color}, Confianza={color.confidence}")
            
            rospy.sleep(1.0)
        
        rospy.loginfo("Streaming de sensores completado")
    
    def run_interactive_menu(self):
        """Ejecuta un menú interactivo para probar funciones individuales"""
        while not rospy.is_shutdown():
            # Limpiar pantalla y mostrar menú
            print("\n" + "="*50)
            print("    MENÚ DE PRUEBAS INDIVIDUALES")
            print("="*50)
            print("1.  Movimiento hacia adelante (cmd_vel)")
            print("2.  Movimiento hacia atrás (cmd_vel)")
            print("3.  Giro a la izquierda (cmd_vel)")
            print("4.  Giro a la derecha (cmd_vel)")
            print("5.  Movimiento hacia adelante (cmd_degrees)")
            print("6.  Giro (cmd_degrees)")
            print("7.  Parada de emergencia")
            print("8.  Estado de batería")
            print("9.  Habilitar/Deshabilitar color")
            print("10. Reiniciar odometría")
            print("11. IR Broadcast")
            print("12. IR Following")
            print("13. IR Off")
            print("14. Streaming de sensores")
            print("0.  Salir")
            print("="*50)
            
            try:
                # Usar raw_input para mejor compatibilidad
                choice = input("\nSeleccione una opción (0-14): ").strip()
                
                if choice == '0':
                    print("\nSaliendo del programa...")
                    break
                elif choice == '1':
                    self.test_cmd_vel_forward()
                elif choice == '2':
                    self.test_cmd_vel_backward()
                elif choice == '3':
                    self.test_cmd_vel_turn_left()
                elif choice == '4':
                    self.test_cmd_vel_turn_right()
                elif choice == '5':
                    self.test_cmd_degrees_forward()
                elif choice == '6':
                    self.test_cmd_degrees_turn()
                elif choice == '7':
                    self.test_emergency_stop()
                elif choice == '8':
                    self.test_battery_state()
                elif choice == '9':
                    self.test_enable_color()
                elif choice == '10':
                    self.test_reset_odom()
                elif choice == '11':
                    self.test_ir_broadcast()
                elif choice == '12':
                    self.test_ir_following()
                elif choice == '13':
                    self.test_ir_off()
                elif choice == '14':
                    self.test_sensor_data_streaming()
                else:
                    print(f"\n❌ Opción inválida: '{choice}'. Por favor seleccione 0-14.")
                
                # Pausa antes de mostrar el menú nuevamente
                input("\n⏸️  Presione Enter para continuar...")
                
            except KeyboardInterrupt:
                print("\n\n⚠️  Interrupción detectada. Saliendo...")
                break
            except EOFError:
                print("\n\n⚠️  Fin de entrada. Saliendo...")
                break
            except Exception as e:
                print(f"\n❌ Error en el menú: {e}")
                input("Presione Enter para continuar...")
        
        print("✅ Saliendo del menú de pruebas")

def main():
    """Función principal"""
    try:
        tester = IndividualFunctionTester()
        
        # Esperar a que el driver esté listo
        rospy.loginfo("Esperando a que el driver esté listo...")
        rospy.sleep(5.0)
        
        # Ejecutar menú interactivo
        tester.run_interactive_menu()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Programa interrumpido")
    except Exception as e:
        rospy.logerr(f"Error en el programa principal: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()
