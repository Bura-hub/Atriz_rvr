#!/usr/bin/env python3

"""
Script interactivo para probar las funcionalidades del Sphero RVR una por una.
"""

import rospy
import time
import sys
from atriz_rvr_msgs.srv import (
    GetEncoders, RawMotors, GetSystemInfo, GetControlState, 
    SetLEDRGB, SetMultipleLEDs, SendInfraredMessage, SetIREvading,
    SetDriveParameters, ConfigureStreaming, StartStreaming
)

class RVRTester:
    def __init__(self):
        rospy.init_node('rvr_tester', anonymous=True)
        rospy.loginfo("🚀 Iniciando probador interactivo del Sphero RVR...")
        
        # Esperar a que los servicios estén disponibles
        self.wait_for_services()
        
    def wait_for_services(self):
        """Espera a que todos los servicios estén disponibles."""
        services = [
            '/get_encoders', '/raw_motors', '/get_system_info', '/get_control_state',
            '/set_led_rgb', '/set_multiple_leds', '/send_infrared_message', '/set_ir_evading',
            '/set_drive_parameters', '/configure_streaming', '/start_streaming'
        ]
        
        rospy.loginfo("Esperando servicios...")
        for service in services:
            try:
                rospy.wait_for_service(service, timeout=5)
                rospy.loginfo(f"✓ {service} disponible")
            except rospy.ROSException:
                rospy.logwarn(f"✗ {service} no disponible")
        
        rospy.loginfo("Servicios verificados!")
        time.sleep(1)
    
    def test_encoders(self):
        """Prueba la funcionalidad de encoders."""
        print("\n" + "="*50)
        print("🔧 PROBANDO ENCODERS")
        print("="*50)
        
        try:
            get_enc = rospy.ServiceProxy('/get_encoders', GetEncoders)
            response = get_enc()
            
            if response.success:
                print(f"✅ Encoders funcionando correctamente:")
                print(f"   - Rueda izquierda: {response.left_wheel_count} ticks")
                print(f"   - Rueda derecha: {response.right_wheel_count} ticks")
                return True
            else:
                print(f"❌ Error en encoders: {response.message}")
                return False
                
        except Exception as e:
            print(f"❌ Excepción en encoders: {e}")
            return False
    
    def test_raw_motors(self):
        """Prueba el control directo de motores."""
        print("\n" + "="*50)
        print("🔧 PROBANDO MOTORES RAW")
        print("="*50)
        
        try:
            raw_motors = rospy.ServiceProxy('/raw_motors', RawMotors)
            
            print("Moviendo motores hacia adelante (velocidad 50)...")
            response = raw_motors(left_mode=1, left_speed=50, right_mode=1, right_speed=50)
            
            if response.success:
                print("✅ Motores iniciados correctamente")
                time.sleep(2)
                
                print("Deteniendo motores...")
                stop_response = raw_motors(left_mode=0, left_speed=0, right_mode=0, right_speed=0)
                
                if stop_response.success:
                    print("✅ Motores detenidos correctamente")
                    return True
                else:
                    print(f"❌ Error deteniendo motores: {stop_response.message}")
                    return False
            else:
                print(f"❌ Error iniciando motores: {response.message}")
                return False
                
        except Exception as e:
            print(f"❌ Excepción en motores raw: {e}")
            return False
    
    def test_system_info(self):
        """Prueba la obtención de información del sistema."""
        print("\n" + "="*50)
        print("🔧 PROBANDO INFORMACIÓN DEL SISTEMA")
        print("="*50)
        
        try:
            get_sys = rospy.ServiceProxy('/get_system_info', GetSystemInfo)
            response = get_sys()
            
            if response.success:
                info = response.system_info
                print("✅ Información del sistema obtenida:")
                print(f"   - App: {info.app_major}.{info.app_minor}.{info.app_revision}")
                print(f"   - Bootloader: {info.bootloader_major}.{info.bootloader_minor}.{info.bootloader_revision}")
                print(f"   - Board: {info.board_revision}")
                print(f"   - MAC: {info.mac_address}")
                print(f"   - SKU: {info.sku}")
                print(f"   - Uptime: {info.uptime_ms} ms")
                print(f"   - Procesador 1: {info.processor_1_name}")
                print(f"   - Procesador 2: {info.processor_2_name}")
                return True
            else:
                print(f"❌ Error obteniendo información: {response.message}")
                return False
                
        except Exception as e:
            print(f"❌ Excepción en información del sistema: {e}")
            return False
    
    def test_control_state(self):
        """Prueba el estado del sistema de control."""
        print("\n" + "="*50)
        print("🔧 PROBANDO ESTADO DE CONTROL")
        print("="*50)
        
        try:
            get_control = rospy.ServiceProxy('/get_control_state', GetControlState)
            response = get_control()
            
            if response.success:
                state = response.control_state
                print("✅ Estado de control obtenido:")
                print(f"   - ID del controlador activo: {state.active_controller_id}")
                print(f"   - Detenido: {state.is_stopped}")
                print(f"   - Fallo de motor: {state.motor_fault}")
                print(f"   - Conduciendo: {state.is_driving}")
                return True
            else:
                print(f"❌ Error obteniendo estado: {response.message}")
                return False
                
        except Exception as e:
            print(f"❌ Excepción en estado de control: {e}")
            return False
    
    def test_led_control(self):
        """Prueba el control de LEDs."""
        print("\n" + "="*50)
        print("🔧 PROBANDO CONTROL DE LEDs")
        print("="*50)
        
        try:
            # Probar LED individual
            set_led = rospy.ServiceProxy('/set_led_rgb', SetLEDRGB)
            print("Encendiendo LED 0 en rojo...")
            response = set_led(led_id=0, red=255, green=0, blue=0)
            
            if response.success:
                print("✅ LED individual controlado correctamente")
                time.sleep(1)
                
                # Probar múltiples LEDs
                set_multiple = rospy.ServiceProxy('/set_multiple_leds', SetMultipleLEDs)
                print("Encendiendo múltiples LEDs en azul...")
                multi_response = set_multiple(
                    led_ids=[0, 1, 2], 
                    red_values=[0, 0, 0], 
                    green_values=[0, 0, 0], 
                    blue_values=[255, 255, 255]
                )
                
                if multi_response.success:
                    print("✅ Múltiples LEDs controlados correctamente")
                    time.sleep(1)
                    
                    # Apagar LEDs
                    print("Apagando LEDs...")
                    set_led(led_id=0, red=0, green=0, blue=0)
                    return True
                else:
                    print(f"❌ Error en múltiples LEDs: {multi_response.message}")
                    return False
            else:
                print(f"❌ Error en LED individual: {response.message}")
                return False
                
        except Exception as e:
            print(f"❌ Excepción en control de LEDs: {e}")
            return False
    
    def test_infrared(self):
        """Prueba las funcionalidades de infrarrojo."""
        print("\n" + "="*50)
        print("🔧 PROBANDO FUNCIONALIDADES INFRARROJAS")
        print("="*50)
        
        try:
            # Probar envío de mensaje IR
            send_ir = rospy.ServiceProxy('/send_infrared_message', SendInfraredMessage)
            print("Enviando mensaje IR...")
            response = send_ir(
                code=1, 
                front_strength=32, 
                left_strength=16, 
                right_strength=16, 
                rear_strength=8
            )
            
            if response.success:
                print("✅ Mensaje IR enviado correctamente")
                
                # Probar modo de evasión IR
                set_evading = rospy.ServiceProxy('/set_ir_evading', SetIREvading)
                print("Configurando modo de evasión IR...")
                evading_response = set_evading(far_code=2, near_code=3)
                
                if evading_response.success:
                    print("✅ Modo de evasión IR configurado correctamente")
                    return True
                else:
                    print(f"❌ Error configurando evasión IR: {evading_response.message}")
                    return False
            else:
                print(f"❌ Error enviando mensaje IR: {response.message}")
                return False
                
        except Exception as e:
            print(f"❌ Excepción en funcionalidades IR: {e}")
            return False
    
    
    def test_drive_parameters(self):
        """Prueba la configuración de parámetros de conducción."""
        print("\n" + "="*50)
        print("🔧 PROBANDO PARÁMETROS DE CONDUCCIÓN")
        print("="*50)
        
        try:
            set_params = rospy.ServiceProxy('/set_drive_parameters', SetDriveParameters)
            print("Configurando parámetros de conducción...")
            response = set_params(
                a=1.0, b=2.0, c=3.0, 
                linear_acceleration=0.5, 
                linear_velocity_slew_method=1
            )
            
            if response.success:
                print("✅ Parámetros de conducción configurados correctamente")
                return True
            else:
                print(f"❌ Error configurando parámetros: {response.message}")
                return False
                
        except Exception as e:
            print(f"❌ Excepción en parámetros de conducción: {e}")
            return False
    
    def test_streaming(self):
        """Prueba las funcionalidades de streaming."""
        print("\n" + "="*50)
        print("🔧 PROBANDO STREAMING")
        print("="*50)
        
        try:
            # Probar configuración de streaming
            configure_stream = rospy.ServiceProxy('/configure_streaming', ConfigureStreaming)
            print("Configurando streaming...")
            config_response = configure_stream(
                token=1, 
                configuration=[1, 2, 3], 
                target=1
            )
            
            if config_response.success:
                print("✅ Streaming configurado correctamente")
                
                # Probar inicio de streaming
                start_stream = rospy.ServiceProxy('/start_streaming', StartStreaming)
                print("Iniciando streaming...")
                start_response = start_stream(period=100, target=1)
                
                if start_response.success:
                    print("✅ Streaming iniciado correctamente")
                    time.sleep(1)
                    return True
                else:
                    print(f"❌ Error iniciando streaming: {start_response.message}")
                    return False
            else:
                print(f"❌ Error configurando streaming: {config_response.message}")
                return False
                
        except Exception as e:
            print(f"❌ Excepción en streaming: {e}")
            return False
    
    def run_all_tests(self):
        """Ejecuta todas las pruebas."""
        print("\n" + "="*60)
        print("🚀 INICIANDO PRUEBAS COMPLETAS DEL SPHERO RVR")
        print("="*60)
        
        tests = [
            ("Encoders", self.test_encoders),
            ("Motores Raw", self.test_raw_motors),
            ("Información del Sistema", self.test_system_info),
            ("Estado de Control", self.test_control_state),
            ("Control de LEDs", self.test_led_control),
            ("Funcionalidades IR", self.test_infrared),
            ("Parámetros de Conducción", self.test_drive_parameters),
            ("Streaming", self.test_streaming)
        ]
        
        results = {}
        
        for test_name, test_func in tests:
            try:
                result = test_func()
                results[test_name] = result
                time.sleep(1)  # Pausa entre pruebas
            except KeyboardInterrupt:
                print("\n⚠️  Pruebas interrumpidas por el usuario")
                break
            except Exception as e:
                print(f"❌ Error inesperado en {test_name}: {e}")
                results[test_name] = False
        
        # Mostrar resumen
        self.show_summary(results)
    
    def show_summary(self, results):
        """Muestra un resumen de los resultados."""
        print("\n" + "="*60)
        print("📊 RESUMEN DE PRUEBAS")
        print("="*60)
        
        passed = 0
        total = len(results)
        
        for test_name, result in results.items():
            status = "✅ PASÓ" if result else "❌ FALLÓ"
            print(f"{test_name:.<30} {status}")
            if result:
                passed += 1
        
        print("-" * 60)
        print(f"Total: {passed}/{total} pruebas pasaron")
        
        if passed == total:
            print("🎉 ¡Todas las pruebas pasaron exitosamente!")
        else:
            print("⚠️  Algunas pruebas fallaron. Revisa los mensajes de error.")

def main():
    """Función principal."""
    try:
        tester = RVRTester()
        
        print("\n¿Qué deseas hacer?")
        print("1. Ejecutar todas las pruebas")
        print("2. Probar funcionalidades individuales")
        print("3. Salir")
        
        choice = input("\nSelecciona una opción (1-3): ").strip()
        
        if choice == "1":
            tester.run_all_tests()
        elif choice == "2":
            print("\nFuncionalidades disponibles:")
            print("1. Encoders")
            print("2. Motores Raw")
            print("3. Información del Sistema")
            print("4. Estado de Control")
            print("5. Control de LEDs")
            print("6. Funcionalidades IR")
            print("7. Parámetros de Conducción")
            print("8. Streaming")
            
            test_choice = input("\nSelecciona una funcionalidad (1-8): ").strip()
            
            test_map = {
                "1": ("Encoders", tester.test_encoders),
                "2": ("Motores Raw", tester.test_raw_motors),
                "3": ("Información del Sistema", tester.test_system_info),
                "4": ("Estado de Control", tester.test_control_state),
                "5": ("Control de LEDs", tester.test_led_control),
                "6": ("Funcionalidades IR", tester.test_infrared),
                "7": ("Parámetros de Conducción", tester.test_drive_parameters),
                "8": ("Streaming", tester.test_streaming)
            }
            
            if test_choice in test_map:
                test_name, test_func = test_map[test_choice]
                print(f"\nProbando {test_name}...")
                result = test_func()
                print(f"\nResultado: {'✅ PASÓ' if result else '❌ FALLÓ'}")
            else:
                print("❌ Opción inválida")
        elif choice == "3":
            print("👋 ¡Hasta luego!")
        else:
            print("❌ Opción inválida")
            
    except KeyboardInterrupt:
        print("\n👋 Pruebas interrumpidas por el usuario")
    except Exception as e:
        print(f"❌ Error inesperado: {e}")

if __name__ == '__main__':
    main()
