#!/usr/bin/env python3

import rospy
import time
import threading
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from atriz_rvr_msgs.srv import SetLEDRGB, SetLEDRGBRequest, TriggerLedEvent, TriggerLedEventRequest
from std_msgs.msg import Bool

class RVRJoystickController:
    def __init__(self):
        # Variables globales para almacenar el estado actual del joystick
        self.current_twist = Twist()
        self.last_button_states = [False] * 12  # Para detectar cambios de botones
        self.led_color_index = 0
        self.led_colors = [
            (255, 0, 0),    # Rojo
            (0, 255, 0),    # Verde
            (0, 0, 255),    # Azul
            (255, 255, 0),  # Amarillo
            (255, 0, 255),  # Magenta
            (0, 255, 255),  # Cian
            (255, 255, 255), # Blanco
            (0, 0, 0)       # Apagado
        ]
        self.emergency_stop_active = False
        self.led_pattern_active = False
        self.last_button_time = 0
        self.button_debounce_time = 0.2  # 200ms debounce
        self.blink_timer = None
        self.blink_state = False
        self.blink_thread = None
        self.blink_running = False
        
        # Inicializar ROS
        rospy.init_node('rvr_joystick_control')
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.emergency_stop_pub = rospy.Publisher('/emergency_stop', Bool, queue_size=10)
        
        # Subscriber
        rospy.Subscriber("/joy", Joy, self.joystick_callback)
        
        # Configurar un temporizador para publicar los comandos periódicamente
        rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        
        rospy.loginfo("Controlador de joystick RVR iniciado")
        rospy.loginfo("Controles:")
        rospy.loginfo("  - Eje izquierdo: Girar")
        rospy.loginfo("  - RT: Acelerar adelante")
        rospy.loginfo("  - LT: Ir hacia atrás")
        rospy.loginfo("  - A: Parada de emergencia")
        rospy.loginfo("  - B: Liberar parada de emergencia")
        rospy.loginfo("  - X: Cambiar color de LEDs")
        rospy.loginfo("  - Y: Patrón de LEDs")
        rospy.loginfo("  - START: Patrón de inicio")
        rospy.loginfo("  - SELECT: Patrón de error")
        rospy.loginfo("  - LB: Apagar todos los LEDs")
        rospy.loginfo("  - RB: Encender todos los LEDs en blanco")

    def joystick_callback(self, data):
        # Detectar cambios en botones (solo cuando se presionan, no cuando se mantienen)
        button_changes = []
        for i in range(len(data.buttons)):
            if data.buttons[i] and not self.last_button_states[i]:
                button_changes.append(i)
            self.last_button_states[i] = data.buttons[i]
        
        # Procesar cambios de botones
        for button in button_changes:
            self.handle_button_press(button)
        
        # Procesar movimiento solo si no hay parada de emergencia
        if not self.emergency_stop_active:
            self.handle_movement(data)

    def handle_button_press(self, button):
        """Maneja las pulsaciones de botones"""
        current_time = time.time()
        
        # Debounce: evitar activaciones múltiples muy rápidas
        if current_time - self.last_button_time < self.button_debounce_time:
            return
            
        self.last_button_time = current_time
        
        if button == 0:  # A - Parada de emergencia
            self.activate_emergency_stop()
        elif button == 1:  # B - Liberar parada de emergencia
            self.release_emergency_stop()
        elif button == 2:  # X - Cambiar color de LEDs
            self.change_led_color()
        elif button == 3:  # Y - Patrón de LEDs
            self.toggle_led_pattern()
        elif button == 6:  # START - Patrón de inicio
            # Detener patrón de parpadeo si está activo
            if self.led_pattern_active:
                self.stop_blink_pattern()
            self.set_all_leds(0, 255, 0)  # Verde para inicio
            rospy.loginfo("Patrón de inicio activado")
        elif button == 7:  # SELECT - Patrón de error
            # Detener patrón de parpadeo si está activo
            if self.led_pattern_active:
                self.stop_blink_pattern()
            self.set_all_leds(255, 0, 0)  # Rojo para error
            rospy.loginfo("Patrón de error activado")
        elif button == 4:  # LB - Apagar LEDs
            # Detener patrón de parpadeo si está activo
            if self.led_pattern_active:
                self.stop_blink_pattern()
            self.set_all_leds(0, 0, 0)  # Apagar todos
        elif button == 5:  # RB - Encender LEDs blancos
            # Detener patrón de parpadeo si está activo
            if self.led_pattern_active:
                self.stop_blink_pattern()
            self.set_all_leds(255, 255, 255)  # Blanco

    def handle_movement(self, data):
        """Maneja el movimiento del robot"""
        # Mapeo del joystick
        x_axis = data.axes[0]  # Eje izquierdo-derecha
        rt = data.axes[5]  # RT para acelerar
        lt = data.axes[2]  # LT para frenar

        # Ajustar la velocidad de avance y reversa
        forward_speed = (1/4) * (1 - rt)
        reverse_speed = (1/8) * (lt - 1)
        
        # Ajustar la rotación con una escala más suave
        angular_scale = 2.0  # Factor de escala para suavizar el giro
        
        if rt < 1:
            if lt < 1:
                self.current_twist.linear.x = forward_speed + reverse_speed
                self.current_twist.angular.z = -x_axis * angular_scale
            else:
                self.current_twist.linear.x = forward_speed
                self.current_twist.angular.z = x_axis * angular_scale
        elif lt < 1:
            self.current_twist.linear.x = reverse_speed
            self.current_twist.angular.z = -x_axis * angular_scale
        elif rt == 1 and lt == 1:
            self.current_twist.linear.x = 0
            self.current_twist.angular.z = x_axis * angular_scale

    def activate_emergency_stop(self):
        """Activa la parada de emergencia"""
        self.emergency_stop_active = True
        self.current_twist = Twist()  # Parar movimiento
        self.emergency_stop_pub.publish(Bool(True))
        # Parpadeo rojo para emergencia
        self.set_all_leds(255, 0, 0)  # Rojo
        rospy.logwarn("PARADA DE EMERGENCIA ACTIVADA")

    def release_emergency_stop(self):
        """Libera la parada de emergencia"""
        self.emergency_stop_active = False
        self.emergency_stop_pub.publish(Bool(False))
        rospy.loginfo("Parada de emergencia liberada")

    def change_led_color(self):
        """Cambia el color de los LEDs"""
        # Detener patrón de parpadeo si está activo
        if self.led_pattern_active:
            self.stop_blink_pattern()
            
        self.led_color_index = (self.led_color_index + 1) % len(self.led_colors)
        color = self.led_colors[self.led_color_index]
        self.set_all_leds(color[0], color[1], color[2])
        
        # Nombres de colores para mejor logging
        color_names = ["Rojo", "Verde", "Azul", "Amarillo", "Magenta", "Cian", "Blanco", "Apagado"]
        color_name = color_names[self.led_color_index]
        rospy.loginfo(f"Color de LED cambiado a: {color_name} {color}")

    def toggle_led_pattern(self):
        """Alterna el patrón de LEDs (efecto de parpadeo)"""
        if self.led_pattern_active:
            # Detener patrón - apagar LEDs y timer
            self.stop_blink_pattern()
            rospy.loginfo("Patrón de LEDs detenido")
        else:
            # Activar patrón de parpadeo
            self.start_blink_pattern()
            rospy.loginfo("Patrón de LEDs activado - parpadeo")

    def stop_blink_pattern(self):
        """Detiene el patrón de parpadeo"""
        self.blink_running = False
        self.led_pattern_active = False
        
        # Detener timer si existe
        if self.blink_timer:
            self.blink_timer.shutdown()
            self.blink_timer = None
            
        # Esperar a que el hilo termine
        if self.blink_thread and self.blink_thread.is_alive():
            self.blink_thread.join(timeout=1.0)
            
        self.set_all_leds(0, 0, 0)

    def start_blink_pattern(self):
        """Inicia el patrón de parpadeo"""
        # Detener patrón anterior si existe
        self.stop_blink_pattern()
        
        self.led_pattern_active = True
        self.blink_running = True
        self.blink_state = False
        
        # Iniciar hilo de parpadeo
        self.blink_thread = threading.Thread(target=self.blink_worker, daemon=True)
        self.blink_thread.start()

    def blink_worker(self):
        """Worker thread para el parpadeo"""
        while self.blink_running and self.led_pattern_active:
            try:
                self.blink_state = not self.blink_state
                if self.blink_state:
                    # Encender en azul
                    self.set_all_leds(0, 0, 255)
                else:
                    # Apagar
                    self.set_all_leds(0, 0, 0)
                    
                time.sleep(0.5)  # Esperar 0.5 segundos
            except Exception as e:
                rospy.logwarn(f"Error en worker de parpadeo: {e}")
                break

    def set_leds(self, led_id, red, green, blue):
        """Controla los LEDs del robot"""
        try:
            set_leds_srv = rospy.ServiceProxy('/set_led_rgb', SetLEDRGB)
            
            request = SetLEDRGBRequest()
            request.led_id = led_id
            request.red = red
            request.green = green
            request.blue = blue
            
            response = set_leds_srv(request)
            
            if not response.success:
                rospy.logwarn(f"Error controlando LEDs: {response.message}")
            else:
                rospy.logdebug(f"LEDs actualizados: ID={led_id}, RGB=({red},{green},{blue})")
                
        except rospy.ServiceException as e:
            rospy.logwarn(f"Fallo en la llamada al servicio de LEDs: {e}")
        except Exception as e:
            rospy.logwarn(f"Error inesperado controlando LEDs: {e}")

    def set_all_leds(self, red, green, blue):
        """Controla todos los LEDs principales del robot"""
        # Lista de LEDs principales para controlar
        main_leds = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]  # Todos los LEDs principales
        
        for led_id in main_leds:
            self.set_leds(led_id, red, green, blue)


    def trigger_led_event(self, event_id, stop_current=False):
        """Activa un evento de LED"""
        try:
            trigger_srv = rospy.ServiceProxy('/trigger_led_event', TriggerLedEvent)
            request = TriggerLedEventRequest()
            request.stop_current_event = stop_current
            request.event_id = event_id
            response = trigger_srv(request)
            if not response.success:
                rospy.logwarn(f"Error activando evento de LED (ID: {event_id})")
        except rospy.ServiceException as e:
            rospy.logerr(f"Fallo en la llamada al servicio de eventos: {e}")

    def timer_callback(self, event):
        """Publica comandos de movimiento periódicamente"""
        self.cmd_vel_pub.publish(self.current_twist)

    def cleanup(self):
        """Limpia recursos al cerrar"""
        self.blink_running = False
        self.led_pattern_active = False
        
        # Detener timer si existe
        if self.blink_timer:
            self.blink_timer.shutdown()
            self.blink_timer = None
            
        # Esperar a que el hilo termine
        if self.blink_thread and self.blink_thread.is_alive():
            self.blink_thread.join(timeout=1.0)

    def run(self):
        """Ejecuta el controlador"""
        try:
            rospy.spin()
        finally:
            self.cleanup()

if __name__ == '__main__':
    try:
        controller = RVRJoystickController()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Controlador de joystick detenido")
    except KeyboardInterrupt:
        rospy.loginfo("Controlador de joystick detenido por usuario")
    finally:
        if 'controller' in locals():
            controller.cleanup()

