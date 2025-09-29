#!/usr/bin/env python3

import random
import time
import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
from atriz_rvr_msgs.msg import Color

# Variables globales
color_enabled = False
detected_color = None
pub_color = None

# Inicializamos el nodo de ROS
rospy.init_node('rvr_random_walk', anonymous=True)

# Publicador para enviar comandos de velocidad
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

# Servicio para habilitar el color
rospy.wait_for_service('/enable_color')
try:
    enable_color_service = rospy.ServiceProxy('/enable_color', SetBool)
    enable_color_service(True)
    rospy.loginfo("Color sensor enabled")
    color_enabled = True
except rospy.ServiceException as e:
    rospy.logerr("Service call failed: %s", e)

# Parámetros para ajustar el movimiento aleatorio
LINEAR_VEL_MIN = 0.1  # Velocidad lineal mínima
LINEAR_VEL_MAX = 0.5  # Velocidad lineal máxima
ANGULAR_VEL_MIN = 0.5  # Velocidad angular mínima
ANGULAR_VEL_MAX = 2.0  # Velocidad angular máxima
TURN_CHANCE = 0.2      # Probabilidad de realizar un giro en cada ciclo (20%)

# Función para leer el color del tópico y verificar condiciones
def check_color_and_respond():
    global detected_color, color_enabled
    if not color_enabled:
        # Si el sensor de color no está habilitado, se sale de la función
        return

    try:
        color_data = rospy.wait_for_message('/color', Color, timeout=1.0)
        detected_color = [color_data.rgb_color[0], color_data.rgb_color[1], color_data.rgb_color[2]]
        confidence = color_data.confidence

        # Log del color detectado
        rospy.logdebug(f"🎨 Color detectado: R={detected_color[0]}, G={detected_color[1]}, B={detected_color[2]}, Confianza: {confidence:.2f}")

        # Verificamos si los valores están por debajo del umbral
        if detected_color[0] < 100 and detected_color[1] < 100 and detected_color[2] < 100:
            rospy.logwarn(f"🚧 OBSTÁCULO DETECTADO - Color oscuro: R={detected_color[0]}, G={detected_color[1]}, B={detected_color[2]} (umbral: <100)")
            avoid_obstacle()
    except rospy.ROSException as e:
        rospy.logwarn("⚠️ Error recibiendo datos de color: %s", e)

# Función para retroceder y girar el robot
def avoid_obstacle():
    rospy.loginfo("🚧 OBSTÁCULO DETECTADO - Iniciando maniobra de evasión...")
    
    # Creamos un mensaje Twist para retroceder y luego girar 90 grados
    twist = Twist()

    # Retrocedemos por 1 segundo
    rospy.loginfo("⬅️ Fase 1: RETROCESO - Velocidad: -0.3 m/s por 1 segundo")
    twist.linear.x = -0.3  # Velocidad de retroceso
    twist.angular.z = 0.0
    cmd_vel_pub.publish(twist)
    time.sleep(1)

    # Luego giramos 90 grados (2π radianes)
    angular_vel = 2 * 3.14159  # 2π rad/s
    angular_deg_s = angular_vel * (180.0 / 3.14159)  # Convertir a grados/s
    rospy.loginfo(f"🔄 Fase 2: GIRO - Velocidad angular: {angular_vel:.3f} rad/s ({angular_deg_s:.1f}°/s) por 2 segundos")
    twist.linear.x = 0.0
    twist.angular.z = angular_vel
    cmd_vel_pub.publish(twist)
    time.sleep(2)  # Giramos por 2 segundos

    # Detenemos el robot
    rospy.loginfo("⏹️ Fase 3: DETENCIÓN - Robot detenido")
    twist.angular.z = 0.0
    cmd_vel_pub.publish(twist)
    
    rospy.loginfo("✅ Maniobra de evasión completada - Reanudando movimiento aleatorio")

# Función para el movimiento aleatorio del robot
def random_walk():
    rate = rospy.Rate(10)  # Frecuencia de 10 Hz para la ejecución del ciclo
    movement_count = 0  # Contador de movimientos
    
    rospy.loginfo("🚀 Iniciando caminata aleatoria...")
    rospy.loginfo(f"📊 Parámetros: Velocidad lineal {LINEAR_VEL_MIN}-{LINEAR_VEL_MAX} m/s, Angular {ANGULAR_VEL_MIN}-{ANGULAR_VEL_MAX} rad/s, Probabilidad de giro: {TURN_CHANCE*100}%")
    
    while not rospy.is_shutdown():
        # Verificamos el color y respondemos si es necesario
        check_color_and_respond()

        # Creamos un mensaje Twist para enviar velocidades lineales y angulares
        twist = Twist()
        movement_count += 1
        
        # Determinamos aleatoriamente si el robot debe girar o moverse hacia adelante
        if random.random() < TURN_CHANCE:
            # Generamos un giro aleatorio
            angular_vel = random.uniform(ANGULAR_VEL_MIN, ANGULAR_VEL_MAX)
            twist.angular.z = angular_vel
            twist.linear.x = 0.2  # Detenemos el movimiento lineal al girar
            
            # Convertir rad/s a grados/s para el log
            angular_deg_s = angular_vel * (180.0 / 3.14159)
            
            rospy.loginfo(f"🔄 Movimiento #{movement_count}: GIRO - Velocidad angular: {angular_vel:.3f} rad/s ({angular_deg_s:.1f}°/s), Lineal: {twist.linear.x:.1f} m/s")
        else:
            # Movemos el robot hacia adelante con una velocidad aleatoria
            linear_vel = 0.3  # Usando valor fijo como en el original
            twist.linear.x = linear_vel
            twist.angular.z = 0.0
            
            rospy.loginfo(f"➡️ Movimiento #{movement_count}: ADELANTE - Velocidad lineal: {linear_vel:.1f} m/s, Angular: {twist.angular.z:.1f} rad/s")
        
        # Publicamos el mensaje Twist
        cmd_vel_pub.publish(twist)
        
        # Pausa hasta el siguiente ciclo
        rate.sleep()

# Nodo principal
if __name__ == '__main__':
    try:
        rospy.loginfo("=" * 60)
        rospy.loginfo("🤖 SPHERO RVR RANDOM WALKER INICIADO")
        rospy.loginfo("=" * 60)
        rospy.loginfo(f"📊 Configuración:")
        rospy.loginfo(f"   • Velocidad lineal: {LINEAR_VEL_MIN}-{LINEAR_VEL_MAX} m/s")
        rospy.loginfo(f"   • Velocidad angular: {ANGULAR_VEL_MIN}-{ANGULAR_VEL_MAX} rad/s ({ANGULAR_VEL_MIN*180/3.14159:.1f}-{ANGULAR_VEL_MAX*180/3.14159:.1f}°/s)")
        rospy.loginfo(f"   • Probabilidad de giro: {TURN_CHANCE*100}%")
        rospy.loginfo(f"   • Frecuencia de control: 10 Hz")
        rospy.loginfo(f"   • Sensor de color: {'Habilitado' if color_enabled else 'Deshabilitado'}")
        rospy.loginfo("=" * 60)
        
        # Inicializamos el publicador para el tópico /color
        pub_color = rospy.Publisher('/color', Color, queue_size=10)
        
        # Comenzamos el movimiento aleatorio
        random_walk()
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 Interrumpido por el usuario")
    except Exception as e:
        rospy.logerr(f"❌ Error inesperado: {e}")
    finally:
        rospy.loginfo("🏁 Script finalizado")
