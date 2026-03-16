#!/usr/bin/env python3

"""
Script de ejemplo para probar la funcionalidad de encoders del Sphero RVR.

Este script demuestra cómo:
1. Leer los valores de los encoders usando el servicio /get_encoders
2. Mover el robot usando comandos de velocidad
3. Observar cómo cambian los valores de los encoders según el movimiento
4. Interpretar los datos de los encoders para entender el movimiento del robot

Los encoders son sensores que miden la rotación de las ruedas del robot,
permitiendo calcular la distancia recorrida y la posición del robot.

Basado en example_degrees_control.py
"""

import rospy
import time
from geometry_msgs.msg import Twist
from atriz_rvr_msgs.srv import GetEncoders

def test_encoders_with_movement():
    """Probar los encoders moviendo el robot correctamente."""
    print("🚀 Prueba de encoders con movimiento del robot")
    print("=" * 60)
    
    # Inicializar nodo ROS
    rospy.init_node('test_encoders_with_movement', anonymous=True)
    
    # Crear publisher para comandos de movimiento (usar /cmd_vel, no /sphero_rvr/cmd_vel)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    # Crear cliente para el servicio de encoders
    print("⏳ Esperando servicio /get_encoders...")
    rospy.wait_for_service('/get_encoders')
    get_encoders = rospy.ServiceProxy('/get_encoders', GetEncoders)
    print("✅ Servicio disponible")
    
    # Esperar a que el publisher se conecte
    rospy.sleep(0.5)
    
    # Función para leer encoders
    def read_encoders():
        try:
            response = get_encoders()
            if response.success:
                return response.left_wheel_count, response.right_wheel_count
            else:
                print(f"❌ Error: {response.message}")
                return None, None
        except Exception as e:
            print(f"❌ Error: {e}")
            return None, None
    
    # Función para enviar comando de movimiento
    def send_movement_command(linear_vel, angular_vel, duration):
        """Enviar comando de movimiento por una duración específica."""
        twist = Twist()
        twist.linear.x = linear_vel
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angular_vel
        
        print(f"🎮 Moviendo: {linear_vel} m/s, {angular_vel} rad/s por {duration}s")
        start_time = time.time()
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown() and (time.time() - start_time) < duration:
            cmd_vel_pub.publish(twist)
            rate.sleep()
        
        # Parar el robot
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        cmd_vel_pub.publish(twist)
        print("⏹️  Robot detenido")
    
    # Valores iniciales
    print("\n📊 Valores iniciales de los encoders:")
    left_initial, right_initial = read_encoders()
    if left_initial is not None and right_initial is not None:
        print(f"   Rueda izquierda: {left_initial}")
        print(f"   Rueda derecha: {right_initial}")
    else:
        print("❌ No se pudieron leer los valores iniciales")
        return
    
    # Prueba 1: Movimiento hacia adelante
    print("\n1️⃣ Movimiento hacia adelante (3 segundos)")
    send_movement_command(0.2, 0.0, 3.0)
    
    # Leer encoders después del movimiento
    print("📊 Valores después del movimiento hacia adelante:")
    left_after, right_after = read_encoders()
    if left_after is not None and right_after is not None:
        left_change = left_after - left_initial
        right_change = right_after - right_initial
        print(f"   Rueda izquierda: {left_after} (cambio: {left_change})")
        print(f"   Rueda derecha: {right_after} (cambio: {right_change})")
        print("   ✅ Ambas ruedas deberían haber aumentado")
    
    time.sleep(2)
    
    # Prueba 2: Giro hacia la derecha
    print("\n2️⃣ Giro hacia la derecha (3 segundos)")
    send_movement_command(0.0, -0.5, 3.0)  # Giro hacia la derecha
    
    # Leer encoders después del giro
    print("📊 Valores después del giro hacia la derecha:")
    left_after_2, right_after_2 = read_encoders()
    if left_after_2 is not None and right_after_2 is not None:
        left_change_2 = left_after_2 - left_after
        right_change_2 = right_after_2 - right_after
        print(f"   Rueda izquierda: {left_after_2} (cambio: {left_change_2})")
        print(f"   Rueda derecha: {right_after_2} (cambio: {right_change_2})")
        print("   ✅ La rueda izquierda debería haber aumentado más que la derecha")
    
    time.sleep(2)
    
    # Prueba 3: Movimiento hacia atrás
    print("\n3️⃣ Movimiento hacia atrás (3 segundos)")
    send_movement_command(-0.2, 0.0, 3.0)  # Movimiento hacia atrás
    
    # Leer encoders después del movimiento
    print("📊 Valores después del movimiento hacia atrás:")
    left_final, right_final = read_encoders()
    if left_final is not None and right_final is not None:
        left_change_3 = left_final - left_after_2
        right_change_3 = right_final - right_after_2
        print(f"   Rueda izquierda: {left_final} (cambio: {left_change_3})")
        print(f"   Rueda derecha: {right_final} (cambio: {right_change_3})")
        print("   ✅ Ambas ruedas deberían haber disminuido")
    
    print("\n" + "=" * 60)
    print("🎉 Prueba de encoders completada")
    print("📊 Resumen de cambios:")
    if left_final is not None and right_final is not None:
        total_left_change = left_final - left_initial
        total_right_change = right_final - right_initial
        print(f"   Cambio total rueda izquierda: {total_left_change}")
        print(f"   Cambio total rueda derecha: {total_right_change}")
        print(f"   Diferencia: {abs(total_left_change - total_right_change)}")
    
    print("\n📚 Explicación de los encoders:")
    print("   • Los encoders miden la rotación de las ruedas en 'pasos'")
    print("   • Valores positivos = rotación hacia adelante")
    print("   • Valores negativos = rotación hacia atrás")
    print("   • Diferencia entre ruedas = giro del robot")
    print("   • Conociendo el diámetro de las ruedas, puedes calcular distancias")
    print("   • Los encoders son esenciales para navegación autónoma")
    
    print("\n🔧 Uso en aplicaciones:")
    print("   • Odometría: calcular posición del robot")
    print("   • Control de movimiento: movimientos precisos")
    print("   • Navegación: seguir rutas programadas")
    print("   • Calibración: ajustar parámetros de movimiento")

if __name__ == '__main__':
    try:
        test_encoders_with_movement()
    except KeyboardInterrupt:
        print("\n⚠️  Prueba interrumpida por el usuario")
    except Exception as e:
        print(f"❌ Error inesperado: {e}")
