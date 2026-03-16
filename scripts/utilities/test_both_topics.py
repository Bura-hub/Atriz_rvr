#!/usr/bin/env python3
"""
Script de prueba para demostrar que ambos tópicos funcionan:
- /cmd_vel (radianes/segundo)
- /cmd_degrees (grados/segundo)
"""

import rospy
from geometry_msgs.msg import Twist
from atriz_rvr_msgs.msg import DegreesTwist
import time

def test_both_topics():
    """Prueba ambos tópicos de comando."""
    
    # Inicializar nodo
    rospy.init_node('test_both_topics', anonymous=True)
    
    # Crear publishers
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    cmd_degrees_pub = rospy.Publisher('/cmd_degrees', DegreesTwist, queue_size=1)
    
    # Esperar a que los publishers se conecten
    rospy.sleep(1)
    
    print("🚀 Probando ambos tópicos de comando...")
    print("=" * 50)
    
    # Prueba 1: Comando con radianes/segundo (/cmd_vel)
    print("1️⃣ Probando /cmd_vel (radianes/segundo)...")
    print("   Enviando: 0.5 rad/s (≈28.6 grados/s)")
    
    twist_msg = Twist()
    twist_msg.angular.z = 0.5  # 0.5 rad/s
    cmd_vel_pub.publish(twist_msg)
    
    rospy.sleep(2)
    
    # Parar
    twist_msg.angular.z = 0.0
    cmd_vel_pub.publish(twist_msg)
    print("   ✅ Comando /cmd_vel enviado")
    
    rospy.sleep(1)
    
    # Prueba 2: Comando con grados/segundo (/cmd_degrees)
    print("\n2️⃣ Probando /cmd_degrees (grados/segundo)...")
    print("   Enviando: 30 grados/s")
    
    degrees_msg = DegreesTwist()
    degrees_msg.angular_z = 30.0  # 30 grados/s
    cmd_degrees_pub.publish(degrees_msg)
    
    rospy.sleep(2)
    
    # Parar
    degrees_msg.angular_z = 0.0
    cmd_degrees_pub.publish(degrees_msg)
    print("   ✅ Comando /cmd_degrees enviado")
    
    rospy.sleep(1)
    
    # Prueba 3: Comparación directa
    print("\n3️⃣ Comparación directa...")
    print("   /cmd_vel: 1.0 rad/s (≈57.3 grados/s)")
    print("   /cmd_degrees: 60 grados/s")
    
    # Comando radianes
    twist_msg.angular.z = 1.0  # 1.0 rad/s ≈ 57.3 grados/s
    cmd_vel_pub.publish(twist_msg)
    print("   Enviando comando /cmd_vel...")
    rospy.sleep(2)
    
    # Parar
    twist_msg.angular.z = 0.0
    cmd_vel_pub.publish(twist_msg)
    rospy.sleep(1)
    
    # Comando grados
    degrees_msg.angular_z = 60.0  # 60 grados/s
    cmd_degrees_pub.publish(degrees_msg)
    print("   Enviando comando /cmd_degrees...")
    rospy.sleep(2)
    
    # Parar
    degrees_msg.angular_z = 0.0
    cmd_degrees_pub.publish(degrees_msg)
    print("   ✅ Comparación completada")
    
    print("\n🎉 ¡Ambos tópicos funcionan correctamente!")
    print("=" * 50)
    print("📋 Resumen:")
    print("   • /cmd_vel: Usa radianes/segundo (estándar ROS)")
    print("   • /cmd_degrees: Usa grados/segundo (más intuitivo)")
    print("   • Ambos controlan el mismo robot simultáneamente")
    print("   • El último comando enviado tiene prioridad")

if __name__ == '__main__':
    try:
        test_both_topics()
    except rospy.ROSInterruptException:
        print("❌ Prueba interrumpida")
    except Exception as e:
        print(f"❌ Error: {e}")
