#!/usr/bin/env python3
"""
Script de prueba para verificar el manejo de Ctrl+C
Este es un script simple que puedes ejecutar para probar que Ctrl+C funciona correctamente
"""

import rospy
from geometry_msgs.msg import Twist
import signal
import sys
import time

class TestCtrlC:
    def __init__(self):
        rospy.init_node('test_ctrl_c', anonymous=True)
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Configurar manejador de señales
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("🧪 TEST DE Ctrl+C")
        rospy.loginfo("=" * 60)
        rospy.loginfo("Este script hará que el robot se mueva en círculo")
        rospy.loginfo("Presiona Ctrl+C en cualquier momento para detenerlo")
        rospy.loginfo("=" * 60)
        
    def signal_handler(self, signum, frame):
        rospy.logwarn("")
        rospy.logwarn("=" * 60)
        rospy.logwarn("✋ Ctrl+C DETECTADO!")
        rospy.logwarn("🛑 Deteniendo robot...")
        rospy.logwarn("=" * 60)
        
        # Detener robot
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        for _ in range(10):
            self.publisher.publish(cmd)
            rospy.sleep(0.1)
        
        rospy.loginfo("✅ Robot detenido correctamente")
        rospy.loginfo("✅ Manejo de Ctrl+C funciona perfectamente!")
        rospy.loginfo("👋 Saliendo...")
        sys.exit(0)
    
    def run(self):
        rospy.loginfo("")
        rospy.loginfo("🔄 Iniciando movimiento en círculo...")
        rospy.loginfo("   (El robot girará indefinidamente hasta que presiones Ctrl+C)")
        rospy.loginfo("")
        
        rate = rospy.Rate(10)
        contador = 0
        
        while not rospy.is_shutdown():
            cmd = Twist()
            cmd.linear.x = 0.2
            cmd.angular.z = 0.3
            
            self.publisher.publish(cmd)
            
            contador += 1
            if contador % 50 == 0:  # Cada 5 segundos
                rospy.loginfo(f"⏱️  Tiempo transcurrido: {contador/10:.1f} segundos - Presiona Ctrl+C para detener")
            
            rate.sleep()

if __name__ == '__main__':
    try:
        test = TestCtrlC()
        test.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROSInterruptException")
    except Exception as e:
        rospy.logerr(f"Error: {e}")

