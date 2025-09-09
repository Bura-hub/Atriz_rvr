#!/usr/bin/env python3

"""
Script de prueba simplificado para verificar el diagnóstico
"""

import rospy
import time
import subprocess
from datetime import datetime

def test_basic_checks():
    """Prueba básica del sistema"""
    print("🔍 Iniciando prueba básica del sistema...")
    
    # Verificar ROS master
    print("1️⃣ Verificando master de ROS...")
    try:
        topics = rospy.get_published_topics()
        print("✓ Master de ROS está ejecutándose")
        print(f"  - Tópicos disponibles: {len(topics)}")
    except Exception as e:
        print(f"✗ Master de ROS no está ejecutándose: {e}")
        return False
    
    # Verificar driver
    print("2️⃣ Verificando driver Atriz RVR...")
    try:
        result = subprocess.run(['pgrep', '-f', 'Atriz_rvr_node.py'], 
                              capture_output=True, text=True)
        if result.returncode == 0:
            print("✓ Driver Atriz RVR está ejecutándose")
            print(f"  - PID: {result.stdout.strip()}")
        else:
            print("✗ Driver Atriz RVR no está ejecutándose")
            return False
    except Exception as e:
        print(f"✗ Error verificando driver: {e}")
        return False
    
    # Verificar tópicos específicos
    print("3️⃣ Verificando tópicos específicos...")
    expected_topics = ['/odom', '/imu', '/cmd_vel', '/cmd_degrees']
    available_topics = [topic[0] for topic in topics]
    
    for topic in expected_topics:
        if topic in available_topics:
            print(f"✓ Tópico {topic} disponible")
        else:
            print(f"✗ Tópico {topic} no disponible")
    
    # Verificar nodos
    print("4️⃣ Verificando nodos...")
    try:
        nodes = rospy.get_node_names()
        print(f"✓ Nodos activos: {len(nodes)}")
        for node in nodes:
            print(f"  - {node}")
    except Exception as e:
        print(f"✗ Error obteniendo nodos: {e}")
    
    print("✅ Prueba básica completada")
    return True

def main():
    """Función principal"""
    try:
        rospy.init_node('test_diagnostic', anonymous=True)
        test_basic_checks()
    except rospy.ROSInterruptException:
        print("Prueba interrumpida")
    except Exception as e:
        print(f"Error en la prueba: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()
