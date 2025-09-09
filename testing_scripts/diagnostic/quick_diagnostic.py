#!/usr/bin/env python3

"""
Script de diagnóstico rápido para el sistema Atriz RVR
"""

import rospy
import subprocess
import time
from datetime import datetime

def quick_diagnostic():
    """Diagnóstico rápido del sistema"""
    print("🔍 Diagnóstico Rápido del Sistema Atriz RVR")
    print("=" * 50)
    
    # 1. Verificar ROS master
    print("1️⃣ Verificando ROS master...")
    try:
        rospy.init_node('quick_diagnostic', anonymous=True)
        print("✓ ROS master está ejecutándose")
    except Exception as e:
        print(f"✗ Error con ROS master: {e}")
        return False
    
    # 2. Verificar driver
    print("2️⃣ Verificando driver...")
    try:
        result = subprocess.run(['pgrep', '-f', 'Atriz_rvr_node.py'], 
                              capture_output=True, text=True)
        if result.returncode == 0:
            print(f"✓ Driver ejecutándose (PID: {result.stdout.strip()})")
        else:
            print("✗ Driver no está ejecutándose")
            return False
    except Exception as e:
        print(f"✗ Error verificando driver: {e}")
        return False
    
    # 3. Verificar tópicos
    print("3️⃣ Verificando tópicos...")
    try:
        result = subprocess.run(['rostopic', 'list'], capture_output=True, text=True)
        if result.returncode == 0:
            topics = result.stdout.strip().split('\n')
            print(f"✓ {len(topics)} tópicos disponibles")
            
            # Verificar tópicos específicos
            expected_topics = ['/odom', '/imu', '/cmd_vel', '/cmd_degrees', '/color', '/ambient_light']
            for topic in expected_topics:
                if topic in topics:
                    print(f"  ✓ {topic}")
                else:
                    print(f"  ✗ {topic}")
        else:
            print("✗ Error obteniendo lista de tópicos")
            return False
    except Exception as e:
        print(f"✗ Error verificando tópicos: {e}")
        return False
    
    # 4. Verificar servicios
    print("4️⃣ Verificando servicios...")
    try:
        result = subprocess.run(['rosservice', 'list'], capture_output=True, text=True)
        if result.returncode == 0:
            services = result.stdout.strip().split('\n')
            print(f"✓ {len(services)} servicios disponibles")
            
            # Verificar servicios específicos
            expected_services = ['/battery_state', '/enable_color', '/reset_odom', '/release_emergency_stop']
            for service in expected_services:
                if service in services:
                    print(f"  ✓ {service}")
                else:
                    print(f"  ✗ {service}")
        else:
            print("✗ Error obteniendo lista de servicios")
            return False
    except Exception as e:
        print(f"✗ Error verificando servicios: {e}")
        return False
    
    # 5. Verificar nodos
    print("5️⃣ Verificando nodos...")
    try:
        result = subprocess.run(['rosnode', 'list'], capture_output=True, text=True)
        if result.returncode == 0:
            nodes = result.stdout.strip().split('\n')
            print(f"✓ {len(nodes)} nodos activos")
            for node in nodes:
                if 'driver' in node or 'rvr' in node:
                    print(f"  ✓ {node}")
        else:
            print("✗ Error obteniendo lista de nodos")
            return False
    except Exception as e:
        print(f"✗ Error verificando nodos: {e}")
        return False
    
    print("=" * 50)
    print("✅ Diagnóstico rápido completado")
    return True

def main():
    """Función principal"""
    try:
        quick_diagnostic()
    except KeyboardInterrupt:
        print("\nDiagnóstico interrumpido")
    except Exception as e:
        print(f"Error en el diagnóstico: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()
