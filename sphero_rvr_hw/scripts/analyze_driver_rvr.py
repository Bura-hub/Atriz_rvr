#!/usr/bin/env python3

"""
Script para analizar completamente el nodo /driver_rvr
Muestra todos los tópicos, servicios y parámetros asociados
"""

import rospy
import subprocess
import sys

def run_command(command):
    """Ejecuta un comando y retorna el resultado."""
    try:
        result = subprocess.run(command, shell=True, capture_output=True, text=True)
        return result.stdout.strip()
    except Exception as e:
        return f"Error ejecutando comando: {e}"

def analyze_driver_rvr_node():
    """Analiza completamente el nodo /driver_rvr."""
    
    print("=" * 80)
    print("ANÁLISIS COMPLETO DEL NODO /driver_rvr")
    print("=" * 80)
    
    # 1. Información básica del nodo
    print("\n1. INFORMACIÓN BÁSICA DEL NODO:")
    print("-" * 40)
    node_info = run_command("rosnode info /driver_rvr")
    print(node_info)
    
    # 2. Tópicos publicados
    print("\n2. TÓPICOS PUBLICADOS:")
    print("-" * 40)
    published_topics = run_command("rosnode info /driver_rvr | grep 'Publishing' -A 20")
    print(published_topics)
    
    # 3. Tópicos suscritos
    print("\n3. TÓPICOS SUSCRITOS:")
    print("-" * 40)
    subscribed_topics = run_command("rosnode info /driver_rvr | grep 'Subscribing' -A 20")
    print(subscribed_topics)
    
    # 4. Servicios ofrecidos
    print("\n4. SERVICIOS OFRECIDOS:")
    print("-" * 40)
    services_offered = run_command("rosnode info /driver_rvr | grep 'Services' -A 20")
    print(services_offered)
    
    # 5. Servicios que usa
    print("\n5. SERVICIOS QUE USA:")
    print("-" * 40)
    services_used = run_command("rosnode info /driver_rvr | grep 'Service clients' -A 20")
    print(services_used)
    
    # 6. Parámetros del nodo
    print("\n6. PARÁMETROS DEL NODO:")
    print("-" * 40)
    parameters = run_command("rosparam list | grep driver_rvr")
    print(parameters)
    
    # 7. Tópicos relacionados con sphero_rvr
    print("\n7. TÓPICOS RELACIONADOS CON SPHERO_RVR:")
    print("-" * 40)
    sphero_topics = run_command("rostopic list | grep -E '(driver_rvr|sphero_rvr)'")
    print(sphero_topics)
    
    # 8. Servicios relacionados con sphero_rvr
    print("\n8. SERVICIOS RELACIONADOS CON SPHERO_RVR:")
    print("-" * 40)
    sphero_services = run_command("rosservice list | grep -E '(driver_rvr|sphero_rvr)'")
    print(sphero_services)
    
    # 9. Información detallada de cada tópico
    print("\n9. INFORMACIÓN DETALLADA DE TÓPICOS:")
    print("-" * 40)
    topics = run_command("rostopic list | grep -E '(driver_rvr|sphero_rvr)'")
    if topics:
        for topic in topics.split('\n'):
            if topic.strip():
                print(f"\nTópico: {topic}")
                topic_info = run_command(f"rostopic info {topic}")
                print(topic_info)
    
    # 10. Información detallada de cada servicio
    print("\n10. INFORMACIÓN DETALLADA DE SERVICIOS:")
    print("-" * 40)
    services = run_command("rosservice list | grep -E '(driver_rvr|sphero_rvr)'")
    if services:
        for service in services.split('\n'):
            if service.strip():
                print(f"\nServicio: {service}")
                service_info = run_command(f"rosservice info {service}")
                print(service_info)
    
    # 11. Frecuencia de publicación de tópicos
    print("\n11. FRECUENCIA DE PUBLICACIÓN:")
    print("-" * 40)
    topics = run_command("rostopic list | grep -E '(driver_rvr|sphero_rvr)'")
    if topics:
        for topic in topics.split('\n'):
            if topic.strip():
                print(f"\nFrecuencia de {topic}:")
                freq_info = run_command(f"rostopic hz {topic} | head -5")
                print(freq_info)
    
    print("\n" + "=" * 80)
    print("ANÁLISIS COMPLETADO")
    print("=" * 80)

def main():
    """Función principal."""
    try:
        rospy.init_node('analyze_driver_rvr', anonymous=True)
        analyze_driver_rvr_node()
    except rospy.ROSInterruptException:
        print("Análisis interrumpido por el usuario")
    except Exception as e:
        print(f"Error durante el análisis: {e}")

if __name__ == '__main__':
    main()
