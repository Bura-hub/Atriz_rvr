#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Script de prueba para verificar la conexión y funcionamiento del LIDAR
Proyecto: Sphero RVR con LIDAR X2
Actualizado: 2025-10-14

Uso:
    python3 test_lidar.py
    python3 test_lidar.py --port /dev/ttyUSB0 --baudrate 115200
    python3 test_lidar.py --scan
    python3 test_lidar.py --identify
"""

import serial
import time
import argparse
import sys
import os

# Colores para la terminal
class Colors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def print_header():
    """Imprimir encabezado del script"""
    print(Colors.BOLD + "=" * 60)
    print("     🔍 SCRIPT DE PRUEBA PARA LIDAR")
    print("     Proyecto: Sphero RVR + YDLIDAR X2")
    print("=" * 60 + Colors.ENDC)
    print()

def print_success(msg):
    """Imprimir mensaje de éxito"""
    print(f"{Colors.OKGREEN}✅ {msg}{Colors.ENDC}")

def print_warning(msg):
    """Imprimir mensaje de advertencia"""
    print(f"{Colors.WARNING}⚠️  {msg}{Colors.ENDC}")

def print_error(msg):
    """Imprimir mensaje de error"""
    print(f"{Colors.FAIL}❌ {msg}{Colors.ENDC}")

def print_info(msg):
    """Imprimir mensaje informativo"""
    print(f"{Colors.OKCYAN}📊 {msg}{Colors.ENDC}")

def check_port_availability(port):
    """Verificar si el puerto está disponible"""
    try:
        ser = serial.Serial(port, timeout=1)
        ser.close()
        return True
    except serial.SerialException as e:
        print_error(f"Error al abrir el puerto {port}: {e}")
        return False
    except Exception as e:
        print_error(f"Error inesperado: {e}")
        return False

def identify_lidar_protocol(data):
    """Identificar el protocolo del LIDAR basándose en los datos recibidos"""
    if len(data) < 2:
        return "Desconocido"
    
    # YDLidar signature: AA 55
    if data[0] == 0xAA and data[1] == 0x55:
        return "YDLidar"
    
    # RPLidar signature: A5 5A
    if data[0] == 0xA5 and data[1] == 0x5A:
        return "RPLidar"
    
    # LDROBOT signature: similar a YDLidar pero con diferencias
    if data[0] == 0xAA:
        return "YDLidar/LDROBOT"
    
    return "Desconocido"

def test_communication(port, baudrate):
    """Probar comunicación básica con el LIDAR"""
    print(f"\n{Colors.OKBLUE}📡 Probando comunicación con el LIDAR...{Colors.ENDC}")
    print(f"   Puerto: {port}")
    print(f"   Baudrate: {baudrate}")
    print()
    
    try:
        # Abrir puerto serial
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=2,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )
        
        print_success("Puerto serial abierto correctamente")
        print(f"   Configuración: {ser}")
        print()
        
        # Limpiar buffers
        ser.flushInput()
        ser.flushOutput()
        
        # Intentar leer datos
        print_info("Intentando leer datos del LIDAR...")
        print("   (Esperando 3 segundos...)")
        print()
        
        start_time = time.time()
        bytes_received = 0
        first_data = None
        lidar_type = None
        
        while time.time() - start_time < 3:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                bytes_received += len(data)
                
                # Guardar primeros datos para identificación
                if first_data is None and len(data) >= 2:
                    first_data = data
                    lidar_type = identify_lidar_protocol(data)
                
                # Mostrar primeros bytes en hexadecimal
                if bytes_received <= 50:
                    hex_data = ' '.join([f'{b:02X}' for b in data[:20]])
                    print(f"   Datos: {hex_data}")
        
        if bytes_received > 0:
            print()
            print_success("LIDAR está respondiendo!")
            print(f"   Total de bytes recibidos: {bytes_received}")
            print(f"   Tasa de datos: {bytes_received/3:.1f} bytes/segundo")
            
            if lidar_type:
                print()
                print_info(f"Tipo de LIDAR detectado: {Colors.BOLD}{lidar_type}{Colors.ENDC}")
                
                # Información específica según el tipo
                if lidar_type == "YDLidar":
                    print(f"{Colors.OKGREEN}   ✅ Protocolo YDLidar confirmado (AA 55){Colors.ENDC}")
                    print("   Modelos compatibles: X2, X2L, X4, G2, G4, TG, etc.")
                    print("   Baudrates típicos: 115200, 128000, 230400")
                    print()
                    print(f"{Colors.BOLD}   🎯 Para tu YDLIDAR X2:{Colors.ENDC}")
                    print("      - Baudrate confirmado: 115200")
                    print("      - Sample rate: 3K (3000 muestras/seg)")
                    print("      - Frecuencia: 10 Hz")
                    print("      - Alcance: 0.1-12 metros")
            
            return True, lidar_type
        else:
            print_warning("No se recibieron datos del LIDAR")
            print("   Posibles causas:")
            print("   - Baudrate incorrecto")
            print("   - LIDAR apagado o sin alimentación")
            print("   - Cable defectuoso")
            print("   - Motor del LIDAR no está girando")
            return False, None
            
    except serial.SerialException as e:
        print_error(f"Error de comunicación serial: {e}")
        return False, None
    except Exception as e:
        print_error(f"Error inesperado: {e}")
        return False, None
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print(f"\n🔌 Puerto serial cerrado")

def test_ydlidar_protocol(port, baudrate):
    """Probar protocolo específico de YDLidar"""
    print(f"\n{Colors.OKBLUE}🔄 Probando protocolo YDLidar...{Colors.ENDC}")
    
    try:
        ser = serial.Serial(port, baudrate, timeout=2)
        
        # Comandos YDLidar
        YDLIDAR_CMD_STOP = b'\xA5\x65'  # Stop scan
        YDLIDAR_CMD_DEVICE_INFO = b'\xA5\x90'  # Get device info
        
        # Limpiar buffers
        ser.flushInput()
        ser.flushOutput()
        
        # Detener escaneo primero
        print("   Enviando comando STOP...")
        ser.write(YDLIDAR_CMD_STOP)
        time.sleep(0.1)
        ser.flushInput()
        
        # Solicitar información del dispositivo
        print("   Enviando comando GET_DEVICE_INFO...")
        ser.write(YDLIDAR_CMD_DEVICE_INFO)
        time.sleep(0.5)
        
        if ser.in_waiting > 0:
            response = ser.read(ser.in_waiting)
            print_success(f"Respuesta recibida: {len(response)} bytes")
            hex_data = ' '.join([f'{b:02X}' for b in response[:20]])
            print(f"   Datos: {hex_data}")
            
            # Verificar respuesta YDLidar
            if len(response) >= 2 and response[0] == 0xAA and response[1] == 0x55:
                print_success("Confirmado: es un YDLidar!")
                return True
        else:
            print_warning("No hay respuesta al comando")
        
        ser.close()
        return False
        
    except Exception as e:
        print_error(f"Error: {e}")
        return False

def scan_baudrates(port):
    """Escanear diferentes baudrates para encontrar el correcto"""
    print(f"\n{Colors.OKBLUE}🔍 Escaneando baudrates comunes...{Colors.ENDC}\n")
    
    # Baudrates comunes para LIDARs (ordenados por probabilidad para YDLidar X2)
    common_baudrates = [
        (115200, "YDLidar X2 (confirmado)"),
        (128000, "YDLidar X2L/X4"),
        (230400, "YDLidar G2/G4, LDROBOT"),
        (256000, "RPLidar A3"),
        (460800, "YDLidar alta velocidad"),
        (921600, "YDLidar muy alta velocidad"),
    ]
    
    results = []
    
    for baudrate, description in common_baudrates:
        print(f"Probando {baudrate:6d} bps ({description})...", end=" ")
        try:
            ser = serial.Serial(port, baudrate, timeout=1)
            ser.flushInput()
            
            # Esperar un poco y ver si hay datos
            time.sleep(0.5)
            
            if ser.in_waiting > 0:
                data = ser.read(min(ser.in_waiting, 50))
                bytes_received = len(data)
                lidar_type = identify_lidar_protocol(data)
                
                print_success(f"{bytes_received} bytes | {lidar_type}")
                results.append((baudrate, bytes_received, lidar_type))
            else:
                print_warning("Sin datos")
            
            ser.close()
        except Exception as e:
            print_error(f"Error: {e}")
    
    if results:
        print(f"\n{Colors.BOLD}📊 Resultados del escaneo:{Colors.ENDC}")
        print("   Baudrates que respondieron:")
        for baudrate, bytes_count, lidar_type in sorted(results, key=lambda x: x[1], reverse=True):
            print(f"   - {baudrate:6d} bps: {bytes_count:4d} bytes | {lidar_type}")
        
        best_baudrate = max(results, key=lambda x: x[1])[0]
        print(f"\n{Colors.OKGREEN}💡 Baudrate recomendado: {best_baudrate} bps{Colors.ENDC}")
        return best_baudrate
    else:
        print_error("No se encontró ningún baudrate que funcione")
        return None

def print_ros_instructions(port, baudrate, lidar_type):
    """Imprimir instrucciones para usar con ROS"""
    print(f"\n{Colors.BOLD}{'=' * 60}")
    print("📋 PRÓXIMOS PASOS - INTEGRACIÓN CON ROS")
    print("=" * 60 + Colors.ENDC)
    print()
    
    if lidar_type and "YDLidar" in lidar_type:
        print(f"{Colors.OKGREEN}✅ YDLIDAR X2 DETECTADO{Colors.ENDC}")
        print()
        print(f"{Colors.BOLD}1. Ejecutar solo LIDAR:{Colors.ENDC}")
        print(f"   {Colors.OKCYAN}roslaunch atriz_rvr_driver lidar_only.launch{Colors.ENDC}")
        print()
        print(f"{Colors.BOLD}2. Ejecutar con integración RVR:{Colors.ENDC}")
        print(f"   {Colors.OKCYAN}roslaunch atriz_rvr_driver rvr_with_lidar.launch{Colors.ENDC}")
        print()
        print(f"{Colors.BOLD}3. Ejecutar script de integración:{Colors.ENDC}")
        print("   Terminal 1:")
        print(f"   {Colors.OKCYAN}roslaunch atriz_rvr_driver lidar_only.launch{Colors.ENDC}")
        print("   Terminal 2:")
        print(f"   {Colors.OKCYAN}rosrun atriz_rvr_driver rvr_lidar_integration.py{Colors.ENDC}")
        print()
        print(f"{Colors.BOLD}4. Visualizar en RViz:{Colors.ENDC}")
        print(f"   {Colors.OKCYAN}rviz{Colors.ENDC}")
        print("   - Add → LaserScan")
        print("   - Topic: /scan")
        print("   - Fixed Frame: laser")
    else:
        print("1. Instalar paquete ROS para tu LIDAR")
        print("2. Configurar el launch file con:")
        print(f"   - Puerto: {port}")
        print(f"   - Baudrate: {baudrate}")
        print("3. Ejecutar: roslaunch <tu_paquete> <tu_launch_file>.launch")
    
    print()
    print(f"{Colors.BOLD}📚 Documentación:{Colors.ENDC}")
    
    # Construir ruta relativa a la documentación
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    print(f"   {Colors.OKCYAN}Guía completa:{Colors.ENDC}")
    print(f"   cat {script_dir}/GUIA_COMPLETA_LIDAR.md")
    print()
    print(f"   {Colors.OKCYAN}Guía de integración:{Colors.ENDC}")
    print(f"   cat {script_dir}/INTEGRACION_RVR_LIDAR.md")
    print()

def main():
    """Función principal"""
    parser = argparse.ArgumentParser(
        description='Script de prueba para LIDAR - Proyecto Sphero RVR',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Ejemplos de uso:
  python3 test_lidar.py                    # Prueba básica
  python3 test_lidar.py --scan            # Escanear baudrates
  python3 test_lidar.py --identify        # Identificar tipo de LIDAR
  python3 test_lidar.py --port /dev/ttyUSB1 --baudrate 115200
        """
    )
    
    parser.add_argument('--port', default='/dev/ttyUSB0', 
                       help='Puerto serial (default: /dev/ttyUSB0)')
    parser.add_argument('--baudrate', type=int, default=115200, 
                       help='Baudrate (default: 115200)')
    parser.add_argument('--scan', action='store_true', 
                       help='Escanear baudrates automáticamente')
    parser.add_argument('--identify', action='store_true', 
                       help='Identificar tipo de LIDAR')
    parser.add_argument('--ydlidar', action='store_true', 
                       help='Probar protocolo YDLidar')
    
    args = parser.parse_args()
    
    print_header()
    
    # Verificar disponibilidad del puerto
    print("🔍 Verificando puerto serial...")
    if not check_port_availability(args.port):
        print(f"\n{Colors.BOLD}💡 Sugerencias:{Colors.ENDC}")
        print("   1. Verifica que el LIDAR esté conectado")
        print("   2. Verifica los permisos:")
        print(f"      {Colors.OKCYAN}sudo chmod 666 {args.port}{Colors.ENDC}")
        print("   3. Verifica que estés en el grupo dialout:")
        print(f"      {Colors.OKCYAN}groups{Colors.ENDC}")
        print("   4. Lista puertos disponibles:")
        print(f"      {Colors.OKCYAN}ls -la /dev/ttyUSB*{Colors.ENDC}")
        sys.exit(1)
    
    print_success(f"Puerto {args.port} disponible\n")
    
    # Escanear baudrates si se solicita
    if args.scan:
        best_baudrate = scan_baudrates(args.port)
        if best_baudrate:
            args.baudrate = best_baudrate
        else:
            sys.exit(1)
    
    # Probar comunicación básica
    success, lidar_type = test_communication(args.port, args.baudrate)
    
    if success:
        print(f"\n{Colors.BOLD}{'=' * 60}")
        print_success("COMUNICACIÓN EXITOSA CON EL LIDAR")
        print(f"{'=' * 60}{Colors.ENDC}")
        
        # Identificar tipo de LIDAR si se solicita
        if args.identify or args.ydlidar or (lidar_type and "YDLidar" in lidar_type):
            test_ydlidar_protocol(args.port, args.baudrate)
        
        # Imprimir instrucciones de ROS
        print_ros_instructions(args.port, args.baudrate, lidar_type)
        
    else:
        print(f"\n{Colors.BOLD}{'=' * 60}")
        print_error("NO SE PUDO ESTABLECER COMUNICACIÓN")
        print(f"{'=' * 60}{Colors.ENDC}")
        print(f"\n{Colors.BOLD}💡 Sugerencias:{Colors.ENDC}")
        print("   1. Probar con --scan para encontrar el baudrate correcto:")
        print(f"      {Colors.OKCYAN}python3 {sys.argv[0]} --scan{Colors.ENDC}")
        print("   2. Verificar alimentación del LIDAR (5V)")
        print("   3. Verificar que el motor del LIDAR esté girando")
        print("   4. Probar con otro puerto USB")
        print()
        print(f"{Colors.BOLD}📚 Ver troubleshooting completo:{Colors.ENDC}")
        script_dir = os.path.dirname(os.path.abspath(__file__))
        print(f"   cat {script_dir}/GUIA_COMPLETA_LIDAR.md")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print(f"\n\n{Colors.WARNING}⚠️  Prueba interrumpida por el usuario{Colors.ENDC}")
        sys.exit(0)
    except Exception as e:
        print_error(f"Error inesperado: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
