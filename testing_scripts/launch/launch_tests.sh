#!/bin/bash

# Script para lanzar las pruebas del driver Atriz RVR
# Uso: ./launch_tests.sh [opción]
# Opciones:
#   all        - Ejecutar todas las pruebas automáticamente
#   individual - Ejecutar pruebas individuales interactivas
#   driver     - Solo lanzar el driver (sin pruebas)
#   cleanup    - Limpiar procesos en ejecución
#   help       - Mostrar esta ayuda

# Obtener el directorio del script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TESTING_DIR="$(dirname "$SCRIPT_DIR")"
PROJECT_ROOT="$(dirname "$TESTING_DIR")"

set -e

# Colores para output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Función para imprimir mensajes con color
print_message() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Función para verificar si ROS está funcionando
check_ros() {
    if ! pgrep -x "roscore" > /dev/null; then
        print_error "ROS no está ejecutándose. Iniciando roscore..."
        roscore &
        sleep 5
    else
        print_success "ROS está ejecutándose"
    fi
}

# Función para verificar si el driver está ejecutándose
check_driver() {
    if pgrep -f "Atriz_rvr_node.py" > /dev/null; then
        print_success "Driver Atriz RVR ya está ejecutándose"
        return 0
    else
        print_warning "Driver Atriz RVR no está ejecutándose"
        return 1
    fi
}

# Función para lanzar el driver
launch_driver() {
    print_message "Lanzando driver Atriz RVR..."
    
    # Verificar que el archivo del driver existe
    if [ ! -f "$PROJECT_ROOT/sphero_rvr_hw/scripts/Atriz_rvr_node.py" ]; then
        print_error "No se encontró el archivo Atriz_rvr_node.py"
        exit 1
    fi
    
    # Hacer el archivo ejecutable
    chmod +x "$PROJECT_ROOT/sphero_rvr_hw/scripts/Atriz_rvr_node.py"
    
    # Lanzar el driver en background
    python3 "$PROJECT_ROOT/sphero_rvr_hw/scripts/Atriz_rvr_node.py" &
    DRIVER_PID=$!
    
    print_message "Driver lanzado con PID: $DRIVER_PID"
    
    # Esperar a que el driver se inicialice
    print_message "Esperando a que el driver se inicialice..."
    sleep 10
    
    # Verificar que el driver está funcionando
    if check_driver; then
        print_success "Driver inicializado correctamente"
    else
        print_error "Error al inicializar el driver"
        exit 1
    fi
}

# Función para ejecutar todas las pruebas
run_all_tests() {
    print_message "Ejecutando todas las pruebas del driver..."
    
    # Verificar que el script de pruebas existe
    if [ ! -f "$TESTING_DIR/automated/test_atriz_rvr_driver.py" ]; then
        print_error "No se encontró el script de pruebas test_atriz_rvr_driver.py"
        exit 1
    fi
    
    # Hacer el archivo ejecutable
    chmod +x "$TESTING_DIR/automated/test_atriz_rvr_driver.py"
    
    # Ejecutar las pruebas
    python3 "$TESTING_DIR/automated/test_atriz_rvr_driver.py"
    
    print_success "Pruebas completadas"
}

# Función para ejecutar pruebas individuales
run_individual_tests() {
    print_message "Ejecutando pruebas individuales..."
    
    # Verificar que el script de pruebas individuales existe
    if [ ! -f "$TESTING_DIR/interactive/test_individual_functions.py" ]; then
        print_error "No se encontró el script de pruebas individuales test_individual_functions.py"
        exit 1
    fi
    
    # Hacer el archivo ejecutable
    chmod +x "$TESTING_DIR/interactive/test_individual_functions.py"
    
    # Ejecutar las pruebas individuales
    python3 "$TESTING_DIR/interactive/test_individual_functions.py"
    
    print_success "Pruebas individuales completadas"
}

# Función para limpiar procesos
cleanup() {
    print_message "Limpiando procesos..."
    
    # Terminar el driver si está ejecutándose
    if pgrep -f "Atriz_rvr_node.py" > /dev/null; then
        print_message "Terminando driver Atriz RVR..."
        pkill -f "Atriz_rvr_node.py"
    fi
    
    # Terminar los scripts de pruebas si están ejecutándose
    if pgrep -f "test_atriz_rvr_driver.py" > /dev/null; then
        print_message "Terminando script de pruebas..."
        pkill -f "test_atriz_rvr_driver.py"
    fi
    
    if pgrep -f "test_individual_functions.py" > /dev/null; then
        print_message "Terminando script de pruebas individuales..."
        pkill -f "test_individual_functions.py"
    fi
    
    print_success "Limpieza completada"
}

# Función para mostrar ayuda
show_help() {
    echo "Script para lanzar las pruebas del driver Atriz RVR"
    echo ""
    echo "Uso: $0 [opción]"
    echo ""
    echo "Opciones:"
    echo "  all        - Ejecutar todas las pruebas automáticamente"
    echo "  individual - Ejecutar pruebas individuales interactivas"
    echo "  driver     - Solo lanzar el driver (sin pruebas)"
    echo "  cleanup    - Limpiar procesos en ejecución"
    echo "  help       - Mostrar esta ayuda"
    echo ""
    echo "Ejemplos:"
    echo "  $0 all        # Ejecutar todas las pruebas"
    echo "  $0 individual # Ejecutar pruebas interactivas"
    echo "  $0 driver     # Solo lanzar el driver"
    echo "  $0 cleanup    # Limpiar procesos"
}

# Función principal
main() {
    # Configurar trap para limpieza al salir
    trap cleanup EXIT
    
    # Verificar argumentos
    if [ $# -eq 0 ]; then
        show_help
        exit 1
    fi
    
    case "$1" in
        "all")
            print_message "Modo: Ejecutar todas las pruebas"
            check_ros
            launch_driver
            run_all_tests
            ;;
        "individual")
            print_message "Modo: Pruebas individuales interactivas"
            check_ros
            launch_driver
            run_individual_tests
            ;;
        "driver")
            print_message "Modo: Solo lanzar driver"
            check_ros
            launch_driver
            print_message "Driver ejecutándose. Presione Ctrl+C para detener."
            wait
            ;;
        "cleanup")
            cleanup
            ;;
        "help")
            show_help
            ;;
        *)
            print_error "Opción inválida: $1"
            show_help
            exit 1
            ;;
    esac
}

# Ejecutar función principal
main "$@"
