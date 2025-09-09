#!/bin/bash

# Script de acceso rápido para las pruebas del driver Atriz RVR
# Uso: ./run_tests.sh [opción]

# Obtener el directorio del script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TESTING_DIR="$SCRIPT_DIR/testing_scripts"

# Colores para output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}🚀 Scripts de Prueba del Driver Atriz RVR${NC}"
echo "================================================"

# Verificar que la carpeta de testing existe
if [ ! -d "$TESTING_DIR" ]; then
    echo "❌ Error: No se encontró la carpeta testing_scripts"
    exit 1
fi

# Mostrar opciones disponibles
echo "Opciones disponibles:"
echo "1. automated    - Pruebas automáticas completas"
echo "2. interactive  - Pruebas interactivas individuales"
echo "3. diagnostic   - Diagnóstico del sistema"
echo "4. complete     - Suite completa con diagnóstico"
echo "5. launch       - Lanzar con opciones avanzadas"
echo "6. help         - Mostrar ayuda detallada"
echo ""

# Procesar argumentos
case "$1" in
    "automated"|"1")
        echo -e "${GREEN}Ejecutando pruebas automáticas...${NC}"
        cd "$TESTING_DIR/automated"
        python3 test_atriz_rvr_driver.py
        ;;
    "interactive"|"2")
        echo -e "${GREEN}Ejecutando pruebas interactivas...${NC}"
        cd "$TESTING_DIR/interactive"
        python3 test_individual_functions.py
        ;;
    "diagnostic"|"3")
        echo -e "${GREEN}Ejecutando diagnóstico del sistema...${NC}"
        cd "$TESTING_DIR/diagnostic"
        python3 diagnose_system.py
        ;;
    "complete"|"4")
        echo -e "${GREEN}Ejecutando suite completa...${NC}"
        cd "$TESTING_DIR/automated"
        python3 run_complete_tests.py
        ;;
    "launch"|"5")
        echo -e "${GREEN}Lanzando script de opciones avanzadas...${NC}"
        cd "$TESTING_DIR/launch"
        ./launch_tests.sh "$2"
        ;;
    "help"|"6"|"")
        echo -e "${BLUE}Ayuda - Scripts de Prueba del Driver Atriz RVR${NC}"
        echo "================================================"
        echo ""
        echo "Uso: ./run_tests.sh [opción]"
        echo ""
        echo "Opciones:"
        echo "  automated    - Ejecutar pruebas automáticas completas"
        echo "  interactive  - Ejecutar pruebas interactivas individuales"
        echo "  diagnostic   - Ejecutar diagnóstico del sistema"
        echo "  complete     - Ejecutar suite completa con diagnóstico"
        echo "  launch       - Lanzar con opciones avanzadas"
        echo "  help         - Mostrar esta ayuda"
        echo ""
        echo "Ejemplos:"
        echo "  ./run_tests.sh automated"
        echo "  ./run_tests.sh interactive"
        echo "  ./run_tests.sh launch all"
        echo ""
        echo "Para más información, consulta:"
        echo "  testing_scripts/README.md"
        ;;
    *)
        echo "❌ Opción inválida: $1"
        echo "Usa './run_tests.sh help' para ver las opciones disponibles"
        exit 1
        ;;
esac
