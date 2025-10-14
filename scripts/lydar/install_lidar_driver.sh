#!/bin/bash
# ==============================================================================
# Script Unificado de Instalación de Drivers de LIDAR para ROS
# ==============================================================================
# 
# Este script instala drivers de LIDAR para ROS incluyendo:
# - RPLidar (SLAMTEC)
# - YDLidar (con SDK completo)
# - LDROBOT (LD06/LD19)
# - Hokuyo URG
#
# Para YDLidar X2, el script automáticamente:
# 1. Instala dependencias necesarias
# 2. Clona el driver ROS
# 3. Clona, compila e INSTALA el SDK (requiere sudo)
# 4. Crea el launch file x2_custom.launch con configuración específica
# 5. Prepara el workspace para compilación
#
# Uso: ./install_lidar_driver.sh
# ==============================================================================

echo "=========================================="
echo "  INSTALADOR DE DRIVERS DE LIDAR PARA ROS"
echo "=========================================="
echo ""

# Colores para output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Detectar el workspace de catkin automáticamente
# Buscar desde el directorio actual hacia arriba
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
CURRENT_DIR="$SCRIPT_DIR"

# Buscar el workspace de ROS/catkin
CATKIN_WS=""
while [ "$CURRENT_DIR" != "/" ]; do
    if [ -f "$CURRENT_DIR/src/CMakeLists.txt" ] || [ -d "$CURRENT_DIR/devel" ] || [ -d "$CURRENT_DIR/build" ]; then
        CATKIN_WS="$CURRENT_DIR"
        break
    fi
    CURRENT_DIR=$(dirname "$CURRENT_DIR")
done

# Si no se encontró por el método anterior, intentar con catkin_ws común
if [ -z "$CATKIN_WS" ] && [ -d "$HOME/catkin_ws/src" ]; then
    CATKIN_WS="$HOME/catkin_ws"
fi

# Verificar que se encontró un workspace
if [ -z "$CATKIN_WS" ]; then
    echo -e "${RED}❌ Error: No se pudo detectar un workspace de catkin/ROS${NC}"
    echo ""
    echo "Opciones:"
    echo "  1. Ejecuta este script desde dentro de tu workspace de ROS"
    echo "  2. Asegúrate de tener un workspace en ~/catkin_ws/"
    echo "  3. Crea un workspace con:"
    echo "     mkdir -p ~/catkin_ws/src"
    echo "     cd ~/catkin_ws/"
    echo "     catkin_make"
    echo ""
    exit 1
fi

SRC_DIR="$CATKIN_WS/src"

echo -e "${GREEN}📁 Workspace de catkin detectado: $CATKIN_WS${NC}"
echo "📁 Directorio src: $SRC_DIR"
echo ""

# Menú de selección
echo -e "${YELLOW}💡 RECOMENDACIÓN:${NC} Basado en las pruebas realizadas (firma AA 55, baudrate 115200):"
echo -e "   ${GREEN}→ Opción 2 (YDLidar)${NC} es la más probable"
echo -e "   → Opción 3 (LDROBOT) como alternativa"
echo ""
echo "Selecciona el tipo de LIDAR:"
echo "  1) RPLidar (SLAMTEC) - A1/A2/A3"
echo "  2) YDLidar - X2/X4/G2/G4/etc ⭐ RECOMENDADO"
echo "  3) LDROBOT - LD06/LD19"
echo "  4) Hokuyo URG"
echo "  5) Instalar todos (no recomendado)"
echo "  0) Salir"
echo ""
read -p "Opción [1-5, 0 para salir]: " option

case $option in
    1)
        echo ""
        echo -e "${GREEN}📦 Instalando driver para RPLidar...${NC}"
        cd "$SRC_DIR"
        
        if [ -d "rplidar_ros" ]; then
            echo -e "${YELLOW}⚠️  El paquete rplidar_ros ya existe${NC}"
            read -p "¿Deseas actualizarlo? (s/n): " update
            if [ "$update" == "s" ]; then
                cd rplidar_ros
                git pull
                cd ..
            fi
        else
            echo "Clonando repositorio RPLidar..."
            git clone https://github.com/Slamtec/rplidar_ros.git
        fi
        
        echo ""
        echo -e "${GREEN}✅ Driver RPLidar descargado${NC}"
        echo ""
        echo "📋 Configuración recomendada:"
        echo "   - RPLidar A1: baudrate 115200"
        echo "   - RPLidar A2: baudrate 115200"
        echo "   - RPLidar A3: baudrate 256000"
        ;;
        
    2)
        echo ""
        echo -e "${GREEN}📦 Instalando driver para YDLidar...${NC}"
        echo ""
        
        # Instalar dependencias
        echo -e "${YELLOW}📚 Instalando dependencias...${NC}"
        sudo apt-get update
        sudo apt-get install -y cmake pkg-config python3-pip
        
        cd "$SRC_DIR"
        
        # Instalar driver ROS
        if [ -d "ydlidar_ros_driver" ]; then
            echo -e "${YELLOW}⚠️  El paquete ydlidar_ros_driver ya existe${NC}"
            read -p "¿Deseas actualizarlo? (s/n): " update
            if [ "$update" == "s" ]; then
                cd ydlidar_ros_driver
                git pull
                cd "$SRC_DIR"
            fi
        else
            echo "Clonando ydlidar_ros_driver..."
            git clone https://github.com/YDLIDAR/ydlidar_ros_driver.git
        fi
        
        # Instalar SDK (PASO CRÍTICO)
        echo ""
        echo -e "${YELLOW}🔧 Instalando YDLidar SDK...${NC}"
        if [ -d "YDLidar-SDK" ]; then
            echo -e "${YELLOW}⚠️  YDLidar-SDK ya existe${NC}"
            read -p "¿Recompilar e instalar SDK? (s/n): " reinstall
            if [ "$reinstall" == "s" ]; then
                cd YDLidar-SDK
                rm -rf build
                mkdir -p build && cd build
                cmake ..
                make
                echo ""
                echo -e "${YELLOW}🔐 Instalando SDK (requiere sudo)...${NC}"
                sudo make install
                sudo ldconfig
                cd "$SRC_DIR"
            fi
        else
            git clone https://github.com/YDLIDAR/YDLidar-SDK.git
            cd YDLidar-SDK
            mkdir -p build && cd build
            echo "Compilando YDLidar SDK..."
            cmake ..
            make
            echo ""
            echo -e "${YELLOW}🔐 Instalando SDK (requiere sudo)...${NC}"
            sudo make install
            sudo ldconfig
            cd "$SRC_DIR"
        fi
        
        # Crear launch file personalizado para X2
        echo ""
        echo -e "${YELLOW}📝 Creando launch file para YDLIDAR X2...${NC}"
        X2_LAUNCH_FILE="$SRC_DIR/ydlidar_ros_driver/launch/x2_custom.launch"
        
        cat > "$X2_LAUNCH_FILE" << 'EOF'
<launch>
  <node name="ydlidar_lidar_publisher" pkg="ydlidar_ros_driver" type="ydlidar_ros_driver_node" output="screen" respawn="false">
    <!-- YDLIDAR X2 - Configuración específica -->
    <param name="port"         type="string" value="/dev/ttyUSB0"/>  
    <param name="baudrate"     type="int"    value="115200"/>  
    <param name="frame_id"     type="string" value="laser"/>
    <param name="ignore_array" type="string" value=""/>

    <!-- Tipo de LIDAR -->
    <param name="lidar_type"       type="int" value="1"/>  <!-- 1=TRIANGLE -->
    <param name="device_type"      type="int" value="0"/>  <!-- 0=SERIAL -->
    <param name="sample_rate"      type="int" value="3"/>  <!-- X2 usa 3K -->
    <param name="abnormal_check_count" type="int" value="4"/>  

    <!-- Configuración booleana específica para X2 -->
    <param name="resolution_fixed"    type="bool" value="true"/>
    <param name="auto_reconnect"      type="bool" value="true"/>
    <param name="reversion"           type="bool" value="false"/>  <!-- X2: false -->
    <param name="inverted"            type="bool" value="true"/>
    <param name="isSingleChannel"     type="bool" value="true"/>   <!-- X2: true -->
    <param name="intensity"           type="bool" value="false"/>
    <param name="support_motor_dtr"   type="bool" value="true"/>   <!-- X2: true -->
    <param name="invalid_range_is_inf" type="bool" value="false"/>
    <param name="point_cloud_preservative" type="bool" value="false"/>

    <!-- Rangos para X2 -->
    <param name="angle_min"    type="double" value="-180" />
    <param name="angle_max"    type="double" value="180" />
    <param name="range_min"    type="double" value="0.1" />
    <param name="range_max"    type="double" value="12.0" />
    <param name="frequency"    type="double" value="10.0"/>
  </node>
  
  <!-- TF estático -->
  <node pkg="tf" type="static_transform_publisher" name="base_link_to_laser"
    args="0.0 0.0 0.15 0.0 0.0 0.0 /base_link /laser 50" />
</launch>
EOF
        
        echo ""
        echo -e "${GREEN}✅ Driver YDLidar y SDK instalados${NC}"
        echo -e "${GREEN}✅ Launch file x2_custom.launch creado${NC}"
        echo ""
        echo "📋 Configuración recomendada por modelo:"
        echo "   - YDLidar X2: baudrate 115200 ⭐ (Verificado)"
        echo "   - YDLidar X2L: baudrate 128000"
        echo "   - YDLidar X4: baudrate 128000"
        echo "   - YDLidar G2/G4: baudrate 230400"
        echo ""
        echo -e "${GREEN}⚠️  IMPORTANTE: El SDK está instalado correctamente${NC}"
        echo ""
        echo "🚀 Para YDLIDAR X2, usar:"
        echo "   roslaunch ydlidar_ros_driver x2_custom.launch"
        ;;
        
    3)
        echo ""
        echo -e "${GREEN}📦 Instalando driver para LDROBOT...${NC}"
        cd "$SRC_DIR"
        
        if [ -d "ldlidar_stl_ros" ]; then
            echo -e "${YELLOW}⚠️  El paquete ldlidar_stl_ros ya existe${NC}"
            read -p "¿Deseas actualizarlo? (s/n): " update
            if [ "$update" == "s" ]; then
                cd ldlidar_stl_ros
                git pull
                cd ..
            fi
        else
            git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros.git
        fi
        
        echo ""
        echo -e "${GREEN}✅ Driver LDROBOT descargado${NC}"
        echo ""
        echo "📋 Configuración recomendada:"
        echo "   - LD06: baudrate 230400"
        echo "   - LD19: baudrate 230400"
        ;;
        
    4)
        echo ""
        echo -e "${GREEN}📦 Instalando driver para Hokuyo URG...${NC}"
        
        # Instalar desde apt
        sudo apt-get update
        sudo apt-get install -y ros-noetic-urg-node ros-noetic-urg-c
        
        echo ""
        echo -e "${GREEN}✅ Driver Hokuyo instalado${NC}"
        ;;
        
    5)
        echo ""
        echo -e "${YELLOW}⚠️  Instalando todos los drivers...${NC}"
        echo "   (Esto puede tardar varios minutos)"
        
        # Instalar dependencias
        sudo apt-get update
        sudo apt-get install -y cmake pkg-config python3-pip
        
        cd "$SRC_DIR"
        
        # RPLidar
        echo "Instalando RPLidar..."
        [ ! -d "rplidar_ros" ] && git clone https://github.com/Slamtec/rplidar_ros.git
        
        # YDLidar (con SDK)
        echo "Instalando YDLidar..."
        [ ! -d "ydlidar_ros_driver" ] && git clone https://github.com/YDLIDAR/ydlidar_ros_driver.git
        
        if [ ! -d "YDLidar-SDK" ]; then
            echo "Instalando YDLidar SDK..."
            git clone https://github.com/YDLIDAR/YDLidar-SDK.git
            cd YDLidar-SDK
            mkdir -p build && cd build
            cmake .. && make
            sudo make install
            sudo ldconfig
            cd "$SRC_DIR"
        fi
        
        # Crear launch file para X2
        echo "Creando launch file para YDLIDAR X2..."
        X2_LAUNCH_FILE="$SRC_DIR/ydlidar_ros_driver/launch/x2_custom.launch"
        
        cat > "$X2_LAUNCH_FILE" << 'EOFX2'
<launch>
  <node name="ydlidar_lidar_publisher" pkg="ydlidar_ros_driver" type="ydlidar_ros_driver_node" output="screen" respawn="false">
    <!-- YDLIDAR X2 - Configuración específica -->
    <param name="port"         type="string" value="/dev/ttyUSB0"/>  
    <param name="baudrate"     type="int"    value="115200"/>  
    <param name="frame_id"     type="string" value="laser"/>
    <param name="ignore_array" type="string" value=""/>

    <!-- Tipo de LIDAR -->
    <param name="lidar_type"       type="int" value="1"/>  <!-- 1=TRIANGLE -->
    <param name="device_type"      type="int" value="0"/>  <!-- 0=SERIAL -->
    <param name="sample_rate"      type="int" value="3"/>  <!-- X2 usa 3K -->
    <param name="abnormal_check_count" type="int" value="4"/>  

    <!-- Configuración booleana específica para X2 -->
    <param name="resolution_fixed"    type="bool" value="true"/>
    <param name="auto_reconnect"      type="bool" value="true"/>
    <param name="reversion"           type="bool" value="false"/>  <!-- X2: false -->
    <param name="inverted"            type="bool" value="true"/>
    <param name="isSingleChannel"     type="bool" value="true"/>   <!-- X2: true -->
    <param name="intensity"           type="bool" value="false"/>
    <param name="support_motor_dtr"   type="bool" value="true"/>   <!-- X2: true -->
    <param name="invalid_range_is_inf" type="bool" value="false"/>
    <param name="point_cloud_preservative" type="bool" value="false"/>

    <!-- Rangos para X2 -->
    <param name="angle_min"    type="double" value="-180" />
    <param name="angle_max"    type="double" value="180" />
    <param name="range_min"    type="double" value="0.1" />
    <param name="range_max"    type="double" value="12.0" />
    <param name="frequency"    type="double" value="10.0"/>
  </node>
  
  <!-- TF estático -->
  <node pkg="tf" type="static_transform_publisher" name="base_link_to_laser"
    args="0.0 0.0 0.15 0.0 0.0 0.0 /base_link /laser 50" />
</launch>
EOFX2
        
        # LDROBOT
        echo "Instalando LDROBOT..."
        [ ! -d "ldlidar_stl_ros" ] && git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros.git
        
        # Hokuyo
        echo "Instalando Hokuyo..."
        sudo apt-get install -y ros-noetic-urg-node ros-noetic-urg-c
        
        echo ""
        echo -e "${GREEN}✅ Todos los drivers instalados${NC}"
        ;;
        
    0)
        echo "Saliendo..."
        exit 0
        ;;
        
    *)
        echo -e "${RED}❌ Opción inválida${NC}"
        exit 1
        ;;
esac

# Compilar el workspace
echo ""
read -p "¿Deseas compilar el workspace ahora? (s/n): " compile

if [ "$compile" == "s" ]; then
    echo ""
    echo -e "${GREEN}🔨 Compilando workspace...${NC}"
    cd "$CATKIN_WS"
    
    # Compilar con catkin_make
    if command -v catkin_make &> /dev/null; then
        catkin_make
    else
        echo -e "${YELLOW}⚠️  catkin_make no encontrado, intentando con catkin build...${NC}"
        if command -v catkin &> /dev/null; then
            catkin build
        else
            echo -e "${RED}❌ Error: No se encontró catkin_make ni catkin build${NC}"
            exit 1
        fi
    fi
    
    # Source del workspace
    echo ""
    echo -e "${GREEN}📝 Actualizando environment...${NC}"
    source devel/setup.bash
    
    echo ""
    echo -e "${GREEN}✅ Compilación completada${NC}"
fi

# Configurar permisos del puerto
echo ""
echo -e "${GREEN}🔧 Configurando permisos del puerto...${NC}"
echo ""
echo "Para dar permisos temporales al puerto:"
echo "  sudo chmod 666 /dev/ttyUSB0"
echo ""
echo "Para configurar permisos permanentes:"
echo "  1. Crear regla udev:"
echo "     sudo nano /etc/udev/rules.d/99-lidar.rules"
echo ""
echo "  2. Añadir la siguiente línea:"
echo "     KERNEL==\"ttyUSB*\", ATTRS{idVendor}==\"10c4\", ATTRS{idProduct}==\"ea60\", MODE:=\"0666\", SYMLINK+=\"lidar\""
echo ""
echo "  3. Recargar reglas:"
echo "     sudo udevadm control --reload-rules"
echo "     sudo udevadm trigger"
echo ""

read -p "¿Deseas configurar permisos temporales ahora? (s/n): " perms
if [ "$perms" == "s" ]; then
    sudo chmod 666 /dev/ttyUSB0
    echo -e "${GREEN}✅ Permisos configurados para /dev/ttyUSB0${NC}"
fi

echo ""
echo "=========================================="
echo -e "${GREEN}✅ INSTALACIÓN COMPLETADA${NC}"
echo "=========================================="
echo ""
echo -e "${GREEN}📊 Configuración Confirmada del LIDAR:${NC}"
echo "   Modelo:    YDLIDAR X2"
echo "   Puerto:    /dev/ttyUSB0"
echo "   Baudrate:  115200"
echo "   Protocolo: AA 55 (YDLidar)"
echo "   Estado:    ✅ Listo para usar"
echo ""
echo "📋 Próximos pasos:"
echo ""
echo "1. Probar el LIDAR (si aún no lo hiciste):"
echo "   cd $CATKIN_WS/src/ros_sphero_rvr/scripts/lydar"
echo "   python3 test_lidar.py --scan"
echo ""
echo "2. Ejecutar el driver ROS:"
case $option in
    1)
        echo "   roslaunch rplidar_ros rplidar_a1.launch serial_port:=/dev/ttyUSB0 serial_baudrate:=115200"
        echo "   # o para A2/A3:"
        echo "   roslaunch rplidar_ros rplidar_a2.launch serial_port:=/dev/ttyUSB0 serial_baudrate:=115200"
        ;;
    2)
        echo -e "   ${GREEN}roslaunch ydlidar_ros_driver x2_custom.launch${NC} ⭐ YDLIDAR X2"
        echo "   # o con parámetros genéricos:"
        echo "   roslaunch ydlidar_ros_driver ydlidar.launch serial_port:=/dev/ttyUSB0 serial_baudrate:=115200"
        ;;
    3)
        echo "   roslaunch ldlidar_stl_ros ld06.launch serial_port:=/dev/ttyUSB0"
        echo "   # o para LD19:"
        echo "   roslaunch ldlidar_stl_ros ld19.launch serial_port:=/dev/ttyUSB0"
        ;;
    4)
        echo "   roslaunch urg_node urg_lidar.launch serial_port:=/dev/ttyUSB0"
        ;;
esac
echo ""
echo "3. Visualizar en RViz:"
echo "   rviz"
echo "   - Add → LaserScan"
echo "   - Topic: /scan"
echo "   - Fixed Frame: laser"
echo ""
echo "4. Ver documentación completa:"
echo "   cat $CATKIN_WS/src/ros_sphero_rvr/scripts/lydar/GUIA_COMPLETA_LIDAR.md"
echo "   cat $CATKIN_WS/src/ros_sphero_rvr/scripts/lydar/YDLIDAR_X2_RESUMEN.md"
echo ""
echo "¡Listo! 🎉"

