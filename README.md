# Sphero RVR ROS Driver - Atriz

Este repositorio contiene el código necesario para operar el robot Sphero RVR utilizando ROS (Robot Operating System). Está diseñado como parte de un proyecto para la Universidad de Nariño, facilitando la integración y control del Sphero RVR en entornos de investigación y desarrollo.

## 🚀 Características Principales

- **Control Dual de Tópicos**: Soporte para `/cmd_vel` (radianes/segundo) y `/cmd_degrees` (grados/segundo)
- **Control Intuitivo**: Comandos directos en grados para facilitar el uso humano
- **Compatibilidad ROS**: Mantiene el estándar ROS con `/cmd_vel`
- **Seguridad Avanzada**: Sistema de parada de emergencia y monitoreo de batería
- **Sensores Integrados**: Odometría, IMU, sensores de color y luz ambiental
- **Control de LEDs**: Sistema completo de control de iluminación
- **Suite de Pruebas**: Scripts de prueba automatizados, interactivos y de diagnóstico
- **Documentación Completa**: Guías detalladas organizadas por categorías
- **Configuración Automática**: Setup automático de Python y dependencias
- **Linter Optimizado**: Configuración completa para desarrollo sin errores

## 📁 Estructura del Proyecto

```
atriz_git/src/Atriz_rvr/
├── 📄 README.md                           # Este archivo
├── 📄 PROJECT_ORGANIZATION_REPORT.md     # Reporte de organización
├── 📄 GUIA_COMPLETA_LIDAR.md              # Guía LIDAR YDLIDAR X2 (udev, SDK, integración)
├── 🚀 run_tests.sh                        # Script principal de pruebas
├── 🚀 start_ros.sh                        # Script de inicio de ROS
├── ⚙️ setup_python_path.py                # Configuración automática de Python
├── ⚙️ pyrightconfig.json                  # Configuración del linter
├── 📁 docs/                               # 📚 Documentación organizada
│   ├── README.md                          # Índice de documentación
│   ├── driver/                            # Documentación del driver
│   │   └── DRIVER_FUNCTIONALITY_GUIDE.md # Guía completa de funcionalidades
│   ├── testing/                           # Documentación de pruebas
│   │   └── README_TESTING.md             # Guía de testing
│   ├── hardware/                          # Documentación de hardware
│   │   └── README.md                     # Documentación RVR++
│   ├── scripts/                           # Documentación de scripts
│   │   ├── GRADOS_CONTROL_README.md      # Control por grados
│   │   ├── GRADOS_DIRECTOS_README.md     # Grados directos
│   │   ├── LINTER_ERRORS_FIXED.md        # Corrección de errores
│   │   └── PYTHON_IMPORT_SETUP.md        # Configuración Python
│   └── packages_README.md                 # Guía de paquetes
├── 📁 atriz_rvr_driver/                   # 📦 Driver principal ROS
│   ├── package.xml                        # Metadatos del paquete
│   ├── setup.py                           # Configuración Python
│   ├── CMakeLists.txt                     # Configuración de build
│   ├── launch/                            # Launch files
│   │   ├── lidar_only.launch             # Solo LIDAR X2
│   │   ├── rvr_with_lidar.launch         # RVR + LIDAR
│   │   └── rvr_with_lidar_autonomous.launch  # RVR + LIDAR + evitación obstáculos
│   └── scripts/                           # Scripts del driver
│       ├── Atriz_rvr_node.py            # Driver principal (ÚNICO)
│       ├── cmd_vel_rviz.py               # Control RViz
│       ├── degrees_control_example.py    # Ejemplo de grados
│       ├── emergency_stop.py             # Parada de emergencia
│       ├── obstacle_avoidance.py        # Evitación autónoma con LIDAR
│       ├── rvr-ros-restarter.py          # Reiniciador automático
│       ├── rvr-ros.py                    # Driver alternativo
│       ├── rvr_tools.py                  # Herramientas
│       ├── rvr_lidar_integration.py      # Integración LIDAR
│       └── sphero_sdk/                   # SDK completo de Sphero
├── 📁 atriz_rvr_msgs/                     # 📦 Mensajes personalizados ROS
│   ├── package.xml                        # Metadatos del paquete
│   ├── msg/                               # Definiciones de mensajes
│   └── srv/                               # Definiciones de servicios
├── 📁 atriz_rvr_serial/                   # 📦 Biblioteca serial ROS
│   ├── package.xml                        # Metadatos del paquete
│   ├── src/                               # Código fuente C++
│   ├── include/                           # Headers
│   └── tests/                             # Pruebas de la biblioteca
├── 📁 scripts/                            # 🚀 Scripts organizados por funcionalidad
│   ├── README.md                          # Guía de scripts
│   ├── examples/                          # Ejemplos de uso
│   │   ├── example_encoder_test.py       # Prueba de encoders
│   │   └── rgbc_direct_test.py           # Prueba sensor RGB
│   ├── tools/                             # Herramientas
│   │   └── color_listener.py             # Listener de color
│   ├── utilities/                         # Utilidades
│   │   └── test_both_topics.py           # Prueba de ambos tópicos
│   └── lydar/                             # LIDAR YDLIDAR
│       ├── README.md                     # Inicio rápido LIDAR
│       ├── INTEGRACION_RVR_LIDAR.md      # Integración RVR + LIDAR
│       ├── install_lidar_driver.sh       # Instalador driver + SDK
│       └── test_lidar.py                 # Prueba LIDAR
└── 📁 testing_scripts/                    # 🧪 Suite completa de pruebas
    ├── README.md                          # Guía de testing
    ├── automated/                         # Pruebas automáticas
    │   ├── test_atriz_rvr_driver.py      # Pruebas del driver
    │   └── run_complete_tests.py         # Pruebas completas
    ├── interactive/                       # Pruebas interactivas
    │   ├── test_individual_functions.py  # Pruebas individuales
    │   ├── test_interactive.py           # Pruebas interactivas
    │   └── test_corrected.py             # Pruebas corregidas
    ├── diagnostic/                        # Diagnóstico del sistema
    │   └── diagnose_system.py            # Diagnóstico completo
    └── launch/                            # Scripts de lanzamiento
        └── launch_tests.sh               # Lanzador de pruebas
```

## 📚 Documentación Completa

Toda la documentación está organizada en la carpeta `docs/`:

- **[Índice de Documentación](docs/README.md)** - Guía principal de documentación
- **[Guía del Driver](docs/driver/DRIVER_FUNCTIONALITY_GUIDE.md)** - Funcionalidades completas del driver
- **[Guía LIDAR YDLIDAR X2](GUIA_COMPLETA_LIDAR.md)** - Instalación udev/SDK, integración RVR, evitación de obstáculos
- **[Guía de Pruebas](docs/testing/README_TESTING.md)** - Scripts y testing detallado
- **[Scripts de Prueba](testing_scripts/README.md)** - Pruebas organizadas y documentadas
- **[Scripts del Proyecto](scripts/README.md)** - Scripts organizados por funcionalidad
- **[Paquetes ROS](docs/packages_README.md)** - Guía de paquetes ROS

## 🚀 Inicio Rápido

### **1. Configuración Automática**
```bash
# Configurar rutas de Python automáticamente
python3 setup_python_path.py

# Verificar configuración
python3 -c "from sphero_sdk_config import setup_sphero_sdk_path; print('SDK path:', setup_sphero_sdk_path())"
```

### **2. Pruebas Rápidas**
```bash
# Ver todas las opciones disponibles
./run_tests.sh help

# Ejecutar todas las pruebas automáticamente
./run_tests.sh automated

# Pruebas interactivas
./run_tests.sh interactive

# Diagnóstico del sistema
./run_tests.sh diagnostic

# Pruebas completas (automáticas + diagnóstico)
./run_tests.sh complete
```

### **3. Ejecutar el Driver**
```bash
# Opción 1: Usar el script de inicio
./start_ros.sh

# Opción 2: Ejecutar directamente
python3 ./atriz_rvr_driver/scripts/Atriz_rvr_node.py
```

## ⚙️ Requisitos Previos

- **ROS Noetic** instalado en Ubuntu 20.04
- **Python 3.8** o superior
- **Sphero RVR** robot
- **Conexión Bluetooth** o USB

## 🔧 Instalación

### 1. Crear el Workspace de ROS

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```

### 2. Clonar el Repositorio

```bash
git clone https://github.com/atriz-udenar/Atriz_rvr.git
cd Atriz_rvr
```

### 3. Configurar el Entorno

```bash
# Configurar rutas de Python
python3 setup_python_path.py

# Compilar paquetes ROS
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 4. Verificar Instalación

```bash
# Ejecutar diagnóstico
./run_tests.sh diagnostic

# Probar configuración
python3 -c "import sys; sys.path.insert(0, 'atriz_rvr_driver/scripts'); from sphero_sdk_config import setup_sphero_sdk_path; print('✅ Configuración exitosa')"
```

## 🎮 Control de Movimiento

### Sistema Dual de Tópicos

El robot Sphero RVR soporta **dos tópicos de control** que funcionan simultáneamente:

#### 1. Control Estándar ROS (`/cmd_vel`)
Usa radianes/segundo (estándar ROS):

```bash
# Mover hacia adelante a 0.5 m/s
rostopic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10

# Girar a 0.5 rad/s (≈28.6 grados/s)
rostopic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" -r 10

# Parar el robot
rostopic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 1
```

#### 2. Control Directo en Grados (`/cmd_degrees`)
Usa grados/segundo (más intuitivo):

```bash
# Mover hacia adelante a 0.5 m/s
rostopic pub /cmd_degrees sphero_rvr_msgs/DegreesTwist "{linear_x: 0.5, linear_y: 0.0, linear_z: 0.0, angular_x: 0.0, angular_y: 0.0, angular_z: 0.0}" -r 10

# Girar a 30 grados/s
rostopic pub /cmd_degrees sphero_rvr_msgs/DegreesTwist "{linear_x: 0.0, linear_y: 0.0, linear_z: 0.0, angular_x: 0.0, angular_y: 0.0, angular_z: 30.0}" -r 10

# Girar a 90 grados/s
rostopic pub /cmd_degrees sphero_rvr_msgs/DegreesTwist "{linear_x: 0.0, linear_y: 0.0, linear_z: 0.0, angular_x: 0.0, angular_y: 0.0, angular_z: 90.0}" -r 10

# Parar el robot
rostopic pub /cmd_degrees sphero_rvr_msgs/DegreesTwist "{linear_x: 0.0, linear_y: 0.0, linear_z: 0.0, angular_x: 0.0, angular_y: 0.0, angular_z: 0.0}" -r 1
```

## 🔍 Sensores y Datos

### Tópicos de Sensores Disponibles

#### Control de Movimiento
- `/cmd_vel` - Control estándar ROS (radianes/segundo)
- `/cmd_degrees` - Control directo en grados/segundo

#### Sensores y Datos
- `/odom` - Odometría del robot
- `/imu` - Datos del IMU (orientación, aceleración, giroscopio)
- `/ambient_light` - Sensor de luz ambiental
- `/color` - Sensor de color con confianza
- `/sphero_rvr/status` - Estado general del robot

#### Control y Seguridad
- `/is_emergency_stop` - Parada de emergencia
- `/ir_messages` - Mensajes de comunicación IR

### Servicios Disponibles

- `/battery_state` - Estado de la batería
- `/enable_color` - Habilitar/deshabilitar sensor de color
- `/reset_odom` - Reiniciar odometría
- `/release_emergency_stop` - Liberar parada de emergencia
- `/ir_mode` - Configurar modo de comunicación IR

### Ejemplos de Uso de Sensores

```bash
# Obtener lecturas del sensor de color
rostopic echo /color

# Obtener lecturas de odometría
rostopic echo /odom

# Estado de la batería
rosservice call /battery_state

# Lectura de luz ambiental
rostopic echo /ambient_light

# Datos del IMU
rostopic echo /imu

# Habilitar sensor de color
rosservice call /enable_color "{data: true}"

# Reiniciar odometría
rosservice call /reset_odom "{}"
```

## 🎨 Control de LEDs

Para cambiar el color de los LEDs, usa el siguiente comando:

```bash
# Rojo
rosservice call /set_leds "[255, 0, 0]"

# Verde
rosservice call /set_leds "[0, 255, 0]"

# Azul
rosservice call /set_leds "[0, 0, 255]"

# Blanco
rosservice call /set_leds "[255, 255, 255]"
```

## 🚨 Seguridad y Emergencia

### Parada de Emergencia

```bash
# Activar parada de emergencia
rostopic pub /is_emergency_stop std_msgs/Empty "{}"

# Liberar parada de emergencia
rosservice call /release_emergency_stop "{}"
```

## 🧪 Suite de Pruebas

### Scripts de Prueba Disponibles

#### Pruebas Automáticas
```bash
# Ejecutar todas las pruebas automáticamente
./run_tests.sh automated

# Pruebas específicas del driver
python3 testing_scripts/automated/test_atriz_rvr_driver.py

# Pruebas completas
python3 testing_scripts/automated/run_complete_tests.py
```

#### Pruebas Interactivas
```bash
# Pruebas interactivas individuales
./run_tests.sh interactive

# O ejecutar directamente
python3 testing_scripts/interactive/test_individual_functions.py
```

#### Diagnóstico del Sistema
```bash
# Diagnóstico completo
./run_tests.sh diagnostic

# O ejecutar directamente
python3 testing_scripts/diagnostic/diagnose_system.py
```

## 🔧 Configuración Avanzada

### Configuración del Linter

El proyecto incluye configuración completa para linters:

- **Pyright/Pylance**: `pyrightconfig.json`
- **Pylint**: `.pylintrc`
- **VS Code**: `.vscode/settings.json`

### Configuración de Python

```bash
# Configuración automática
python3 setup_python_path.py

# Verificar configuración
python3 -c "from sphero_sdk_config import setup_sphero_sdk_path; print('SDK path:', setup_sphero_sdk_path())"
```

## 📊 Monitoreo del Sistema

### Comandos Útiles

```bash
# Listar todos los tópicos
rostopic list

# Listar todos los servicios
rosservice list

# Ver información de un tópico
rostopic info /cmd_vel

# Ver información de un servicio
rosservice info /battery_state

# Ver nodos activos
rosnode list
```

## 🎯 Ejemplos Prácticos

### Girar 360 grados en 10 segundos
```bash
# 36 grados/s = 360° en 10s
rostopic pub /cmd_degrees sphero_rvr_msgs/DegreesTwist "{linear_x: 0.0, linear_y: 0.0, linear_z: 0.0, angular_x: 0.0, angular_y: 0.0, angular_z: 36.0}" -r 10
```

### Girar 90 grados en 1 segundo
```bash
# 90 grados/s = 90° en 1s
rostopic pub /cmd_degrees sphero_rvr_msgs/DegreesTwist "{linear_x: 0.0, linear_y: 0.0, linear_z: 0.0, angular_x: 0.0, angular_y: 0.0, angular_z: 90.0}" -r 10
```

### Movimiento combinado (adelante + giro)
```bash
rostopic pub /cmd_degrees sphero_rvr_msgs/DegreesTwist "{linear_x: 0.3, linear_y: 0.0, linear_z: 0.0, angular_x: 0.0, angular_y: 0.0, angular_z: 30.0}" -r 10
```

## ⚡ Ventajas del Sistema

### Control Estándar ROS (`/cmd_vel`)
- ✅ Compatible con herramientas ROS estándar
- ✅ Funciona con RViz y otros visualizadores
- ✅ Integración con navegación ROS
- ✅ Usa radianes/segundo (estándar matemático)

### Control Directo en Grados (`/cmd_degrees`)
- ✅ Más intuitivo para humanos
- ✅ Fácil de entender y usar
- ✅ Conversión directa sin cálculos
- ✅ Ideal para control manual y pruebas

### Suite de Pruebas
- ✅ Pruebas automatizadas completas
- ✅ Pruebas interactivas individuales
- ✅ Diagnóstico del sistema
- ✅ Scripts de lanzamiento organizados

### Documentación
- ✅ Guías detalladas por categoría
- ✅ Ejemplos prácticos
- ✅ Configuración paso a paso
- ✅ Solución de problemas

## 🚀 Inicio Rápido Completo

1. **Configurar el entorno:**
   ```bash
   python3 setup_python_path.py
   ```

2. **Iniciar roscore:**
   ```bash
   roscore
   ```

3. **Ejecutar el driver:**
   ```bash
   python3 ./atriz_rvr_driver/scripts/Atriz_rvr_node.py
   ```

4. **Probar control en grados:**
   ```bash
   rostopic pub /cmd_degrees sphero_rvr_msgs/DegreesTwist "{linear_x: 0.0, linear_y: 0.0, linear_z: 0.0, angular_x: 0.0, angular_y: 0.0, angular_z: 30.0}" -r 10
   ```

5. **Ejecutar pruebas:**
   ```bash
   ./run_tests.sh automated
   ```

## 🤝 Contribuciones

Este proyecto es parte de la investigación en la Universidad de Nariño. Para contribuir:

1. Fork el repositorio
2. Crea una rama para tu feature
3. Realiza tus cambios
4. Ejecuta las pruebas: `./run_tests.sh automated`
5. Envía un Pull Request

## 📄 Licencia

Este proyecto está bajo la licencia MIT. Ver el archivo `LICENSE` para más detalles.

## 👥 Autores

- **Universidad de Nariño** - Proyecto Atriz
- **Equipo de Investigación** - Desarrollo del driver ROS

## 📞 Soporte

Para soporte técnico o preguntas sobre el proyecto:

1. **Revisa la documentación** en `docs/README.md`
2. **Ejecuta el diagnóstico**: `./run_tests.sh diagnostic`
3. **Consulta los logs** de ROS
4. **Contacta al equipo** de investigación de la Universidad de Nariño

## 🔄 Mantenimiento

Para mantener el proyecto actualizado:

1. **Mantén READMEs actualizados** cuando agregues funcionalidades
2. **Actualiza documentación** en `docs/`
3. **Revisa enlaces** en la documentación
4. **Mantén scripts organizados** por funcionalidad
5. **Ejecuta pruebas regularmente** con `./run_tests.sh automated`