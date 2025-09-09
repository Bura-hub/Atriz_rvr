# Sphero RVR ROS Driver - Atriz

Este repositorio contiene el código necesario para operar el robot Sphero RVR utilizando ROS (Robot Operating System). Está diseñado como parte de un proyecto para la Universidad de Nariño, facilitando la integración y control del Sphero RVR en entornos de investigación y desarrollo.

## 🚀 Características Principales

- **Control Dual de Tópicos**: Soporte para `/cmd_vel` (radianes/segundo) y `/cmd_degrees` (grados/segundo)
- **Control Intuitivo**: Comandos directos en grados para facilitar el uso humano
- **Compatibilidad ROS**: Mantiene el estándar ROS con `/cmd_vel`
- **Seguridad Avanzada**: Sistema de parada de emergencia y monitoreo de batería
- **Sensores Integrados**: Odometría, IMU, sensores de color y luz ambiental
- **Control de LEDs**: Sistema completo de control de iluminación
- **Scripts de Prueba**: Suite completa de pruebas automatizadas e interactivas
- **Documentación Organizada**: Guías detalladas por categorías

## 📚 Documentación

Toda la documentación está organizada en la carpeta `docs/`:

- **[Índice de Documentación](docs/README.md)** - Guía principal de documentación
- **[Guía del Driver](docs/driver/DRIVER_FUNCTIONALITY_GUIDE.md)** - Funcionalidades completas
- **[Guía de Pruebas](docs/testing/README_TESTING.md)** - Scripts y testing
- **[Scripts de Prueba](testing_scripts/README.md)** - Pruebas organizadas

## 🚀 Inicio Rápido

### **1. Pruebas Rápidas**
```bash
# Ejecutar todas las pruebas automáticamente
./run_tests.sh automated

# Pruebas interactivas
./run_tests.sh interactive

# Diagnóstico del sistema
./run_tests.sh diagnostic
```

### **2. Ver Ayuda**
```bash
# Ver todas las opciones disponibles
./run_tests.sh help
```

## Requisitos Previos

- ROS Noetic instalado en Ubuntu.
- Python 3.8 o superior.

## Instalación

Sigue los pasos a continuación para configurar tu entorno y ejecutar el nodo principal:

### 1. Crear el Workspace de ROS

Si aún no tienes un workspace creado, ejecuta el siguiente comando para crearlo:

```bash
mkdir -p ~/catkin_ws/src
```

### 2. Clonar el Repositorio

Clona este repositorio dentro de la carpeta `src` de tu workspace:

```bash
cd ~/catkin_ws/src
git clone https://github.com/atriz-udenar/ros_sphero_rvr.git
```

### 3. Compilar los Paquetes de ROS
Compila los paquetes clonados en tu workspace:

```bash
cd ~/catkin_ws/
source devel/setup.sh
catkin_make
```

## Ejecución del Nodo Principal

Para ejecutar el nodo principal y comenzar a interactuar con el Sphero RVR, sigue los pasos a continuación:

### 1. Iniciar el Nodo Maestro de ROS

En la primera terminal, inicia el nodo maestro de ROS:

```bash
roscore
```

### 2. Ejecutar el Nodo Principal

En una segunda terminal, ejecuta el nodo principal para controlar el Sphero RVR:

```bash
rosrun sphero_rvr_hw Atriz_rvr_node.py
```

### 3. Configuración de Permisos de Ejecución

Si experimentas problemas al intentar ejecutar los scripts del nodo, es posible que necesites otorgar permisos de ejecución a los archivos. Para hacerlo, ejecuta los siguientes comandos:

```bash
chmod +x ~/catkin_ws/src/ros_sphero_rvr/sphero_rvr_hw/scripts/Atriz_rvr_node.py
chmod +x ~/catkin_ws/src/ros_sphero_rvr/sphero_rvr/scripts/rvr_joystick_control.py
chmod +x ~/catkin_ws/src/ros_sphero_rvr/sphero_rvr/scripts/color_listener.py
```

## Ejecución de los Nodos Adicionales

Para ejecutar otros nodos disponibles en el repositorio, sigue los pasos a continuación:

### Control del Joystick

Ejecuta el siguiente comando para iniciar el nodo de control del joystick:

```bash
rosrun sphero_rvr rvr_joystick_control.py
```

> **Nota:** Este nodo requiere que el paquete **`joy_node`** esté instalado. Puedes seguir [este tutorial](https://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick) para configurarlo.

```bash
rosrun joy joy_node
```

### Listener de Color

```bash
rosrun sphero_rvr color_listener.py
```

## Acceso a Tópicos y Servicios

Puedes listar los tópicos y servicios disponibles ejecutando los siguientes comandos:

```bash
rostopic list
rosservice list
```

### Tópicos Disponibles:

#### Control de Movimiento
- `/cmd_vel` - Control estándar ROS (radianes/segundo)
- `/cmd_degrees` - Control directo en grados/segundo (más intuitivo)

#### Sensores y Datos
- `/ambient_light` - Sensor de luz ambiental
- `/color` - Sensor de color
- `/magnet` - Magnetómetro
- `/odom` - Odometría del robot
- `/imu` - Datos del IMU
- `/battery_state` - Estado de la batería

#### Control y Seguridad
- `/is_emergency_stop` - Parada de emergencia
- `/sphero_rvr/status` - Estado general del robot

### Servicios Disponibles

- `/battery_state`
- `/calibrate_magnetometer`
- `/enable_color`
- `/move_to_pos_and_yaw`
- `/move_to_pose`
- `/release_emergency_stop`
- `/reset_odom`
- `/rvr_ros_interface/get_loggers`
- `/rvr_ros_interface/set_logger_level`
- `/set_leds`
- `/trigger_led_event`

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

### Scripts de Ejemplo

#### Script Interactivo de Grados
```bash
python3 /home/sphero/atriz_git/src/ros_sphero_rvr/sphero_rvr_hw/scripts/degrees_control_example.py
```

#### Script de Prueba de Ambos Tópicos
```bash
python3 /home/sphero/atriz_git/src/ros_sphero_rvr/sphero_rvr_hw/scripts/test_both_topics.py
```

## 🔧 Sensores y Datos

### Control del LED del Sensor de Color

Para encender o apagar el LED del sensor de color:

```bash
rosservice call /enable_color "{data: true}"
```

### Obtener Lecturas del Sensor de Color

```bash
rostopic echo /color
```

### Obtener Lecturas de Odometría

```bash
rostopic echo /odom
```

### Estado de la Batería

```bash
rosservice call /battery_state
```

### Lectura de Luz Ambiental

```bash
rostopic echo /ambient_light
```

### Datos del IMU

```bash
rostopic echo /imu
```

## 🎨 Control de LEDs

Para cambiar el color de los LEDs, usa el siguiente comando, donde `[R, G, B]` son valores entre 0 y 255:

```bash
rosservice call /set_leds "[255, 0, 0]"  # Rojo
rosservice call /set_leds "[0, 255, 0]"  # Verde
rosservice call /set_leds "[0, 0, 255]"  # Azul
```

## 🚨 Seguridad y Emergencia

### Parada de Emergencia

Para activar la parada de emergencia:

```bash
rostopic pub /is_emergency_stop std_msgs/Empty "{}"
```

### Liberar Parada de Emergencia

Para liberar la parada de emergencia:

```bash
rosservice call /release_emergency_stop "{}"
```

## 📊 Monitoreo del Sistema

### Estado General del Robot

```bash
rostopic echo /sphero_rvr/status
```

### Listar Todos los Tópicos

```bash
rostopic list
```

### Listar Todos los Servicios

```bash
rosservice list
```

## 📚 Documentación Adicional

### Mensaje Personalizado: DegreesTwist

El sistema incluye un mensaje personalizado `DegreesTwist` que permite control directo en grados/segundo:

```msg
# sphero_rvr_msgs/DegreesTwist.msg
float32 linear_x    # Velocidad lineal en m/s
float32 linear_y    # Velocidad lineal en m/s (no usado)
float32 linear_z    # Velocidad lineal en m/s (no usado)
float32 angular_x   # Velocidad angular en grados/s (no usado)
float32 angular_y   # Velocidad angular en grados/s (no usado)
float32 angular_z   # Velocidad angular en grados/s (PRINCIPAL)
```

### Ejemplos Prácticos de Movimiento

#### Girar 360 grados en 10 segundos
```bash
# 36 grados/s = 360° en 10s
rostopic pub /cmd_degrees sphero_rvr_msgs/DegreesTwist "{linear_x: 0.0, linear_y: 0.0, linear_z: 0.0, angular_x: 0.0, angular_y: 0.0, angular_z: 36.0}" -r 10
```

#### Girar 90 grados en 1 segundo
```bash
# 90 grados/s = 90° en 1s
rostopic pub /cmd_degrees sphero_rvr_msgs/DegreesTwist "{linear_x: 0.0, linear_y: 0.0, linear_z: 0.0, angular_x: 0.0, angular_y: 0.0, angular_z: 90.0}" -r 10
```

#### Movimiento combinado (adelante + giro)
```bash
rostopic pub /cmd_degrees sphero_rvr_msgs/DegreesTwist "{linear_x: 0.3, linear_y: 0.0, linear_z: 0.0, angular_x: 0.0, angular_y: 0.0, angular_z: 30.0}" -r 10
```

## 🔧 Estructura del Proyecto

```
ros_sphero_rvr/
├── sphero_rvr_hw/                 # Paquete principal de hardware
│   ├── scripts/
│   │   ├── Atriz_rvr_node.py     # Nodo principal
│   │   ├── degrees_control_example.py  # Ejemplo interactivo
│   │   ├── test_both_topics.py   # Prueba de ambos tópicos
│   │   └── GRADOS_DIRECTOS_README.md  # Documentación detallada
│   └── ...
├── sphero_rvr_msgs/               # Mensajes personalizados
│   └── msg/
│       └── DegreesTwist.msg      # Mensaje para grados/segundo
└── ...
```

## ⚡ Ventajas del Sistema Dual

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

## 🚀 Inicio Rápido

1. **Iniciar roscore:**
   ```bash
   roscore
   ```

2. **Ejecutar el nodo principal:**
   ```bash
   cd ~/atriz_git && source devel/setup.sh && rosrun sphero_rvr_hw Atriz_rvr_node.py
   ```

3. **Probar control en grados:**
   ```bash
   rostopic pub /cmd_degrees sphero_rvr_msgs/DegreesTwist "{linear_x: 0.0, linear_y: 0.0, linear_z: 0.0, angular_x: 0.0, angular_y: 0.0, angular_z: 30.0}" -r 10
   ```

4. **Probar control estándar:**
   ```bash
   rostopic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" -r 10
   ```

## 🤝 Contribuciones

Este proyecto es parte de la investigación en la Universidad de Nariño. Para contribuir:

1. Fork el repositorio
2. Crea una rama para tu feature
3. Realiza tus cambios
4. Envía un Pull Request

## 📄 Licencia

Este proyecto está bajo la licencia MIT. Ver el archivo `LICENSE` para más detalles.

## 👥 Autores

- **Universidad de Nariño** - Proyecto Atriz
- **Equipo de Investigación** - Desarrollo del driver ROS

## 📞 Soporte

Para soporte técnico o preguntas sobre el proyecto, contacta al equipo de investigación de la Universidad de Nariño.
