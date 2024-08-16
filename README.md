# Sphero RVR ROS Driver - Atriz

Este repositorio contiene el código necesario para operar el robot Sphero RVR utilizando ROS (Robot Operating System). Está diseñado como parte de un proyecto para la Universidad de Nariño, facilitando la integración y control del Sphero RVR en entornos de investigación y desarrollo.

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

- `/ambient_light`
- `/cmd_vel`
- `/color`
- `/is_emergency_stop`
- `/magnet`
- `/odom`

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

## Ejemplos de Uso

### Control del LED del Sensor de Color

Para encender o apagar el LED del sensor de color, utiliza:

```bash
rosservice call /enable_color "{data: false/true}"
```

### Obtener Lecturas del Sensor de Color

Asegúrate de haber activado el LED del sensor antes de leer las lecturas:

```bash
rostopic echo /color
```

### Obtener Lecturas de Odometría

Para obtener las lecturas de odometría:

```bash
rostopic echo /odom
```

### Estado de la Batería

Consulta el estado de la batería utilizando:

```bash
rosservice call /battery_state
```

### Control de los LEDs

Para cambiar el color de los LEDs, usa el siguiente comando, donde `[R, G, B]` son valores entre 0 y 255:

```bash
rosservice call /set_leds "[255, 0, 0]"
```

### Control de Movimiento

Para mover el robot, publica en el tópico `/cmd_vel`:

```bash
rostopic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}" -r 10
```

### Lectura de Luz Ambiental

Obtén las lecturas del sensor de luz ambiental:

```bash
rostopic echo /ambient_light
```

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
