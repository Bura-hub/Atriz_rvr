# Guía de Funcionalidades del Driver Atriz RVR

## Descripción General

El driver `Atriz_rvr_node.py` es el nodo principal de ROS para controlar el robot Sphero RVR. Proporciona una interfaz completa entre ROS y la SDK de Sphero, incluyendo control de movimiento, sensores, LEDs, comunicación IR y sistemas de seguridad.

## Tópicos Publicados

### 1. `/odom` (nav_msgs/Odometry)
**Descripción**: Publica la odometría del robot
**Frecuencia**: ~15 Hz
**Contenido**:
- Posición (x, y, z) en metros
- Orientación como quaternion
- Velocidad lineal y angular
- Covarianza de posición y velocidad

**Uso**:
```bash
rostopic echo /odom
```

### 2. `/imu` (sensor_msgs/Imu)
**Descripción**: Publica datos del IMU (Inertial Measurement Unit)
**Frecuencia**: ~15 Hz
**Contenido**:
- Orientación como quaternion
- Velocidad angular (rad/s)
- Aceleración lineal (m/s²)
- Covarianza de mediciones

**Uso**:
```bash
rostopic echo /imu
```

### 3. `/color` (sphero_rvr_msgs/Color)
**Descripción**: Publica datos del sensor de color
**Frecuencia**: Variable (solo cuando está habilitado)
**Contenido**:
- Valores RGB detectados
- Nivel de confianza de la detección

**Uso**:
```bash
rostopic echo /color
```

### 4. `/ambient_light` (sensor_msgs/Illuminance)
**Descripción**: Publica datos del sensor de luz ambiental
**Frecuencia**: ~15 Hz
**Contenido**:
- Intensidad de luz en lux
- Varianza (siempre 0)

**Uso**:
```bash
rostopic echo /ambient_light
```

### 5. `/ir_messages` (std_msgs/String)
**Descripción**: Publica mensajes recibidos por IR
**Frecuencia**: Variable
**Contenido**:
- Mensajes de texto recibidos por comunicación IR

**Uso**:
```bash
rostopic echo /ir_messages
```

## Tópicos Suscritos

### 1. `/cmd_vel` (geometry_msgs/Twist)
**Descripción**: Recibe comandos de velocidad en unidades SI
**Contenido**:
- `linear.x`: Velocidad lineal en m/s
- `angular.z`: Velocidad angular en rad/s

**Uso**:
```bash
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.2
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.5"
```

### 2. `/cmd_degrees` (sphero_rvr_msgs/DegreesTwist)
**Descripción**: Recibe comandos de velocidad en grados/segundo
**Contenido**:
- `linear_x`: Velocidad lineal en m/s
- `angular_z`: Velocidad angular en grados/s

**Uso**:
```bash
rostopic pub /cmd_degrees sphero_rvr_msgs/DegreesTwist "linear_x: 0.2
angular_z: 30.0"
```

### 3. `/is_emergency_stop` (std_msgs/Empty)
**Descripción**: Activa la parada de emergencia
**Uso**:
```bash
rostopic pub /is_emergency_stop std_msgs/Empty
```

## Servicios

### 1. `/enable_color` (std_srvs/SetBool)
**Descripción**: Habilita o deshabilita el sensor de color
**Parámetros**:
- `data`: true para habilitar, false para deshabilitar

**Uso**:
```bash
rosservice call /enable_color "data: true"
```

### 2. `/battery_state` (sphero_rvr_msgs/BatteryState)
**Descripción**: Obtiene el estado de la batería
**Respuesta**:
- `battery_percentage`: Porcentaje de carga (0-100)
- `voltage_state`: Estado de voltaje (texto)

**Uso**:
```bash
rosservice call /battery_state
```

### 3. `/reset_odom` (std_srvs/Empty)
**Descripción**: Reinicia la odometría del robot

**¿Qué hace?**
- Resetea la posición a (0, 0)
- Resetea la orientación a 0°
- Limpia el historial de movimiento

**¿Cuándo usarlo?**
- ✅ Inicio de una misión
- ✅ Después de mover el robot manualmente
- ✅ Calibración y pruebas
- ❌ Durante la navegación (perderías la posición actual)

**Uso**:
```bash
# Reiniciar odometría
rosservice call /reset_odom

# Verificar que se reseteó
rostopic echo /odom
```

### 4. `/release_emergency_stop` (std_srvs/Empty)
**Descripción**: Libera la parada de emergencia
**Uso**:
```bash
rosservice call /release_emergency_stop
```

### 5. `/ir_mode` (sphero_rvr_msgs/SetIRMode)
**Descripción**: Configura el modo de comunicación IR
**Parámetros**:
- `mode`: "broadcast", "following", o "off"
- `far_code`: Código para distancia lejana
- `near_code`: Código para distancia cercana

**Uso**:
```bash
# Modo broadcast
rosservice call /ir_mode "mode: 'broadcast'
far_code: 1
near_code: 2"

# Modo following
rosservice call /ir_mode "mode: 'following'
far_code: 3
near_code: 4"

# Apagar IR
rosservice call /ir_mode "mode: 'off'
far_code: 0
near_code: 0"
```

## Funciones de Movimiento

### 1. Movimiento Básico
```python
# Movimiento hacia adelante
twist = Twist()
twist.linear.x = 0.2  # 0.2 m/s
twist.angular.z = 0.0
cmd_vel_pub.publish(twist)

# Giro a la izquierda
twist.linear.x = 0.0
twist.angular.z = 0.5  # 0.5 rad/s
cmd_vel_pub.publish(twist)
```

### 2. Movimiento con Grados
```python
# Movimiento con velocidad angular en grados
degrees_twist = DegreesTwist()
degrees_twist.linear_x = 0.2  # 0.2 m/s
degrees_twist.angular_z = 30.0  # 30 grados/s
cmd_degrees_pub.publish(degrees_twist)
```

### 3. Parada
```python
# Parar el robot
twist = Twist()
twist.linear.x = 0.0
twist.angular.z = 0.0
cmd_vel_pub.publish(twist)
```

## Sistema de Seguridad

### Parada de Emergencia
```python
# Activar parada de emergencia
emergency_pub.publish(Empty())

# Liberar parada de emergencia
rospy.ServiceProxy('/release_emergency_stop', EmptySrv)()
```

### Timeout de Comandos
- Los comandos de velocidad tienen un timeout de 0.3 segundos
- Si no se reciben comandos nuevos, el robot se detiene automáticamente

## Sistema de Odometría

### ¿Qué es la Odometría?
La **odometría** es el sistema que calcula la **posición y orientación** del robot basándose en:
- Movimiento de las ruedas
- Sensores de rotación
- Datos del IMU (giroscopio, acelerómetro)

### Características del Sistema
- **Precisión**: Buena para distancias cortas
- **Deriva**: Puede acumular error en distancias largas
- **Actualización**: ~15 Hz
- **Formato**: Coordenadas cartesianas (x, y) en metros

### Reinicio de Odometría
```bash
# Reiniciar a posición (0, 0) y orientación 0°
rosservice call /reset_odom

# Verificar posición actual
rostopic echo /odom | grep "x:\|y:"
```

### Casos de Uso
- **Navegación relativa**: Movimientos desde posición actual
- **Navegación absoluta**: Ir a coordenadas específicas
- **Calibración**: Establecer punto de referencia
- **Pruebas**: Punto de partida conocido

## Sensores Disponibles

### 1. Localizador (Locator)
- Proporciona posición (x, y) en metros
- Se actualiza en tiempo real
- Se incluye en el mensaje de odometría

### 2. Giroscopio (Gyroscope)
- Velocidad angular en grados/segundo
- Se convierte a radianes/segundo para ROS
- Se incluye en el mensaje de IMU

### 3. Velocímetro (Velocity)
- Velocidad lineal en m/s
- Se incluye en el mensaje de odometría

### 4. Acelerómetro (Accelerometer)
- Aceleración lineal en m/s²
- Se incluye en el mensaje de IMU

### 5. IMU (Inertial Measurement Unit)
- Orientación como quaternion
- Combina datos de múltiples sensores
- Se publica en el tópico `/imu`

### 6. Sensor de Color
- Detecta colores RGB
- Nivel de confianza de detección
- Debe habilitarse con el servicio `/enable_color`

### 7. Sensor de Luz Ambiental
- Intensidad de luz en lux
- Se publica en el tópico `/ambient_light`

## Comunicación IR

### Modo Broadcast
- El robot emite códigos IR
- Útil para comunicación con otros robots
- Se configura con códigos far_code y near_code

### Modo Following
- El robot sigue códigos IR específicos
- Útil para seguimiento de otros robots
- Se configura con códigos far_code y near_code

### Modo Off
- Desactiva toda la comunicación IR
- Ahorra batería

## Control de LEDs

El driver incluye un sistema de control de LEDs que muestra diferentes patrones según el estado del robot:

- **Startup**: Patrón de arranque
- **Driving**: Patrón durante la conducción
- **Emergency Stop**: LEDs rojos parpadeantes

## Parámetros de Configuración

### `pub_tf` (bool, default: true)
- Controla si se publican transformaciones TF
- Útil para navegación y mapeo

### `cmd_vel_timeout` (float, default: 0.3)
- Timeout en segundos para comandos de velocidad
- Si no se reciben comandos nuevos, el robot se detiene

## Ejemplos de Uso

### 1. Control Básico
```bash
# Mover hacia adelante
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"

# Girar a la izquierda
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}"

# Parar
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"
```

### 2. Verificar Estado
```bash
# Verificar batería
rosservice call /battery_state

# Verificar odometría
rostopic echo /odom

# Verificar IMU
rostopic echo /imu
```

### 3. Configurar Sensores
```bash
# Habilitar sensor de color
rosservice call /enable_color "data: true"

# Configurar IR broadcast
rosservice call /ir_mode "mode: 'broadcast', far_code: 1, near_code: 2"
```

### 4. Parada de Emergencia
```bash
# Activar parada de emergencia
rostopic pub /is_emergency_stop std_msgs/Empty

# Liberar parada de emergencia
rosservice call /release_emergency_stop
```

## Solución de Problemas

### 1. Robot no responde a comandos
- Verificar que no esté en parada de emergencia
- Verificar conexión con el robot
- Verificar que el driver esté ejecutándose

### 2. Datos de sensores no se reciben
- Verificar que los sensores estén habilitados
- Verificar que el robot esté conectado
- Verificar la configuración de streaming

### 3. Comunicación IR no funciona
- Verificar que el modo IR esté configurado correctamente
- Verificar que los códigos IR sean válidos
- Verificar que otros robots estén en rango

### 4. Batería baja
- Verificar estado de batería con el servicio
- Cargar el robot si es necesario
- El robot puede entrar en modo de suspensión automáticamente

## Notas Importantes

1. **Seguridad**: Siempre use el sistema de parada de emergencia
2. **Batería**: Monitoree el estado de la batería regularmente
3. **Conexión**: Asegúrese de que el robot esté conectado antes de enviar comandos
4. **Sensores**: Algunos sensores requieren habilitación explícita
5. **IR**: La comunicación IR tiene alcance limitado
6. **Timeouts**: Los comandos tienen timeout automático por seguridad

## Archivos de Prueba

- `test_atriz_rvr_driver.py`: Pruebas automáticas completas
- `test_individual_functions.py`: Pruebas interactivas individuales
- `launch_tests.sh`: Script para lanzar las pruebas

## Uso de los Scripts de Prueba

```bash
# Ejecutar todas las pruebas automáticamente
./launch_tests.sh all

# Ejecutar pruebas individuales interactivas
./launch_tests.sh individual

# Solo lanzar el driver
./launch_tests.sh driver

# Limpiar procesos
./launch_tests.sh cleanup
```
