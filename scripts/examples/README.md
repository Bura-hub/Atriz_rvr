# Ejemplos de Funcionamiento del Sphero RVR

Esta carpeta contiene scripts de ejemplo que demuestran las diferentes funcionalidades implementadas en el nodo ROS del Sphero RVR.

## 📋 Lista de Ejemplos

### 1. Control de Movimiento
- **`example_degrees_control.py`** - Control del robot usando grados/segundo
  - Demuestra cómo enviar comandos de velocidad
  - Incluye ejemplos de movimiento hacia adelante, giros y movimientos combinados
  - Usa el tópico `/cmd_vel` para controlar el robot

### 2. Encoders
- **`example_encoder_test.py`** - Prueba de funcionalidad de encoders
  - Demuestra cómo leer los valores de los encoders
  - Mueve el robot y observa los cambios en los encoders
  - Explica cómo interpretar los datos de los encoders
  - Usa el servicio `/get_encoders` para leer datos

## 🚀 Cómo usar los ejemplos

### Prerrequisitos
1. Asegúrate de que `roscore` esté ejecutándose
2. Asegúrate de que el nodo `Atriz_rvr_node` esté ejecutándose
3. Asegúrate de que el robot Sphero RVR esté conectado

### Ejecutar un ejemplo
```bash
# Navegar al directorio del workspace
cd /home/sphero/atriz_git

# Configurar el entorno ROS
source devel/setup.bash

# Ejecutar un ejemplo
python3 ./src/Atriz_rvr/scripts/examples/example_encoder_test.py
```

### Verificar que todo esté funcionando
```bash
# Verificar que roscore esté ejecutándose
rosnode list

# Verificar que el nodo del RVR esté ejecutándose
rosnode list | grep driver_rvr

# Verificar que los servicios estén disponibles
rosservice list | grep encoder

# Verificar que los tópicos estén disponibles
rostopic list | grep cmd_vel
```

## 📚 Funcionalidades Implementadas

### ✅ Funcionalidades Disponibles
1. **Encoders** - Lectura de rotación de ruedas
2. **Control de motores raw** - Control directo de motores
3. **Mensajes IR personalizados** - Comunicación por infrarrojos
4. **Modos IR adicionales** - Diferentes modos de comunicación IR
5. **Control individual de LEDs** - Control de LEDs individuales
6. **Información del sistema** - Datos del firmware y hardware
7. **Estado de control** - Estado actual del robot
8. **Parámetros de control** - Configuración de parámetros de movimiento
9. **Configuración de streaming** - Configuración de sensores en tiempo real

### ❌ Funcionalidades No Disponibles
- **Magnetómetro** - No disponible en esta versión del firmware

## 🔧 Servicios ROS Disponibles

| Servicio | Descripción |
|----------|-------------|
| `/get_encoders` | Obtener valores actuales de los encoders |
| `/raw_motors` | Control directo de motores |
| `/send_infrared_message` | Enviar mensaje por infrarrojos |
| `/set_ir_evading` | Configurar modo de evasión IR |
| `/set_led_rgb` | Controlar LED individual |
| `/set_multiple_leds` | Controlar múltiples LEDs |
| `/get_system_info` | Obtener información del sistema |
| `/get_control_state` | Obtener estado de control |
| `/set_drive_parameters` | Configurar parámetros de movimiento |
| `/configure_streaming` | Configurar streaming de sensores |
| `/start_streaming` | Iniciar streaming de sensores |

## 📖 Tópicos ROS Disponibles

| Tópico | Tipo | Descripción |
|--------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Comandos de velocidad |
| `/cmd_degrees` | `atriz_rvr_msgs/DegreesTwist` | Comandos de velocidad en grados |
| `/encoders` | `atriz_rvr_msgs/Encoder` | Datos de encoders en tiempo real |
| `/odom` | `nav_msgs/Odometry` | Odometría del robot |
| `/imu` | `sensor_msgs/Imu` | Datos del IMU |
| `/ambient_light` | `sensor_msgs/Illuminance` | Luz ambiental |
| `/color` | `atriz_rvr_msgs/Color` | Color detectado |
| `/infrared_messages` | `atriz_rvr_msgs/InfraredMessage` | Mensajes IR recibidos |

## 🐛 Solución de Problemas

### El robot no se mueve
- Verifica que el nodo `Atriz_rvr_node` esté ejecutándose
- Verifica que el robot esté conectado y despierto
- Usa el tópico `/cmd_vel` (no `/sphero_rvr/cmd_vel`)

### Los servicios no responden
- Verifica que `roscore` esté ejecutándose
- Verifica que el nodo `Atriz_rvr_node` esté ejecutándose
- Espera unos segundos para que el nodo se inicialice completamente

### Errores de conexión
- Verifica que el robot esté encendido
- Verifica que la conexión USB esté estable
- Reinicia el nodo si es necesario

## 📝 Notas Importantes

- Los encoders miden la rotación en "pasos" (no en metros)
- Los valores positivos indican rotación hacia adelante
- Los valores negativos indican rotación hacia atrás
- La diferencia entre ruedas indica el giro del robot
- Para calcular distancias, necesitas conocer el diámetro de las ruedas
