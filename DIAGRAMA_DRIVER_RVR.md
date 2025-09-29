# Diagrama del Nodo /driver_rvr

## Información del Nodo
- **Nombre**: `/driver_rvr`
- **PID**: 32079
- **Puerto**: http://ubuntu:36711/
- **Estado**: Activo

## Arquitectura del Nodo

```
┌─────────────────────────────────────────────────────────────────┐
│                        NODO /driver_rvr                        │
│                    (Sphero RVR Driver)                         │
└─────────────────────────────────────────────────────────────────┘
```

## Tópicos Publicados (9 tópicos)

```
┌─────────────────────────────────────────────────────────────────┐
│                      TÓPICOS PUBLICADOS                        │
├─────────────────────────────────────────────────────────────────┤
│ /ambient_light      - sensor_msgs/Illuminance                  │
│ /color              - atriz_rvr_msgs/Color                     │
│ /encoders           - atriz_rvr_msgs/Encoder                    │
│ /imu                - sensor_msgs/Imu                           │
│ /infrared_messages  - atriz_rvr_msgs/InfraredMessage          │
│ /ir_messages        - std_msgs/String                           │
│ /odom               - nav_msgs/Odometry                        │
│ /rosout             - rosgraph_msgs/Log                        │
│ /tf                 - tf2_msgs/TFMessage                       │
└─────────────────────────────────────────────────────────────────┘
```

## Tópicos Suscritos (5 tópicos)

```
┌─────────────────────────────────────────────────────────────────┐
│                     TÓPICOS SUSCRITOS                          │
├─────────────────────────────────────────────────────────────────┤
│ /cmd_degrees        - atriz_rvr_msgs/DegreesTwist              │
│ /cmd_vel            - geometry_msgs/Twist                      │
│ /is_emergency_stop  - std_msgs/Empty                           │
│ /tf                 - tf2_msgs/TFMessage                       │
│ /tf_static          - tf2_msgs/TFMessage                       │
└─────────────────────────────────────────────────────────────────┘
```

## Servicios Ofrecidos (21 servicios)

```
┌─────────────────────────────────────────────────────────────────┐
│                      SERVICIOS OFRECIDOS                       │
├─────────────────────────────────────────────────────────────────┤
│ /battery_state                  - Estado de batería            │
│ /configure_streaming            - Configurar streaming         │
│ /driver_rvr/get_loggers         - Obtener loggers              │
│ /driver_rvr/set_logger_level    - Configurar nivel de log     │
│ /enable_color                   - Habilitar sensor de color    │
│ /get_control_state              - Obtener estado de control    │
│ /get_encoders                   - Obtener encoders             │
│ /get_rgbc_sensor_values         - Obtener valores RGB          │
│ /get_system_info                - Obtener información sistema │
│ /ir_mode                        - Modo infrarrojo              │
│ /raw_motors                     - Motores en crudo            │
│ /release_emergency_stop         - Liberar parada emergencia    │
│ /reset_odom                     - Resetear odometría           │
│ /send_infrared_message          - Enviar mensaje infrarrojo    │
│ /set_drive_parameters           - Configurar parámetros drive │
│ /set_ir_evading                 - Configurar evasión IR       │
│ /set_led_rgb                    - Configurar LED RGB           │
│ /set_leds                       - Configurar LEDs             │
│ /set_multiple_leds              - Configurar múltiples LEDs   │
│ /start_streaming                - Iniciar streaming           │
│ /trigger_led_event              - Disparar evento LED          │
└─────────────────────────────────────────────────────────────────┘
```

## Diagrama de Flujo de Datos

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   COMANDOS      │    │   /driver_rvr   │    │   SENSORES     │
│                 │    │                 │    │                 │
│ /cmd_vel        │───▶│                 │───▶│ /odom           │
│ (Twist)         │    │                 │    │ (Odometría)     │
│                 │    │                 │    │                 │
│ /cmd_degrees    │───▶│                 │───▶│ /imu            │
│ (DegreesTwist)  │    │                 │    │ (IMU)           │
│                 │    │                 │    │                 │
│ /is_emergency_  │───▶│                 │───▶│ /ambient_light  │
│ stop            │    │                 │    │ (Luz ambiente)  │
│                 │    │                 │    │                 │
│ /tf             │───▶│                 │───▶│ /color          │
│ (Transform)     │    │                 │    │ (Color)         │
│                 │    │                 │    │                 │
│ /tf_static      │───▶│                 │───▶│ /encoders       │
│ (Transform)     │    │                 │    │ (Encoders)     │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

## Diagrama de Servicios

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   CLIENTES      │    │   /driver_rvr   │    │   FUNCIONES     │
│                 │    │                 │    │                 │
│ Control LED     │───▶│ /set_led_rgb    │───▶│ Control LEDs    │
│                 │    │ /set_leds       │    │                 │
│                 │    │ /set_multiple_  │    │                 │
│                 │    │ leds            │    │                 │
│                 │    │                 │    │                 │
│ Streaming       │───▶│ /start_streaming│───▶│ Iniciar stream  │
│                 │    │ /configure_     │    │                 │
│                 │    │ streaming       │    │                 │
│                 │    │                 │    │                 │
│ Motores         │───▶│ /raw_motors     │───▶│ Control motores │
│                 │    │ /set_drive_     │    │                 │
│                 │    │ parameters      │    │                 │
│                 │    │                 │    │                 │
│ Sensores        │───▶│ /get_encoders   │───▶│ Leer sensores  │
│                 │    │ /get_rgbc_      │    │                 │
│                 │    │ sensor_values   │    │                 │
│                 │    │                 │    │                 │
│ Sistema         │───▶│ /get_system_info │───▶│ Info sistema    │
│                 │    │ /battery_state  │    │                 │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

## Conexiones de Red

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   /driver_rvr   │    │   ROS MASTER    │    │   CLIENTES     │
│                 │    │                 │    │                 │
│ Puerto: 36711   │◀──▶│                 │◀──▶│ Puerto: 44112   │
│ IP: 192.168.1.19│    │                 │    │ IP: 192.168.1.19│
│                 │    │                 │    │                 │
│ TCPROS          │    │                 │    │ TCPROS          │
│ Transport       │    │                 │    │ Transport       │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

## Funcionalidades Principales

### **1. Control de Movimiento**
- Recibe comandos de velocidad (`/cmd_vel`)
- Recibe comandos en grados (`/cmd_degrees`)
- Publica odometría (`/odom`)
- Control de motores (`/raw_motors`)

### **2. Sensores**
- IMU (`/imu`)
- Sensor de color (`/color`)
- Sensor de luz ambiente (`/ambient_light`)
- Encoders (`/encoders`)
- Mensajes infrarrojos (`/infrared_messages`)

### **3. Control de LEDs**
- LED RGB (`/set_led_rgb`)
- Múltiples LEDs (`/set_leds`, `/set_multiple_leds`)
- Eventos LED (`/trigger_led_event`)

### **4. Streaming de Datos**
- Configurar streaming (`/configure_streaming`)
- Iniciar streaming (`/start_streaming`)
- Habilitar color (`/enable_color`)

### **5. Sistema y Diagnóstico**
- Estado de batería (`/battery_state`)
- Información del sistema (`/get_system_info`)
- Estado de control (`/get_control_state`)
- Logs (`/driver_rvr/get_loggers`)

### **6. Seguridad**
- Parada de emergencia (`/is_emergency_stop`)
- Liberar parada (`/release_emergency_stop`)

## Comandos de Prueba

### **Probar Tópicos:**
```bash
# Ver odometría
rostopic echo /odom

# Ver IMU
rostopic echo /imu

# Ver color
rostopic echo /color

# Ver encoders
rostopic echo /encoders
```

### **Probar Servicios:**
```bash
# Estado de batería
rosservice call /battery_state

# Información del sistema
rosservice call /get_system_info

# Configurar LED
rosservice call /set_led_rgb "r: 1.0, g: 0.0, b: 0.0"

# Iniciar streaming
rosservice call /start_streaming
```

### **Enviar Comandos:**
```bash
# Comando de velocidad
rostopic pub /cmd_vel geometry_msgs/Twist "
linear:
  x: 0.3
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" -r 10
```

## Resumen

El nodo `/driver_rvr` es el **driver principal** del Sphero RVR que:

✅ **Publica 9 tópicos** con datos de sensores y odometría  
✅ **Suscribe 5 tópicos** para recibir comandos y transformaciones  
✅ **Ofrece 21 servicios** para control granular del robot  
✅ **Maneja comunicación TCPROS** en puerto 36711  
✅ **Proporciona interfaz completa** para control del Sphero RVR  

Es el **núcleo de comunicación** entre ROS y el hardware del Sphero RVR.
