# Guía de Pruebas del Driver Atriz RVR

Esta guía contiene información completa sobre cómo probar todas las funcionalidades del driver `Atriz_rvr_node.py` del robot Sphero RVR.

## 📁 Scripts de Prueba Organizados

Los scripts de prueba están organizados en la carpeta `testing_scripts/`:

```
testing_scripts/
├── automated/          # Pruebas automáticas
├── interactive/        # Pruebas interactivas
├── diagnostic/         # Scripts de diagnóstico
├── launch/            # Scripts de lanzamiento
└── README.md          # Guía de uso de scripts
```

## 🚀 Uso Rápido

### **Script Principal**
```bash
# Desde la raíz del proyecto
./run_tests.sh [opción]

# Opciones disponibles:
./run_tests.sh automated    # Pruebas automáticas
./run_tests.sh interactive  # Pruebas interactivas
./run_tests.sh diagnostic   # Diagnóstico del sistema
./run_tests.sh complete     # Suite completa
./run_tests.sh launch all   # Lanzamiento avanzado
./run_tests.sh help         # Ayuda
```

### **Scripts Específicos**
```bash
# Pruebas automáticas
cd testing_scripts/automated
python3 test_atriz_rvr_driver.py

# Pruebas interactivas
cd testing_scripts/interactive
python3 test_individual_functions.py

# Diagnóstico
cd testing_scripts/diagnostic
python3 diagnose_system.py
```

## 📋 Tópicos Probados

### **Publishers (Datos enviados por el driver)**
- `/odom` - Odometría del robot
- `/imu` - Datos del IMU
- `/color` - Datos del sensor de color
- `/ambient_light` - Datos de iluminancia
- `/ir_messages` - Mensajes IR recibidos

### **Subscribers (Comandos enviados al driver)**
- `/cmd_vel` - Comandos de velocidad en unidades SI
- `/cmd_degrees` - Comandos de velocidad en grados
- `/is_emergency_stop` - Activación de parada de emergencia

## 🔧 Servicios Probados

- `/enable_color` - Habilitar/deshabilitar sensor de color
- `/battery_state` - Obtener estado de batería
- `/reset_odom` - Reiniciar odometría
- `/release_emergency_stop` - Liberar parada de emergencia
- `/ir_mode` - Configurar modo IR (broadcast/following/off)

## 🤖 Funcionalidades de Movimiento Probadas

1. **Movimiento hacia adelante**:
   ```bash
   rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"
   ```

2. **Movimiento hacia atrás**:
   ```bash
   rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: -0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"
   ```

3. **Giro a la izquierda**:
   ```bash
   rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}"
   ```

4. **Giro a la derecha**:
   ```bash
   rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.5}"
   ```

5. **Parada**:
   ```bash
   rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"
   ```

## 🚨 Sistema de Seguridad Probado

1. **Parada de Emergencia**:
   ```bash
   rostopic pub /is_emergency_stop std_msgs/Empty
   ```

2. **Liberación de Parada**:
   ```bash
   rosservice call /release_emergency_stop
   ```

3. **Timeout de Comandos**: Verificación automática

## 📡 Funcionalidad IR Probada

1. **Modo Broadcast**:
   ```bash
   rosservice call /ir_mode "mode: 'broadcast', far_code: 1, near_code: 2"
   ```

2. **Modo Following**:
   ```bash
   rosservice call /ir_mode "mode: 'following', far_code: 3, near_code: 4"
   ```

3. **Modo Off**:
   ```bash
   rosservice call /ir_mode "mode: 'off', far_code: 0, near_code: 0"
   ```

## 🔄 Reinicio de Odometría

### ¿Qué es la Odometría?
La **odometría** es el sistema que calcula la **posición y orientación** del robot basándose en:
- Movimiento de las ruedas
- Sensores de rotación  
- Datos del IMU (giroscopio, acelerómetro)

### ¿Qué Hace "Reiniciar Odometría"?
Cuando reinicias la odometría, el robot:

1. **Resetea la Posición a (0, 0)**
   ```bash
   # Antes del reset
   Posición: x = 1.5m, y = 2.3m
   
   # Después del reset  
   Posición: x = 0.0m, y = 0.0m
   ```

2. **Resetea la Orientación a 0°**
   ```bash
   # Antes del reset
   Orientación: 45° (apuntando al noreste)
   
   # Después del reset
   Orientación: 0° (apuntando al norte)
   ```

3. **Limpia el Historial de Movimiento**
   - Borra todos los datos de posición anteriores
   - El robot "olvida" dónde estaba antes
   - Comienza a contar desde cero

### ¿Cuándo Usar Reiniciar Odometría?

#### ✅ **Casos Útiles:**
1. **Inicio de una misión**: Cuando quieres que el robot comience desde una posición conocida
2. **Después de mover el robot manualmente**: Si lo levantas y lo pones en otro lugar
3. **Calibración**: Cuando quieres establecer un punto de referencia
4. **Navegación absoluta**: Antes de enviar comandos de movimiento a posiciones específicas
5. **Pruebas de movimiento**: Para tener un punto de partida conocido

#### ❌ **Casos donde NO usarlo:**
1. **Durante la navegación**: Perderías la posición actual
2. **Si el robot está en movimiento**: Podría causar confusión
3. **En misiones de seguimiento**: Perderías el rastro de la ruta
4. **Después de una ruta larga**: Perderías todo el progreso

### Cómo Usar el Reinicio de Odometría

#### **Método 1: Servicio ROS**
```bash
# Reiniciar odometría
rosservice call /reset_odom

# Verificar que se reseteó
rostopic echo /odom
```

#### **Método 2: Script de Pruebas**
```bash
# Usar el script interactivo
python3 test_individual_functions.py
# Seleccionar opción 9 (Reiniciar odometría)
```

#### **Método 3: Script de Pruebas Original**
```bash
# Usar el script original
python3 test_individual_functions.py
# Seleccionar opción 10 (Reiniciar odometría)
```

### Verificar el Reinicio

#### **Antes del Reset:**
```bash
rostopic echo /odom
# Salida esperada:
# position: 
#   x: 2.5
#   y: 1.8
#   z: 0.0
```

#### **Después del Reset:**
```bash
rostopic echo /odom
# Salida esperada:
# position: 
#   x: 0.0
#   y: 0.0
#   z: 0.0
```

### Ejemplo Práctico

```bash
# 1. Ver posición actual
rostopic echo /odom | grep "x:\|y:"
# x: 1.234
# y: 0.567

# 2. Reiniciar odometría
rosservice call /reset_odom

# 3. Ver nueva posición
rostopic echo /odom | grep "x:\|y:"
# x: 0.000
# y: 0.000

# 4. Mover el robot
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"

# 5. Ver nueva posición después del movimiento
rostopic echo /odom | grep "x:\|y:"
# x: 0.200  (se movió 20cm hacia adelante)
# y: 0.000
```

### ⚠️ **Importante**

- **Es irreversible**: No puedes recuperar la posición anterior
- **Afecta la navegación**: Si el robot estaba siguiendo una ruta, la perderá
- **Útil para calibración**: Es la forma de establecer un punto de referencia conocido
- **Solo cuando el robot está quieto**: No lo uses mientras se mueve

### Casos de Uso Comunes

1. **Robot en base de carga**: Reset antes de comenzar una misión
2. **Navegación absoluta**: Cuando quieres ir a coordenadas específicas
3. **Calibración de sensores**: Después de mover el robot manualmente
4. **Pruebas de movimiento**: Para tener un punto de partida conocido
5. **Inicio de sesión**: Al comenzar a trabajar con el robot

## 📊 Reportes Generados

Los scripts generan reportes en formato JSON en `/tmp/`:

- `atriz_rvr_test_report.json` - Reporte de pruebas básicas
- `atriz_rvr_diagnostic_report.json` - Reporte de diagnóstico
- `atriz_rvr_complete_test_report.json` - Reporte completo

## 🚨 Solución de Problemas

### 1. Robot no responde
- Verificar conexión Bluetooth
- Verificar que el driver esté ejecutándose
- Verificar parada de emergencia

### 2. Datos de sensores no se reciben
- Verificar que el robot esté conectado
- Verificar configuración de streaming
- Verificar habilitación de sensores

### 3. Servicios no disponibles
- Verificar que el driver esté ejecutándose
- Verificar conexión con el robot
- Verificar configuración de ROS

### 4. Movimiento no funciona
- Verificar parada de emergencia
- Verificar comandos de velocidad
- Verificar estado de batería

## 📋 Comandos Útiles

### Verificar tópicos
```bash
rostopic list
rostopic echo /odom
rostopic echo /imu
```

### Verificar servicios
```bash
rosservice list
rosservice call /battery_state
```

### Verificar nodos
```bash
rosnode list
rosnode info /driver_rvr
```

### Verificar parámetros
```bash
rosparam list
rosparam get /emergency_stop
```

## 📚 Documentación Adicional

- **[Guía de Funcionalidades del Driver](driver/DRIVER_FUNCTIONALITY_GUIDE.md)** - Funcionalidades completas
- **[Scripts de Prueba](testing_scripts/README.md)** - Guía de uso de scripts
- **[Índice de Documentación](README.md)** - Guía principal

## 🔄 Actualizaciones

Para mantener los scripts actualizados:

1. **Verificar dependencias**: Asegúrate de que todas las dependencias estén instaladas
2. **Probar scripts**: Ejecuta las pruebas regularmente
3. **Revisar logs**: Monitorea los logs de ROS para errores
4. **Actualizar documentación**: Mantén la documentación actualizada

## 📞 Soporte

Para problemas o preguntas sobre las pruebas:
1. Revisar los logs de ROS
2. Verificar los reportes generados
3. Ejecutar el diagnóstico del sistema
4. Consultar la documentación del driver
