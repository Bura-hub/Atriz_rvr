# Control Directo con Grados/Segundo - Sphero RVR

## Descripción General

El sistema ahora soporta comandos **directamente en grados/segundo** a través del tópico `/cmd_degrees`, eliminando la necesidad de conversión de radianes. Esto hace el control mucho más intuitivo y directo.

## Nuevo Tópico: `/cmd_degrees`

### Mensaje: `DegreesTwist`
```yaml
# Comando de velocidad en grados/segundo para Sphero RVR
float32 linear_x    # Velocidad lineal en m/s
float32 linear_y    # Velocidad lineal en m/s (no usado)
float32 linear_z    # Velocidad lineal en m/s (no usado)
float32 angular_x   # Velocidad angular en grados/s (no usado)
float32 angular_y   # Velocidad angular en grados/s (no usado)
float32 angular_z   # Velocidad angular en grados/s (PRINCIPAL)
```

## Comandos Directos con Grados

### 1. Girar 90 grados por segundo
```bashesstopic pub /cmd_degrees atriz_rvr_msgs/DegreesTwist "{linear_x: 0.0, linear_y: 0.0, linear_z: 0.0, angular_x: 0.0, angular_y: 0.0, angular_z: 90.0}" -r 10
```

### 2. Girar 45 grados por segundo
```bash
rostopic pub /cmd_degrees atriz_rvr_msgs/DegreesTwist "{linear_x: 0.0, linear_y: 0.0, linear_z: 0.0, angular_x: 0.0, angular_y: 0.0, angular_z: 45.0}" -r 10
```

### 3. Girar 10 grados por segundo
```bash
rostopic pub /cmd_degrees atriz_rvr_msgs/DegreesTwist "{linear_x: 0.0, linear_y: 0.0, linear_z: 0.0, angular_x: 0.0, angular_y: 0.0, angular_z: 10.0}" -r 10
```

### 4. Mover hacia adelante a 0.5 m/s
```bash
rostopic pub /cmd_degrees atriz_rvr_msgs/DegreesTwist "{linear_x: 0.5, linear_y: 0.0, linear_z: 0.0, angular_x: 0.0, angular_y: 0.0, angular_z: 0.0}" -r 10
```

### 5. Movimiento combinado (adelante + giro)
```bash
rostopic pub /cmd_degrees atriz_rvr_msgs/DegreesTwist "{linear_x: 0.3, linear_y: 0.0, linear_z: 0.0, angular_x: 0.0, angular_y: 0.0, angular_z: 30.0}" -r 10
```

### 6. Parar el robot
```bash
rostopic pub /cmd_degrees atriz_rvr_msgs/DegreesTwist "{linear_x: 0.0, linear_y: 0.0, linear_z: 0.0, angular_x: 0.0, angular_y: 0.0, angular_z: 0.0}" -r 1
```

## Ejemplos Prácticos

### Girar 360 grados en 10 segundos
```bash
# 36 grados/s = 360° en 10s
rostopic pub /cmd_degrees atriz_rvr_msgs/DegreesTwist "{linear_x: 0.0, linear_y: 0.0, linear_z: 0.0, angular_x: 0.0, angular_y: 0.0, angular_z: 36.0}" -r 10
```

### Girar 90 grados en 1 segundo
```bash
# 90 grados/s = 90° en 1s
rostopic pub /cmd_degrees atriz_rvr_msgs/DegreesTwist "{linear_x: 0.0, linear_y: 0.0, linear_z: 0.0, angular_x: 0.0, angular_y: 0.0, angular_z: 90.0}" -r 10
```

### Girar 1 grado por segundo (muy lento)
```bash
# 1 grado/s = 360° en 6 minutos
rostopic pub /cmd_degrees atriz_rvr_msgs/DegreesTwist "{linear_x: 0.0, linear_y: 0.0, linear_z: 0.0, angular_x: 0.0, angular_y: 0.0, angular_z: 1.0}" -r 10
```

## Script de Ejemplo Interactivo

```bash
python3 /home/sphero/atriz_git/src/Atriz_rvr/sphero_rvr_hw/scripts/degrees_control_example.py
```

## Comparación de Tópicos

| Tópico | Unidades | Ventajas | Uso |
|--------|----------|----------|-----|
| `/cmd_vel` | Radianes/s | Estándar ROS | Compatibilidad |
| `/cmd_degrees` | Grados/s | Más intuitivo | Control directo |

## Ventajas del Tópico `/cmd_degrees`

1. **🎯 Más intuitivo**: Grados son más fáciles de entender
2. **⚡ Sin conversión**: Directamente en grados/segundo
3. **🧠 Más preciso**: Sin errores de conversión
4. **🔧 Más simple**: Menos cálculos mentales
5. **📐 Consistente**: Coincide con la SDK de Sphero

## Arquitectura del Sistema

```
Comando en grados/s → /cmd_degrees → cmd_degrees_callback → write_rc_si → drive_rc_si_units
```

## Funciones Auxiliares Disponibles

```python
# En el código del nodo
await move_forward(0.5)        # 0.5 m/s adelante
await move_backward(0.3)       # 0.3 m/s atrás
await turn_left(90.0)          # 90°/s izquierda
await turn_right(45.0)         # 45°/s derecha
await stop_robot()             # Detener
await move_and_turn(0.4, 30.0) # 0.4 m/s + 30°/s
```

## Recomendación de Uso

- **Para control manual**: Usa `/cmd_degrees` (más intuitivo)
- **Para compatibilidad ROS**: Usa `/cmd_vel` (estándar)
- **Para programación**: Usa las funciones auxiliares (más limpio)

El sistema mantiene ambos tópicos para máxima flexibilidad y compatibilidad.
