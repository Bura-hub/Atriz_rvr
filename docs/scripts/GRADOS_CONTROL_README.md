# Control del Sphero RVR con Grados

## Descripción General

El sistema ha sido actualizado para trabajar directamente con grados/segundo, lo que es más intuitivo y consistente con la SDK nativa de Sphero. El sistema mantiene compatibilidad con ROS (que usa radianes) mediante conversión automática.

## Funciones Principales

### 1. Funciones de Movimiento Básico

```python
# Mover hacia adelante
await move_forward(0.5)  # 0.5 m/s

# Mover hacia atrás
await move_backward(0.3)  # 0.3 m/s

# Girar a la izquierda
await turn_left(90.0)  # 90 grados/s

# Girar a la derecha
await turn_right(45.0)  # 45 grados/s

# Detener el robot
await stop_robot()

# Movimiento combinado
await move_and_turn(0.4, 30.0)  # 0.4 m/s + 30°/s izquierda
```

### 2. Función de Control RC

```python
# Control directo con grados/segundo
await write_rc_si(linear_vel_m_s, angular_vel_deg_s)
```

## Comandos ROS con Grados

### Conversión Automática

El sistema convierte automáticamente de radianes (ROS) a grados (Sphero):

```bash
# Girar 90 grados por segundo
rostopic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.5708}}" -r 10
# 1.5708 rad/s = 90°/s

# Girar 45 grados por segundo
rostopic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.7854}}" -r 10
# 0.7854 rad/s = 45°/s

# Girar 10 grados por segundo
rostopic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1745}}" -r 10
# 0.1745 rad/s = 10°/s
```

## Tabla de Conversión Rápida

| Grados/s | Radianes/s | Comando ROS |
|----------|------------|-------------|
| 1°/s     | 0.01745    | z: 0.01745  |
| 10°/s    | 0.1745     | z: 0.1745   |
| 30°/s    | 0.5236     | z: 0.5236   |
| 45°/s    | 0.7854     | z: 0.7854   |
| 90°/s    | 1.5708     | z: 1.5708   |
| 180°/s   | 3.1416     | z: 3.1416   |
| 360°/s   | 6.2832     | z: 6.2832   |

## Ejemplos Prácticos

### 1. Girar 360 grados en 10 segundos
```bash
# 36 grados/s = 0.6283 rad/s
rostopic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.6283}}" -r 10
```

### 2. Girar 90 grados en 1 segundo
```bash
# 90 grados/s = 1.5708 rad/s
rostopic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.5708}}" -r 10
```

### 3. Movimiento combinado
```bash
# 0.5 m/s adelante + 30°/s giro izquierda
rostopic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5236}}" -r 10
```

## Script de Ejemplo

Usa el script `example_degrees_control.py` para probar los comandos:

```bash
python3 example_degrees_control.py
```

## Ventajas del Sistema con Grados

1. **Más intuitivo**: Los grados son más fáciles de entender que los radianes
2. **Consistente con Sphero**: La SDK nativa usa grados/segundo
3. **Precisión nativa**: Sin necesidad de calibración manual
4. **Compatibilidad ROS**: Mantiene la interfaz estándar de ROS
5. **Funciones auxiliares**: Proporciona funciones de alto nivel para movimientos comunes

## Arquitectura del Sistema

```
ROS Twist (rad/s) → Conversión automática → Sphero SDK (grados/s)
     ↓                    ↓                        ↓
geometry_msgs/Twist → cmd_vel_callback → write_rc_si → drive_rc_si_units
```

El sistema mantiene la compatibilidad con ROS mientras aprovecha la precisión nativa de la SDK de Sphero.
