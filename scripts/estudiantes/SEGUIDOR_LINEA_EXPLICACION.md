# Seguidor de línea con un solo sensor de color (RVR)

Este documento explica de forma completa el diseño, cálculo del error, control, recuperación y parámetros del seguidor de línea implementado en:

- `scripts/estudiantes/seguidor_linea_pid_demo.py`

Incluye dos enfoques complementarios, adecuados para un solo sensor de color:
- Seguimiento de borde (edge-following) con histéresis y recuperación robusta.
- Cálculo de error normalizado apto para PID discreto (opcional/híbrido).

---

## 1) Arquitectura y tópicos ROS

- Publica: `geometry_msgs/Twist` en `/cmd_vel` (control del RVR).
- Suscribe (versión edge-following): `atriz_rvr_msgs/Color` en `/color`.
- Alternativa (versión servicio): llama servicio `/get_rgbc_sensor_values` para obtener `clear` (brillo).
- Servicio de activación del sensor: `/enable_color` (`std_srvs/SetBool`).

---

## 2) Señal de entrada y preprocesamiento

Con un solo sensor RGB se sintetiza una “intensidad” (brillo) como media simple:

```
intensidad = (R + G + B) / 3
```

Para robustez frente a ruido se usa un buffer corto (3 muestras) y se promedia. Luego se clasifica por **histéresis**:

- `BLACK` si `intensidad ≤ (umbral_negro + margen_histeresis)`
- `WHITE` si `intensidad ≥ (umbral_blanco − margen_histeresis)`
- `MID` en caso contrario (zona de transición/borde)

La histéresis evita el “rebote” cerca de los umbrales cuando cambian las condiciones de iluminación.

---

## 3) Estrategia de control con un solo sensor: Edge-Following

Con un solo sensor no es fiable estimar el “desalineamiento lateral” clásico. En su lugar, se sigue el **borde** de la línea:

- Si el sensor ve `BLACK` o `MID` (estamos sobre la línea o en el borde):
  - Avanzar a `vel_max`.
  - Aplicar un giro constante “hacia el blanco” para mantenerse pegado al borde.
  - El lado del borde que seguimos se guarda en `last_edge_dir` (derecha: +1, izquierda: −1).

- Si el sensor ve `WHITE` (se salió):
  - Ejecutar una **recuperación en dos fases** pensada para dos orugas y **sensor adelantado** respecto al centro:
    1) Retroceso breve con giro **contrario** al último borde (trae el borde bajo el sensor).
    2) Escaneo en sitio hacia el lado opuesto. Si no encuentra, alterna F1↔F2 (barrido amplio).

Este método es muy efectivo con un solo sensor porque convierte la tarea en “mantenerse pegado al borde” en lugar de “centrarse” sobre la línea.

---

## 4) Cálculo del “error” y control PID discreto (opcional/híbrido)

Si se desea un control más fino, se puede calcular un **error normalizado** apto para PID:

1. Definir `punto_medio = (umbral_negro + umbral_blanco) / 2`
2. Error normalizado (en [-1, 1]):

```
error = (intensidad − punto_medio) / max(punto_medio, 1)
error = clamp(error, −1.0, 1.0)
```

- `error < 0` → más oscuro que el punto medio (hacia el negro)
- `error > 0` → más claro que el punto medio (hacia el blanco)

### PID discreto implementado (clase `DiscretePID`)

- Proporcional: `P = Kp * error`
- Integral (método del trapecio):

```
error_avg = 0.5 * (error + error_prev)
integral += error_avg * dt
integral = clamp(integral, integral_min, integral_max)
I = Ki * integral
```

- Derivada (diferencia finita):

```
derivative = (error − error_prev) / dt
D = Kd * derivative
```

- Manejo del tiempo:
  - `dt_min` para evitar dividir por números muy pequeños.
  - `dt_max` para evitar acumulaciones si hubo pausas largas (se resetea integral/derivada si `dt > dt_max`).

- Salida: `u = P + I + D`, saturada a `±vel_ang_max`.

El script “demo” actual usa edge-following por robustez, pero puedes mezclarlo (ej.: usar `u` como giro cuando `state == MID`).

---

## 5) Recuperación y consideraciones mecánicas

- El **sensor está adelantado** respecto al centro: cuando ve blanco, probablemente el centro ya pasó el borde. Por eso:
  - Fase 1: retrocede ligeramente y gira contrario → vuelve a traer el borde bajo el sensor.
  - Fase 2: escanea al lado opuesto.
  - Alterna si no encuentra en tiempos configurables (`recovery_reverse_time`, `recovery_scan_time`).

- El robot tiene **dos orugas**: pivota bien en sitio. El escaneo en sitio es fiable para reenganchar el borde.

---

## 6) Parámetros principales (rosparam o constantes)

- Velocidades:
  - `vel_max` (m/s), `vel_min` (m/s), `vel_ang_max` (rad/s)
- Histéresis y umbrales:
  - `umbral_negro`, `umbral_blanco`, `hysteresis_margin`
- Filtrado:
  - `tamano_buffer` (3 recomendado para reacción rápida)
- Recuperación:
  - `recovery_reverse_time` (s), `recovery_scan_time` (s)
  - `recovery_linear` (m/s, típico negativo para retroceder), `recovery_angular` (rad/s)
- PID (si usas el cálculo de error):
  - `kp`, `ki`, `kd`, `integral_min`, `integral_max`, `dt_min`, `dt_max`

Sugerencias de tuning:
- Aumenta `hysteresis_margin` si hay parpadeo entre BLACK/WHITE.
- Reduce `tamano_buffer` para mayor rapidez; súbelo si hay mucho ruido.
- Si zigzaguea mucho: baja `omega` (giro en borde) o `vel_max`.
- Si tarda al recuperar: sube `recovery_angular` o las duraciones.

---

## 7) Movimiento constante y suavizado

Para evitar “pasos” o microparadas:
- Publicar `Twist` a alta frecuencia (60–120 Hz).
- Mantener una velocidad lineal constante mientras el giro corrige.
- Suavizar el `angular.z` con un filtro exponencial si el sensor tiene latencia.

---

## 8) Inicialización del sensor

Activación recomendada al iniciar el nodo:
- Llamar a `/enable_color` (`SetBool`) con `True` para encender el sensor.
- Verificar servicio de lectura: suscriptor a `/color` o `ServiceProxy` persistente a `/get_rgbc_sensor_values`.

---

## 9) Flujo simplificado del bucle

1) Leer intensidad (promedio filtrado o valor `clear`).
2) Clasificar: `BLACK`, `MID`, `WHITE`.
3) Si `BLACK`/`MID`: avanzar y girar hacia el blanco (seguir el borde).
4) Si `WHITE`: recuperación F1 (retroceso+contrario) → F2 (escaneo) → alternar hasta reenganchar.
5) Publicar `Twist` continuamente.

---

## 10) Errores comunes y soluciones

- “Se sale en curvas”: baja `vel_max` o sube ligeramente `omega`/`vel_ang_max`.
- “Oscila demasiado”: baja `omega` o añade suavizado en `angular.z`.
- “No recupera”: aumenta `recovery_angular` y tiempos de `recovery_*`.
- “Parpadeo BLACK/WHITE”: sube `hysteresis_margin` y/o `tamano_buffer`.

---

## 11) Extensiones

- Usar `DiscretePID` solo en `MID` (zona de borde) para un giro más fino y edge-following para `BLACK/WHITE`.
- Ajuste dinámico de `vel_max` según curvatura (|error| alto → reduce v).
- Calibración automática de umbrales tomando muestras de `negro` y `blanco` antes de empezar.

---

## 12) Referencias prácticas

- Script de ejemplo: `scripts/estudiantes/seguidor_linea_pid_demo.py`
- Mensajes:
  - `/cmd_vel` → `geometry_msgs/Twist`
  - `/color` → `atriz_rvr_msgs/Color`
- Servicios:
  - `/enable_color` → `std_srvs/SetBool`
  - `/get_rgbc_sensor_values` → `atriz_rvr_msgs/GetRGBCSensorValues`

---

Con este esquema, un solo sensor es suficiente para seguir la línea de forma robusta en escenarios reales: se mantiene pegado al **borde** con histéresis y, si se pierde, el robot aprovecha sus orugas y la posición adelantada del sensor para reenganchar la línea rápidamente.
