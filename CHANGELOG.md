# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).


## [2026-08-14] — README al día, dos números medidos, y el driver deja de mentir con el RVR apagado

### Fixed
- **`rvr_driver_node.py`: «streaming reanudado» era mentira con el RVR apagado y ya no existe**
  (evidencias 52 y 116 de `Atriz_migracion_ros2`). `wake+stop+start` no lanzan excepción con el
  robot muerto, así que el mensaje salía cada ~6 s para siempre (~46.000 líneas/día por robot
  sobre la microSD). Ahora el éxito lo declara el único testigo válido — el contador de
  muestras—: **«el RVR VOLVIÓ: primera muestra tras N intento(s)»**. Verificado con un apagado
  real: 0 mentiras en la ventana.
- **Espera creciente entre reintentos de reanudación**: `timeout × 2^fallidas` con tope de
  60 s (3→6→12→24→48→60, progresión medida en el journal). Desde el 2º intento el diagnóstico
  dice la verdad: «apagado, cargando o el cable fuera. Próximo intento en N s». ⚠️ Coste: tras
  encender el RVR, el reenganche puede tardar hasta ~1 min (necesita un intento posterior).
- **`supervisor_navegacion.py`: el detalle de `nav_latcheado` incorpora lo medido** (evidencia
  112): «espera ~5 minutos (el bloqueo caduca solo) y quita la causa antes de reintentar, o
  reset-failed desde el robot para no esperar» — antes solo mandaba al reset-failed.

### Changed
- **`README.md`: el bloque de referencia ROS 2 se puso al día** (estaba fechado 2026-07-31):
  los seis topics que faltaban en la tabla (`/estado_robot` con `conduciendo_por_ir`,
  `/encoders`, `/motor_status`, `/estado_ir`, `/infrared_messages`, `/ambient_light` con su
  «no lo uses»), la sección del material docente con su estado físico (sesión en banda el
  2026-08-13, solo queda el seguidor de línea), y el repo de migración ya no figura como
  privado (es público desde el 2026-08-11).
- **El caudal de `/estado_robot` que decía el README era inventado y se corrigió dos veces
  el mismo día**: el «~0,03 kB/s» era el de `/battery_state` copiado (aviso del PC) — primero
  se retiró a «SIN MEDIR» y después entró el número medido: **0,35 kB/s por rosbridge, 348
  B/msg** (evidencia 110 de `Atriz_migracion_ros2`).

### Notas de operación (viven en `Atriz_migracion_ros2`, se citan aquí por visibilidad)
- El robot ahora **se cura solo** de dos fallos: nacer mudo en DDS (`atriz-vigia-dds`,
  evidencia 113) y el USB del LIDAR desenchufado (`atriz-lidar-reenganche` por udev,
  evidencia 115). Al parar la navegación, el barrido **vuelve al estado que la unidad
  encontró** (evidencia 114).


## [2026-08-09] — `Aproximacion.radius` del collision_monitor: 0.18 → 0.15

### Changed
- **`collision_monitor.yaml`: `Aproximacion.radius` baja de `0.18` a `0.15`.** El círculo
  **inmoviliza por completo** al robot —no gira, no avanza y ni siquiera puede alejarse— cuando hay
  algo dentro. La banda en la que eso ocurre sin que el robot toque nada pasa de **3,6 a 0,6 cm**.
  - `banda de inmovilización` = `margen ante el error del LIDAR` = `radius − 0.1442`. **Son el mismo
    número**, así que no se puede encoger uno sin el otro. Por eso NO se baja a 0.145: dejaría
    0,1 cm contra un ruido de LIDAR medido de ±0,3.
  - Hueco al parar **medido** con los dos valores: `0.18` → 9,3-9,4 cm a 0,25 m/s y 10,9 a 0,40;
    `0.15` → 6,3 a 0,25 y **7,4 / 6,6 a 0,40** (velocidad máxima).
  - Control decisivo a la misma distancia: pared a 15,8 cm de `base_footprint`, con `0.18`
    congelado y con `0.15` girando 34,9°.
  - **No arregla**: los 0,6 cm de banda que quedan, el centímetro ciego de `range_min`, ni que
    `approach` no distinga acercarse de alejarse.
- **`nav2_atriz.yaml`**: comentarios que citaban el 0.18.
- **`scripts/estudiantes/atriz.py`**: el aviso al alumno pasa de «algo a menos de 18 cm» a 15, y de
  «mira si tiene algo a menos de 20 cm» a 17.

### Fixed
- **`collision_monitor.yaml` afirmaba que el punto ciego del LIDAR cae dentro del chasis y que «no
  hay zona muerta».** Estaba calculado con la media longitud **cruzada** del URDF (0.109). Con la
  real (0.095) el punto ciego de `range_min` **sobresale ~1 cm por delante y por detrás**, y ningún
  polígono puede cubrirlo.
- **Cotas del robot cerradas con un tercer instrumento.** Con el robot tocando la pared, el LIDAR da
  **9.00 cm** al borde trasero y **10.03** al delantero: exactamente `base_length 0.190` con
  `laser_x −0.005`. El URDF acierta; la cinta que daba 0.182 medía al chasis.

### Notes
- 🔴 **`Aproximacion.radius` NO se puede cambiar en caliente**: `ros2 param set` lo guarda y `get` lo
  devuelve, pero el nodo no reconstruye el polígono — con 0.18, 0.15 y **0.30** el perfil sale
  idéntico. Hay que editar el YAML y reiniciar `atriz-robot`.
- `verificar_robot.sh` da **FALLO** si encuentra `0.18` en un robot: significa que no le llegó este
  fichero. Y la banda de la fase F6 de la aceptación se reajustó a `[14.0, 19.0]`, cuyo techo
  detecta exactamente ese caso.
- Evidencias 93, 94 y 95 en `Atriz_migracion_ros2`.

## [rama `ros2`] - 2026-07-31

> 🔴 Las entradas de más abajo son del sistema **ROS 1 / Noetic** y describen otro software.
> Se conservan como historia, no como estado.

### Añadido
- **`cancelar_nav2`** — nodo nuevo, lo arranca `nav2.launch.py`. Cancela los objetivos de Nav2
  cuando se pulsa la parada de emergencia. Sin él, **liberar** la parada devolvía el robot a
  navegar: medido con control, **34.7 cm** sin el arreglo contra **0.0 cm** con él.
- **17 servicios más** en el driver (de 1 a 18): LEDs, RGBC, encoders, info de sistema, IR,
  parámetros de conducción y cinco de movimiento.
- **`collision_monitor`** en `robot.launch.py`, con su propio gestor de ciclo de vida — vive
  aquí y no en Nav2 porque los estudiantes teleoperan **sin** navegación.
- **`localizacion.launch.py`** (map_server + AMCL), **excluyente** con `slam.launch.py`: los dos
  publican `map → odom` y juntos parten el árbol TF sin dar error.
- `/battery_state`, keepalive de 30 s y detector de silencio: el RVR se dormía a los 300.6 s y
  el nodo seguía «sano», publicando cero.
- Parámetros `color_detection` y `publicar_inclinacion` (los dos **false** por defecto).

### Arreglado
- **`behavior_server` publicaba 5× directamente en `/cmd_vel`**, saltándose la capa de
  seguridad — y justo en las conductas de recuperación, que se ejecutan con el robot pegado a
  algo. Remapeado a `cmd_vel_raw`.
- **La parada de emergencia**, por namespace y por QoS. `TRANSIENT_LOCAL` en un suscriptor no
  añade garantías: **solo restringe** con quién empareja, y ni `ros2 topic pub` ni rosbridge lo
  son. Ahora **VOLATILE**.
- **`/color` publicaba `[0,0,0]` desde siempre**: el sensor no da nada sin su luz (canal claro
  **4 apagada contra 741 encendida**) y el driver nunca la encendía.
- **El URDF copiaba la ficha del RVR, y la ficha miente**: decía `0.218 × 0.185 × 0.114` y el
  robot mide **18.2 × 21.7 × 7.0 cm**. Largo y ancho estaban **cruzados**.
- `robot_radius` 0.11 → **0.145** (el circunscrito real) en los dos costmaps.

### Cambiado
- `desired_linear_vel` a **0.40 m/s**, que es la meseta real medida.
- El progress checker de Nav2, relajado a 0.25 m en 15 s: con la capa de seguridad frenando al
  40 %, **ir despacio ya no es prueba de estar atascado**.

## [Unreleased]

### Added
- Professional project organization
- Complete documentation structure
- Comprehensive testing suite
- Dual topic control system (/cmd_vel and /cmd_degrees)
- Safety mechanisms and emergency stop
- LED control system
- Sensor integration (IMU, odometry, color, light)
- Infrared communication capabilities

### Changed
- Fixed package names in package.xml files
- Updated CMakeLists.txt to use modern CMake
- Consolidated script organization
- Improved documentation structure

### Fixed
- Removed incorrect dependencies
- Fixed package metadata
- Eliminated duplicate scripts
- Resolved linter errors
- Fixed catkin build system compatibility

## [0.0.2] - 2024-01-XX

### Added
- Initial release of Atriz RVR ROS Driver
- Basic robot control functionality
- ROS message and service definitions
- Serial communication library

### Changed
- Migrated from original sphero_rvr_hw package
- Updated for Universidad de Nariño project

## [0.0.1] - 2024-01-XX

### Added
- Initial project setup
- Basic ROS package structure
- Sphero SDK integration
