# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

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
