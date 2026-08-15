# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).


## [2026-08-15d] — `atriz.py` apagaba el barrido de OTRO 3 de cada 5 veces, en silencio

### Fixed
- 🔴🔴 **`_encender_barrido()` daba 1,0 s al primer `/scan` para decidir si el barrido ya estaba
  encendido, y el primer mensaje tarda más de eso la mitad de las veces.** Cuando el plazo
  vencía, se creía dueña del barrido y al cerrar llamaba a `/stop_scan` sobre un barrido ajeno
  — **dejando ciega a una navegación en curso, y sin imprimir su propio aviso**.

  Encontrado por la casilla 4-9 de `VALIDAR_CON_EL_ROBOT.md`, medido desde el terminal web
  (evidencia 119 en `atriz_migracion`). Con el barrido encendido de antemano y `/scan` a 12 Hz:

  ```
  corrida 1  aviso=0 → APAGADO      corrida 4  aviso=1 → ENCENDIDO
  corrida 2  aviso=0 → APAGADO      corrida 5  aviso=1 → ENCENDIDO
  corrida 3  aviso=0 → APAGADO
  ```

  La correlación es exacta: falla la **detección**, no la decisión — `debe_apagar_balido()` hace
  bien su trabajo con la bandera que le dan.

  **La causa, medida:** el retardo no es el ritmo del topic, es el **descubrimiento de DDS**.
  Primer `/scan` en una suscripción recién creada: `40 · 1282 · 16 · 1677 · 28 · 964 ms` (n=6).
  Partido en sus dos mitades: `emparejar 11-1598 ms` y, una vez emparejado, `mensaje en
  22-333 ms`. Casi todo es descubrimiento.

  ✅ **El arreglo separa las dos esperas**, porque son dos fenómenos distintos y solo uno es
  lento: primero `ESPERA_EMPAREJAR_S = 5.0` (3× el peor emparejamiento medido) a que
  `get_publisher_count() > 0`, y **solo entonces** `ESPERA_PRIMER_SCAN_S = 1.0` (3× el peor
  hueco medido) al dato.
  🔴 **Y el segundo plazo NO se puede subir a lo bruto**: con el barrido apagado —el caso
  normal— se paga entero en cada arranque de cada programa de cada alumno.
  📌 Control medido: con el barrido **apagado**, la suscripción empareja en **10 ms** y no llega
  ningún mensaje en 8 s. Por eso el emparejamiento sirve de señal y el mensaje de discriminador.

  📝 La familia: **un plazo puesto contra un fenómeno cuya latencia no se había medido**, igual
  que el `default_server_timeout: 20` de Nav2 y el `MAX_SIN_CAMBIO = 5` de `girar()`.

  ⚠️ **Sin prueba unitaria**, y se dice: la lógica nueva es espera de E/S y necesita ROS. La
  verificación es empírica —5 corridas antes y 5 después— y **la escribió el PC, que no puede
  correr las 65 pruebas de este fichero**. 👤 Revisadlo desde la Pi.

---

## [2026-08-15c] — `soy_el_dueno` se difundía calculado para UNO, y el segundo alumno veía el programa del primero como suyo

### Fixed
- 🔴🔴 **`agente_sesion.py` difundía el estado con `soy_el_dueno` calculado para el dueño.**
  `difundir(estado_actual(actual['sujeto']))` manda **el mismo mensaje a todos los clientes**,
  así que **todos** lo recibían en `True`.

  **Medido desde el navegador contra rvr-01** (2026-08-15, dos alumnos y un solo robot): con
  `bura_hub` ejecutando `05_sensor_color.py`, la pantalla de `ana` decía **«Ya tienes un programa
  corriendo. Párralo antes.»** sobre el programa ajeno, con su PID (61700) delante.

  ⚠️ **No era un agujero de seguridad, y conviene decirlo con precisión:** `ana` pulsó «Parar el
  programa» y **el programa siguió vivo** — la comprobación de dueño de `atriz_signal` nunca
  dependió de este campo. Lo que fallaba era **lo que la pantalla podía afirmar**, que es
  exactamente lo que la casilla 4-10 existe para cazar: *«el nombre es la diferencia entre
  esperar y cruzar el aula a preguntar»*.

  → **Arreglo en dos piezas.** `difundir_estado()` manda a cada cliente **el suyo**, leyendo su
  nombre con `getattr(c, 'sujeto', '')` —un `AttributeError` ahí lo tragaría el `except` de al
  lado y **descartaría al cliente en silencio**—; y la decisión se extrae a `es_el_dueno()` en
  `agente_nucleo.py`, que **sí se puede probar sin robot** (aquí no hay tornado). El sujeto vacío
  nunca es dueño: sin ese guardia, un cliente sin nombre casaría con un dueño vacío, que es la
  rama por descarte sin condición de señal que este proyecto persigue.

  → **3 pruebas nuevas** (suite 33 → **36** en el PC, +17 saltadas del PTY). Mutada a
  `return True` —el fallo original— caen dos.

  📌 **La regla que deja: cuando un campo depende de QUIÉN pregunta, no se puede difundir.** Es
  la misma forma que rosbridge compartiendo una única suscripción por topic, donde el QoS del
  primero se lo impone a todos.

⚠️ **rvr-01 sigue con el código viejo** hasta que se haga `git pull` y `systemctl restart
atriz-agente` en el robot.

---

## [2026-08-15b] — Una prueba del núcleo del agente que solo pasaba en Linux (desde el PC)

### Fixed
- **`test_entorno_conserva_el_pythonpath_de_ros_DETRAS_de_la_sesion` escribía el separador de
  rutas a mano** (`'/opt/ros/…:/otro'`) mientras el código, correctamente, usa `os.pathsep`.
  Resultado: **pasaba en la Pi y fallaba en el PC**, donde `os.pathsep` es `;`, la cadena no se
  partía y el `in partes` daba falso **sobre un código correcto** — un falso positivo, de la
  familia que este proyecto ya lleva once veces.
  🔴 Y lo que lo hace algo más que un papercut: `agente_nucleo.py` está separado de `agente_pty.py`
  **justamente para poder probarse donde no hay robot**. Una prueba suya que solo corre en Linux
  devuelve el fichero a depender de la Pi, que es lo que la separación existe para evitar.
  → **Verificado en el PC (Windows): 33 pasan, 17 se saltan** (las 17 son las del PTY, que no
  existe aquí y tienen que saltarse). Antes: 32 pasaban y 1 fallaba.

---

## [2026-08-15] — El agente de sesión, auditado en la Pi y validado en vivo: cinco arreglos con su experimento

### Fixed
- **`agente_pty.lanzar()`: la carrera del pgid, confirmada por efecto** (evidencia 117 de
  `Atriz_migracion_ros2`): `os.getpgid(pid)` tras `pty.fork()` compite con el `setsid()` del
  hijo; si gana el padre, el peldaño SIGKILL **suicida al agente con su propio grupo**. Sin
  parche, 2 de 4 tandas de la propia suite murieron por SIGKILL; con `pgid = pid`, 6/6. Cinturón
  además en `senalar()`/`vive()`: `pgid <= 1` se niega (`killpg(0)` es el grupo del llamante).
- **`agente_sesion.terminar()`: faltaba `remove_handler`** antes de cerrar el fd maestro — el
  kernel reutiliza el número y la SEGUNDA ejecución chocaba con el registro rancio de tornado.
  Verificado en vivo: segunda y tercera ejecución corren y terminan.
- **El manejador de señales que la unidad prometía y no existía**: `apagar_ordenado()` recorre
  los cuatro peldaños del hijo y para el bucle al terminar; rechazo `AGENTE_PARANDO` a
  ejecuciones nuevas durante el cierre; la unidad pasa a `KillMode=mixed` (la señal solo al
  principal, que ahora sí orquesta) y pierde el grupo `video` (no hay cámara).
- **`entorno_de_ejecucion()` pisaba `PYTHONPATH` — CAZADO EN VIVO**: la práctica 05 moría en
  `import rclpy` a los 0 s (es PYTHONPATH, no `AMENT_PREFIX_PATH`, quien hace visible
  `/opt/ros/…/site-packages`). La sesión va primero (la copia de `atriz.py` gana) y el heredado
  detrás. Con el arreglo, la 05 corre de punta a punta por el agente con el sensor real.
- **`cosechar()` con memoria — cazado en vivo**: `latir()` (1 Hz) le ganaba el `waitpid` a
  `terminar()` y el `atriz_fin` salía con `codigo=None` sobre un programa que terminó bien.
- **`copiar_biblioteca()` lleva también los `.json`**: el seguidor lee `seguidor_config.json`
  con `if exists else {}` — en la sesión no moría, corría **callado con los umbrales de fábrica**
  en vez de los calibrados del aula.
- Arnés: `leer_hasta(hasta='X ')` cortaba en la subcadena (carrera, 1 de 4 tandas) → `'X None'`.
- `atriz_leer` resuelve contra el listado real antes de abrir (el contrato del docstring de
  `nombre_seguro`, ahora cumplido), y `terminar()` recoge la carpeta tmpfs de la sesión.

La suite del agente pasa de 44 a **50 pruebas**, ×3 tandas limpias en la Pi. Validación en vivo
completa (19 casillas, práctica 05 con el RVR) en la evidencia 117.

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
