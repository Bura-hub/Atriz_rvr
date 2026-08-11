# Estado del sistema de infrarrojos robot-a-robot. Se publica PERIÓDICAMENTE.
#
# POR QUÉ EXISTE, ADEMÁS DE /infrared_messages
#
#   `/infrared_messages` es un EVENTO: se publica cuando llega un código. Un
#   evento no dice qué ven mis sensores AHORA, ni si estoy conduciendo por IR, ni
#   distingue «nadie ha emitido» de «esto está roto».
#
#   Es el mismo problema que resolvió `EstadoRobot`, y por eso este mensaje se
#   parece: un topic mudo no distingue el silencio del fallo. Aquí además hay un
#   RELOJ — la lectura del firmware se borra al segundo — así que las antigüedades
#   son parte del dato, no metadatos.
#
#   Diseño: docs/superpowers/specs/2026-08-11-sistema-ir-robot-a-robot-design.md

std_msgs/Header header

# ── Lo que ven mis cuatro sensores receptores ────────────────────────────────
#
# 🔴 `crudo` ES EL DATO DE VERDAD. Los cuatro `sensor_N` son mi interpretación.
#
#    El firmware devuelve UN uint32 (`get_bot_to_bot_infrared_readings`). La
#    documentación del SDK dice que son cuatro bytes, uno por sensor, y da una
#    máscara — pero la etiqueta «Mask description ON BOLT». El robot de este
#    laboratorio es un Sphero RVR, NO un BOLT: otro chasis y otra disposición.
#
#    Así que publicar solo mi troceado sería publicar una suposición heredada de
#    otro producto. Va el valor íntegro para que, si me equivoco al partirlo, la
#    evidencia siga entera y se pueda reinterpretar sin repetir la medición.
#    (Este proyecto ya se quemó fiándose del SDK: documenta los encoders como
#    `Left`/`Right` y el payload real trae `LeftTicks`/`RightTicks`.)
uint32 crudo

# Los cuatro bytes de `crudo`, del MENOS significativo al más. Ese orden es una
# decisión mía, no un hecho del hardware.
#   255 = ese sensor no ve nada  ·  0-15 = ha visto ese código
#
# ⏳ QUÉ LADO DEL ROBOT ES CADA UNO: **SIN MEDIR**. Se llaman `sensor_N` y no
#    `frontal_izquierdo` justamente por eso. Los bautiza la prueba de viabilidad
#    con dos robots (ver el diseño, §Pruebas); hasta entonces, nombrarlos sería
#    inventarse la orientación del robot.
uint8 sensor_0
uint8 sensor_1
uint8 sensor_2
uint8 sensor_3

# false si el sondeo está apagado (`ir_sondeo_hz:=0.0`) o si la consulta falló.
# 🔴 Cuando es false, los cinco campos de arriba NO son lecturas: son relleno.
#    Nunca se publican ceros como si fueran datos.
bool lecturas_validas

# Cuánto hace de la última consulta al firmware. Importa porque el dato del
# firmware CADUCA EN 1 SEGUNDO: con más de 1 s de antigüedad, un 255 significa
# «hace demasiado que no miro», no «no hay nadie».
float32 antiguedad_lectura_s

# ── El último mensaje recibido ───────────────────────────────────────────────
uint8 ultimo_codigo
# false si no ha llegado NINGUNO desde que arrancó el driver.
# 🔴 Sin esto, `ultimo_codigo = 0` no se distingue de «llegó el código 0», que es
#    un código válido.
bool hay_mensaje
float32 antiguedad_mensaje_s

# ── Qué le he pedido yo al firmware ──────────────────────────────────────────
# broadcasting · following · evading · off
string modo
uint8 far_code
uint8 near_code

# ── 🔴 Y el campo por el que este mensaje merece existir ─────────────────────
#
# `following` y `evading` son modos del FIRMWARE: el RVR conduce solo, sin pasar
# por `cmd_vel`. Así que el watchdog no los ve, el `collision_monitor` no los ve,
# y hasta hoy NADA EN ROS SABÍA QUE EL ROBOT SE ESTABA MOVIENDO.
#
# `get_active_control_system_id()` devuelve 8 (`infrared_follow_and_evade`)
# mientras eso pasa. Publicarlo es la única forma que tiene el resto del sistema
# —y la web, y el profesor— de enterarse.
bool conduciendo_por_ir
