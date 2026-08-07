# Estado de SLAM y de Nav2, para que la web pinte los botones sin mentir.
# Lo publica `supervisor_navegacion` a 1 Hz. NO lo publica el driver.
#
# ⚠️ TODO ESTE MENSAJE ES **NO VERIFICADO** salvo lo que dice cada campo: se
#    escribió el 2026-08-07 con los números de las evidencias 78 y 79 delante,
#    pero el nodo que lo publica se estrena con él.
#
# POR QUÉ NO VA EN `EstadoRobot`
#   Porque el productor es OTRO NODO. Meter `slam_activo` en `EstadoRobot`
#   obligaría al supervisor a hablar con el driver para que republicara, y
#   acoplaría justo lo que la Decisión 2 separa: el driver tiene que funcionar
#   sin Nav2. Y hay una razón peor: si el driver se reinicia, `slam_toolbox`
#   SOBREVIVE —vivo y mudo, que es el fallo medido—, así que una señal alojada
#   en el driver se pondría a cero justo en el caso que existe para detectar.
#
# POR QUÉ NO BASTA UN BOOLEANO, y esto está medido
#   `systemctl is-active` dice `active` sobre un `slam_toolbox` que sobrevivió a
#   un reinicio del driver con el búfer TF roto: deja de procesar, y el mapa
#   sale idéntico celda a celda tras mover el robot 80 cm. **`active` no es
#   «funcionando».** Por eso hay seis estados y no dos.

std_msgs/Header header

# ── Señal de vida DEL SUPERVISOR ─────────────────────────────────────────────
# Contador monótono, +1 en cada publicación. Si no avanza, **todo lo demás de
# este mensaje es viejo** y la web debe pintar «no se sabe», no el último valor.
# Misma convención que `latido` de EstadoRobot, y por la misma razón: un topic
# que existe no prueba que haya alguien detrás.
uint64 latido

# ── Los seis estados ─────────────────────────────────────────────────────────
# Los dos del medio son los que `is-active` esconde, y son los que este proyecto
# ya ha pagado:
#
#   CIEGO  la unidad está `active`, los procesos vivos, el ciclo de vida en
#          `active`… y NO llega `/scan`. Sin barrido, el `collision_monitor`
#          bloquea el movimiento (medido: 0,0 cm contra 9,9 con el mismo
#          comando) y **el robot parece averiado**.
#   MUDO   llega `/scan` y los procesos están, pero SLAM no procesa. Es el
#          `slam_toolbox` vivo y mudo de la Fase 4.
uint8 APAGADO=0
uint8 ARRANCANDO=1
uint8 FUNCIONANDO=2
uint8 CIEGO=3
uint8 MUDO=4
uint8 FALLO=5
uint8 DESCONOCIDO=6

uint8 slam
uint8 nav

# Por qué está en ese estado, en texto para la persona. Vacío salvo rechazo o
# fallo. La web lo muestra TAL CUAL: «no hay mapa», «hace falta reset-failed
# desde el robot», «slam_toolbox está corriendo».
string slam_detalle
string nav_detalle

# ── Cuánto lleva arrancando ──────────────────────────────────────────────────
# Segundos desde que se aceptó la petición. **-1.0 = no está arrancando**, que
# en este proyecto significa siempre «no se sabe / no aplica», nunca «cero».
#
# 📊 MEDIDO el 2026-08-07 (evidencia 79, n=2): Nav2 tarda **24,3 s** hasta
#    aceptar objetivos, con 0,44 s de dispersión. `TimeoutStartSec=120`.
#    → La web pinta SEGUNDOS TRANSCURRIDOS, no un porcentaje: 24,3 s es una
#      medida en reposo, y nadie la ha probado con 16 robots y la batería baja.
float32 slam_arrancando_s
float32 nav_arrancando_s

# ── Lo que decide si el botón se puede pulsar ────────────────────────────────
# Sin mapa legible, Nav2 no puede arrancar: `atriz-nav.sh` sale con error. La
# web debe DESHABILITAR el botón, no dejar que el alumno lo intente.
bool hay_mapa

# 🔴 EL CAMPO QUE EVITA UNA LLAMADA DE TELÉFONO.
#    `Restart=on-failure` + `StartLimitBurst=3`: **un solo `systemctl start` sin
#    mapa agota el presupuesto** —el arranque inicial más dos reintentos
#    automáticos son ya los tres, en ~40 s— y la unidad queda `failed`. De ahí
#    solo se sale con `systemctl reset-failed`, o sea con PRIVILEGIO, que nadie
#    tiene desde el navegador (medido dos veces, evidencia 79).
#
#    Sin este campo, «no arrancó» y «bloqueado hasta que alguien entre por SSH»
#    son indistinguibles desde la web: los dos son un botón que no hace nada.
bool slam_latcheado
bool nav_latcheado
