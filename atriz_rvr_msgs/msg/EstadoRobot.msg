# Estado del NODO y del enlace con el RVR. Un solo topic a 1 Hz.
#
# ⚠️ TODO EL COMPORTAMIENTO DE ESTE MENSAJE ESTÁ **NO VERIFICADO**: se escribió
#    el 2026-08-04 sin robot delante. Lo único comprobado es que compila y que
#    el fichero está en el CMakeLists. Ver `NOTAS_ESTADO_ROBOT.md` en la raíz.
#
# POR QUÉ EXISTE
#   El muro del profesor mira 16 robots a la vez, y hasta ahora la única forma
#   de saber si uno estaba vivo era mirar `/odom` — 13,05 kB/s por robot, o sea
#   1,7 Mbit/s por los 16 solo para pintar dieciséis luces verdes. Este mensaje
#   cuesta ~0,03 kB/s por robot: **0,5 kB/s los dieciséis**.
#
#   Y lo más barato que había, `/motor_status`, NO SIRVE de señal de vida: se
#   republica cada segundo con el último valor conocido, así que llega
#   exactamente igual con el RVR mudo. Un topic que sigue llegando cuando el
#   robot ha dejado de hablar es peor que no tener nada.
#
# QUÉ MIDE CADA COSA — y son DOS SUJETOS DISTINTOS, que es el punto entero
#   `latido`                      -> el NODO de la Pi
#   `rvr_responde`, `antiguedad_muestra_s`, `reanudaciones_fallidas`
#                                 -> el ROBOT al otro lado del puerto serie
#   Confundirlos es el fallo favorito de este proyecto: hay estados en los que
#   la Pi va perfectamente y el RVR no está.

std_msgs/Header header

# ── Señal de vida DEL NODO ───────────────────────────────────────────────────
# Contador monótono, +1 en cada publicación. Si no avanza entre dos lecturas,
# el nodo no está corriendo — por mucho que el topic exista.
#
# 🔴 Y el topic PUEDE existir sin nadie detrás: `ros2 topic list` conserva los
#    topics de nodos muertos (los guarda el daemon). El verificador de este
#    proyecto ya declaró «el RVR está dormido» sobre un robot APAGADO por creerse
#    esa lista. Por eso la señal es un NÚMERO QUE AVANZA y no la existencia del
#    topic ni un booleano `vivo`, que un mensaje viejo latcheado repetiría igual.
#
# ⚠️ AVANZA AUNQUE EL RVR NO CONTESTE, a propósito. Es la vida del nodo, no la
#    del robot; para el robot están los tres campos de abajo. Condicionarlo a que
#    lleguen datos perdería justo el caso que se quiere cubrir: «la Pi va bien y
#    el RVR no».
uint64 latido

# ── Parada de emergencia ─────────────────────────────────────────────────────
# La bandera del driver, tal cual. Antes no se publicaba en ninguna parte, así
# que la web solo podía decir «parada ENVIADA», nunca «parada ACTIVA» — y son
# cosas distintas: este botón ha fallado en silencio CINCO veces en este
# proyecto, cuatro de ellas devolviendo 200 OK con cero efecto en el robot.
#
# Con este campo, la web puede comprobar el EFECTO en vez del código de salida,
# que es la regla de oro de este proyecto.
#
# 📝 true implica que el driver descarta todo `cmd_vel` y que los servicios de
#    movimiento se niegan. Se baja con el servicio `/release_emergency_stop`.
bool parada_emergencia

# ── ¿Contesta el RVR? ────────────────────────────────────────────────────────
# false = hace más de `silence_timeout` (3 s por defecto) que no llega una
# muestra DEL RVR.
#
# Distingue «la Pi va bien y el RVR no contesta» de «no llego a la Pi» — que
# desde el navegador son indistinguibles, porque en los dos casos la web
# simplemente deja de recibir. Con `latido` avanzando y `rvr_responde` en false,
# el diagnóstico es inequívoco y no hay que ir al edificio a mirar.
bool rvr_responde

# Cuánto hace que llegó la última muestra REAL del RVR, en segundos.
#
# 🔴 -1.0 = no ha llegado NINGUNA desde que arrancó el nodo. En este proyecto
#    -1.0 significa SIEMPRE «no se sabe», nunca «cero» — es la misma convención
#    que las tres `antiguedad_*_s` de MotorStatus.msg, y existe porque publicar
#    un 0.0 que en realidad quiere decir «nadie ha dicho nada todavía» es falsa
#    tranquilidad, que es peor que no publicar el campo.
#
# ⚠️ Cuenta muestras DE VERDAD. El driver reinicia su reloj interno de silencio
#    al reanudar el streaming y al conectar, y esos dos instantes NO son datos
#    que hayan llegado: este campo no los cuenta.
float32 antiguedad_muestra_s

# ── CARGANDO contra DORMIDO, que es la razón de ser de este campo ────────────
# Reanudaciones consecutivas que NO devolvieron ningún dato.
#
# 🔴 SALE DE UNA MEDIDA, no de una teoría. El 2026-08-02 se apagó el RVR para
#    cargarlo con la Pi encendida —un estado COTIDIANO del laboratorio que nadie
#    había probado— y el driver escribió **8 veces «streaming reanudado» en 30 s**
#    con `/odom` a **0 mensajes en 15 s**, y 123 intentos de reconexión seguidos.
#    Lo imprimía porque `wake`+`stop`+`start` NO LANZAN EXCEPCIÓN con el robot
#    apagado, no porque volviera un dato. Un robot muerto parecía sano en el log.
#
#    Este contador es lo que separa los dos casos, y se apoya en que se comportan
#    distinto: **un RVR dormido se recupera al primer intento; uno apagado no se
#    recupera nunca.**
#
#      0     todo bien
#      1-2   pudo ser una siesta, y se está recuperando sola
#      >2    el RVR no está ahí: apagado, cargando, o el cable fuera
#
# 🔴 SE PONE A CERO CUANDO LLEGA UNA MUESTRA, no cuando la reanudación «tiene
#    éxito». El éxito es que llegue un dato, no que la llamada vuelva — es
#    exactamente el fallo medido de arriba, y darlo por bueno aquí lo reproduciría
#    dentro del campo escrito para detectarlo.
#
# ⚠️ Los umbrales 1-2 / >2 son una LECTURA RAZONADA de esa medida, no un umbral
#    calibrado: nadie ha cronometrado cuántos intentos tarda una siesta real en
#    recuperarse. Trátalos como orientación hasta que se midan.
# Cuanto hace que se completo un /odom de verdad. -1.0 = ninguno desde que
# arranco el nodo.
#
# 🔴 EXISTE PORQUE `antiguedad_muestra_s` NO LO CUBRE, y es un tercer estado.
# El driver arma /odom con CINCO componentes asincronos; el reloj de
# `antiguedad_muestra_s` avanza con CADA componente, a proposito (lo que vigila
# es que el RVR siga enviando). Asi que si llegan 4 de 5:
#
#     el latido avanza · rvr_responde dice true · /odom esta a 0 Hz
#
# ...y no lo detecta nadie: ni el vigilante de silencio ni rvr_responde. Desde
# el muro del profesor ese robot se pinta VERDE con la odometria muerta.
#
# El discriminante:
#     antiguedad_muestra ~0  ·  antiguedad_odom CRECE  -> faltan componentes
#     las dos crecen                                   -> el RVR callo
#
# ⚠️ NO se podia resolver con un `odom_completo: bool`: el conjunto de
# componentes se VACIA en cada ciclo y llegan asincronos a 16,5 Hz, asi que en
# un instante cualquiera esta medio lleno CON EL ROBOT SANO. Hay que medir
# cuanto hace que se completo uno, no si lo esta ahora.
float32 antiguedad_odom_s

uint32 reanudaciones_fallidas

# ── ¿Está encendida la LUZ del sensor de color? ──────────────────────────────
# Sin esa luz, `/color` publica [0,0,0] y `get_rgbc_sensor_values` devuelve
# oscuridad. Se enciende con el servicio `enable_color`.
#
# 🔴 EXISTE PORQUE LA LUZ SE APAGA SOLA, y eso hace que el estado NO se pueda
#    recordar. El driver la apaga por inactividad y por tope duro
#    (`color_apagado_inactividad_s`, `color_apagado_max_s`), así que un cliente
#    que guarde «yo la encendí» acabaría pintando el botón encendido sobre un
#    sensor a oscuras. Con este campo, la web LEE el estado en vez de deducirlo.
#
# ⚠️ Y no vale mirar si `/color` trae ceros: publica igual con la luz apagada
#    —no calla, al revés que el driver de ROS 1— y una superficie negra de verdad
#    también da valores muy bajos. El topic dice qué se ve; este campo, si hay luz
#    para verlo.
#
# 📝 true también cuando se arrancó con `color_detection:=true`. En ese caso NO
#    se apaga sola: la puso alguien a propósito.
bool color_activo
