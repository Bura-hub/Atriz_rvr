# Salud de los motores del RVR.
#
# POR QUÉ EXISTE
#   Hasta el 2026-08-01, un robot con una oruga trabada se veía EXACTAMENTE igual
#   que uno que navegaba mal: /odom avanzaba poco, Nav2 abortaba por «no
#   progresar», y nada decía si el problema era el algoritmo o el hardware. Con
#   16 robots en otro edificio, esa diferencia decide si alguien tiene que ir.
#
# CÓMO LLEGA EL DATO
#   El RVR notifica por EVENTOS, no por sondeo. El driver mantiene el último
#   estado conocido y lo republica, así que un suscriptor que llegue tarde recibe
#   el estado actual (el publicador es TRANSIENT_LOCAL).

std_msgs/Header header

# ── Atasco ───────────────────────────────────────────────────────────────────
# ⚠️ `atascado` NO significa «el robot está parado». Significa que el firmware
#    detectó que un motor recibe corriente y NO gira lo que debería. Un robot
#    parado a propósito no lo dispara.
#
# Se publican por separado porque saber CUÁL está trabado es la mitad del
# diagnóstico: uno solo suele ser un obstáculo en esa oruga; los dos a la vez,
# el robot encallado o sobre un desnivel.
# Índices del SDK: 0 = izquierdo, 1 = derecho (`MotorIndexesEnum`).
bool atascado_izquierdo
bool atascado_derecho

# ── Fallo eléctrico ──────────────────────────────────────────────────────────
# Más grave que un atasco: no se resuelve levantando el robot.
bool fallo

# ── Protección térmica ───────────────────────────────────────────────────────
# El firmware limita la potencia cuando los motores se calientan, y un robot así
# navegará mal sin que nada más lo explique.
#
# 📝 La temperatura la da el propio RVR en esta notificación. Es la ÚNICA vía por
#    la que este proyecto tiene temperatura de motores; `get_temperature` del SDK
#    no se usa.
float32 temperatura_izquierdo    # °C
float32 temperatura_derecho      # °C

# Estado que reporta el firmware. 0 = normal; el resto de valores los define el
# RVR y este proyecto NO los ha caracterizado: se publican en crudo a propósito,
# en vez de inventarles un significado.
uint8 estado_termico_izquierdo
uint8 estado_termico_derecho

# ── Frescura del dato: UNA POR FUENTE, y no es un detalle ────────────────────
# Segundos desde que se supo cada cosa.
#
# 🔴 -1.0 = NUNCA se ha sabido nada de eso, así que el campo de arriba es el
#    VALOR INICIAL y no una medida. Sin esto, «todo a false» sería
#    indistinguible de «nadie ha dicho nada todavía».
#
# 🔴 Y VAN SEPARADAS PORQUE LAS TRES FUENTES SON DISTINTAS. Una sola antigüedad
#    para las tres dio, en la primera versión de este mensaje, un
#    `antiguedad_atasco_s = 0.0` —o sea «recién comprobado»— cuando el atasco
#    **no se comprueba nunca**: lo refrescaba el sondeo del fallo. Falsa
#    tranquilidad, que es peor que no publicar el campo.
#
#   fallo y térmica  -> se SONDEAN cada 30 s, así que su antigüedad es real
#   atasco           -> solo existe por notificación, y en este firmware
#                       NO LLEGA NINGUNA: se queda en -1.0 para siempre
float32 antiguedad_atasco_s
float32 antiguedad_fallo_s
float32 antiguedad_termico_s
