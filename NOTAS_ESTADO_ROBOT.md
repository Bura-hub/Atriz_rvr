# `/estado_robot` — las tres señales que le faltaban al driver

**Rama:** `feat/estado-robot`, partiendo de `ros2`.
**Escrito el:** 2026-08-04.
**Estado:** 🔴 **TODO EL COMPORTAMIENTO ESTÁ NO VERIFICADO.** Se escribió **sin robot
delante**. Lo único comprobado es lo que se puede comprobar sin él, y está listado abajo.

> **No fusiones esta rama sin pasar antes la sección «Cómo verificarlo».** Este proyecto tiene
> media docena de fallos documentados en los que el nodo arranca, los topics existen y no
> circula un solo dato. Un `colcon build` verde no dice nada de eso.

---

## 1 · Qué añade

Un topic nuevo, **`/estado_robot`**, a **1 Hz**, con cinco campos. Nada más: no cambia ningún
ritmo, ningún QoS existente, ni la lógica de reconexión más allá de contarla.

| Fichero | Qué se le hizo |
|---|---|
| `atriz_rvr_msgs/msg/EstadoRobot.msg` | **nuevo** · `header` + **6 campos de datos** (el sexto, `antiguedad_odom_s`, añadido el 2026-08-04 tras la revisión desde el robot: ver §3 bis). Un validador anterior contó «6» incluyendo el `header`: hoy serían 7 |
| `atriz_rvr_msgs/CMakeLists.txt` | una línea en `rosidl_generate_interfaces` (y el rótulo `(6)`→`(8)`, que ya estaba mal antes: había siete ficheros) |
| `atriz_rvr_driver/scripts/atriz_rvr_driver/rvr_driver_node.py` | import, estado, publicador, temporizador de 1 Hz, `_publicar_estado()`, el contador dentro de `_vigilar_silencio`, y el espejo de muestras en los cuatro handlers |
| `atriz_rvr_bringup/launch/robot.launch.py` | `/estado_robot` en `topics_sub_glob` (la lista `LEER`) |

**El lado cliente (web / `atriz-lab`) NO se ha tocado**, a propósito: había otro agente
trabajando ahí.

### Coste

~0,03 kB/s por robot → **~0,5 kB/s los dieciséis**. La alternativa que había —vigilar la flota
mirando `/odom`— son 13,05 kB/s por robot, **1,7 Mbit/s por 16**, solo para pintar dieciséis
luces verdes.

⚠️ Ese 0,03 kB/s es una **estimación de sobremesa**, no una medida: sale del tamaño del mensaje
por 1 Hz. Lo que hay que medir es el tráfico real por rosbridge, que es JSON y abulta más
(`mediciones_banco/probar_rosbridge.py`).

---

## 2 · Por qué cada campo, con la medida que lo justifica

### `uint64 latido` — la señal de vida DEL NODO

Contador monótono, +1 en cada publicación.

**Por qué un número y no un booleano `vivo`:** porque un booleano latcheado por
`TRANSIENT_LOCAL` diría `true` para siempre. Lo único que no se puede falsificar por accidente
es **un número que tiene que avanzar entre dos lecturas**.

**Por qué no basta con que el topic exista:** `ros2 topic list` **conserva topics de nodos
muertos** — los guarda el daemon. El verificador de este proyecto ya declaró «el RVR está
dormido» sobre un robot **apagado** por creerse esa lista (CLAUDE.md).

**Por qué no valía `/motor_status`, que era lo más barato que había:** se republica cada segundo
con el último valor conocido, así que **llega exactamente igual con el RVR mudo**. Un topic que
sigue llegando cuando el robot dejó de hablar es peor que no tener nada.

🔴 **Avanza aunque el RVR no conteste, a propósito.** Es la vida del *nodo*, no la del robot.
Condicionarlo a que lleguen datos perdería justo el caso que se quiere cubrir: «la Pi va bien y
el RVR no está».

### `bool parada_emergencia` — la bandera, tal cual

Es `self._parada_emergencia`, que existía desde siempre y **no se publicaba en ninguna parte**.
La web solo podía decir «parada **enviada**», nunca «parada **activa**».

**Por qué importa:** este botón ha fallado en silencio **cinco veces** en este proyecto (nombre
de topic en ROS 1, namespace, QoS, el arranque solo al liberarla, y `rclpy.init()` invalidando
su propio contexto), y **cuatro de ellas devolvían 200 OK con cero efecto en el robot**. Con
este campo la web puede comprobar **el efecto**, que es la regla de oro del proyecto.

### `bool rvr_responde` + `float32 antiguedad_muestra_s` — Pi viva contra RVR vivo

`false` = hace más de `silence_timeout` (3 s por defecto) que no llega una muestra **del RVR**.

**Qué resuelve:** desde el navegador, «la Pi no responde» y «el RVR no contesta» son
**indistinguibles** — en los dos casos la web simplemente deja de recibir. Con `latido`
avanzando y `rvr_responde` en `false`, el diagnóstico es inequívoco y nadie tiene que cruzar el
campus a mirar.

`antiguedad_muestra_s = -1.0` significa **no ha llegado ninguna desde que arrancó el nodo**. En
este proyecto **-1.0 es siempre «no se sabe», nunca «cero»** — misma convención que las tres
`antiguedad_*_s` de `MotorStatus.msg`, y por la misma razón: publicar un `0.0` que en realidad
quiere decir «nadie ha dicho nada todavía» es falsa tranquilidad.

### `uint32 reanudaciones_fallidas` — CARGANDO contra DORMIDO

🔴 **Este es el campo que sale de una medida concreta, y el que tiene la trampa.**

El 2026-08-02 se apagó el RVR para cargarlo **con la Pi encendida** —un estado **cotidiano** del
laboratorio que nadie había probado— y el driver escribió **8 veces «streaming reanudado» en
30 s** mientras `/odom` daba **0 mensajes en 15 s**, con 123 intentos de reconexión seguidos.
Lo imprimía porque **`wake`+`stop`+`start` no lanzan excepción con el robot apagado**, no porque
volviera un dato. *Un robot muerto parecía sano en el log.* (Evidencia 52.)

El campo separa los dos casos apoyándose en que se comportan distinto: **un RVR dormido se
recupera al primer intento; uno apagado no se recupera nunca.**

| Valor | Lectura |
|---|---|
| `0` | todo bien |
| `1-2` | pudo ser una siesta, y se está recuperando sola |
| `>2` | el RVR no está ahí: apagado, cargando, o el cable fuera |

⚠️ **Esos umbrales son una lectura razonada de esa medida, no un umbral calibrado.** Nadie ha
cronometrado cuántos intentos tarda una siesta real en recuperarse. Trátalos como orientación
hasta que se midan.

---

## 3 · La decisión de diseño que hay que revisar con ojo

**El contador se pone a cero cuando LLEGA UNA MUESTRA, no cuando la reanudación «tiene éxito».**

Si se hubiera apoyado en que `_recuperar_streaming()` termina sin excepción, habría reproducido
el fallo del 2026-08-02 **dentro del campo escrito para detectarlo**.

Y hay una segunda trampa, más fina, que es la razón de la única parte no trivial del parche:
**`_t_ultima_muestra` no sirve para esto.** Ese campo lo reinician también `_conectar_rvr` y
`_recuperar_streaming` —a propósito, para que el vigilante no vuelva a disparar mientras la
recuperación está en curso—, así que dice *«hace poco que pasó algo»*, no *«hace poco que llegó
un dato»*. Una reanudación con el RVR apagado lo dejaría a cero y **parecería un éxito**.

Por eso se añaden dos campos **espejo que solo tocan los handlers del SDK**:

```python
self._t_muestra_real   # solo lo escriben los 4 handlers, nunca el reconectador
self._n_muestras       # contador monótono de muestras REALES
```

y el éxito se decide comparando `_n_muestras` contra la foto que se tomó al lanzar el intento.
Los cuatro sitios son los cuatro donde ya se marcaba el latido del enlace: `_quiza_publicar`,
`_h_color`, `_h_luz`, `_h_encoders`. La línea añadida **reusa el reloj que la línea de arriba ya
leyó**, así que no cuesta una lectura más a 16 Hz.

---

## 3 bis · 🔴 EL TERCER ESTADO, encontrado desde el robot antes de fusionar (2026-08-04)

Los espejos de arriba avanzan en `_quiza_publicar` **antes** del `return` que se toma cuando aún no
han llegado los cinco componentes de `/odom`. Eso es **correcto** —lo que vigilan es que el RVR siga
enviando, y no hay que confundir «faltan componentes» con «el enlace calló», que son dos fallos
distintos—, pero deja un estado que **no detecta nadie**:

| estado | latido | `rvr_responde` | `/odom` | |
|---|---|---|---|---|
| todo bien | avanza | `true` | publica | |
| la Pi va y el RVR no | avanza | `false` | 0 Hz | ✅ cubierto |
| **llegan 4 de los 5 componentes** | **avanza** | **`true`** | **0 Hz** | 🔴 **nadie avisa** |

Desde el muro del profesor ese robot **se pinta verde con la odometría muerta**. Y no es
hipotético: es el estado que **no se pudo descartar el 2026-08-04** —los cinco topics del stream a
cero, el RVR contestando a consultas y el vigilante callado—. Se cerró apagando el robot, así que
nunca se supo si faltaban todos los componentes o solo uno.

**Se cierra con `float32 antiguedad_odom_s`**, un tercer reloj que se pone **después** del `return`,
o sea que solo avanza cuando un `/odom` se completa de verdad. El discriminante que le da a la web:

```
antigüedad_muestra ~0   ·  antigüedad_odom CRECE   ->  faltan componentes
las dos crecen                                     ->  el RVR calló
```

⚠️ **No valía un `odom_completo: bool`**, y conviene saber por qué: `_recibidos` **se vacía en cada
ciclo** y los componentes llegan asíncronos a 16,5 Hz, así que en un instante cualquiera está medio
lleno **con el robot sano**. Muestreado a 1 Hz diría «incompleto» casi siempre. **Lo que hay que
medir es cuánto hace que se completó uno, no si lo está ahora.**

---

## 4 · Lo que SÍ se ha comprobado (sin robot)

```
✅ python -m py_compile rvr_driver_node.py        -> OK
✅ python -m py_compile robot.launch.py           -> OK
✅ "msg/EstadoRobot.msg" está en CMakeLists.txt   -> línea 38
✅ el diff no cambia ninguna línea existente salvo cuatro, y las cuatro son las pedidas:
     · la lista LEER del launch (insertar /estado_robot)
     · el import de atriz_rvr_msgs.msg (reflujo para añadir EstadoRobot)
     · el rótulo «Mensajes (6)» -> «(8)»
```

⚠️ **Y una que NO cuenta como verificación:** la máquina de estados del contador se probó con
una **simulación de sobremesa** que reproduce los cuatro casos (sano → 0; siesta corta → sube a
1 y vuelve a 0; apagado 20 s → sube a 5 y vuelve a 0 al reaparecer; muerto para siempre → sube
sin parar). **Eso es un modelo, no una medida** — misma categoría que `simular_sobregiro.py`,
que este proyecto renombró justamente por eso. Valida el razonamiento; **no valida el código que
corre en el robot**, porque la simulación es una reescritura de la lógica, no la lógica misma.

---

## 5 · Cómo verificarlo cuando haya robot

### Paso 0 · Antes de nada

```bash
git -C ~/atriz_ws/src/Atriz_rvr fetch origin
git -C ~/atriz_ws/src/Atriz_rvr checkout feat/estado-robot
git -C ~/atriz_ws/src/Atriz_rvr status -sb
sudo systemctl stop atriz-robot      # si no, se pelea por /dev/rvr
```

### Paso 1 · Compilar el paquete de mensajes EN LIMPIO

🔴 **UN `.msg` NUEVO NO BASTA CON `colcon build`.** Está medido en este proyecto: se añadió un
campo, el build dijo «2 packages finished», y el `.msg` **instalado** seguía sin él — el
suscriptor daba `AttributeError`.

```bash
rm -rf ~/atriz_ws/build/atriz_rvr_msgs ~/atriz_ws/install/atriz_rvr_msgs
bash ~/atriz_migracion/scripts/compilar.sh --limpio atriz_rvr_msgs    # ~4.5 min
bash ~/atriz_migracion/scripts/compilar.sh atriz_rvr_driver
```

⚠️ **Usa `compilar.sh`, no `colcon build` a pelo.** Lanzado desde el directorio equivocado,
colcon crea un workspace parásito, dice «Finished» y **no instala nada**. Se cometió seis veces
en una sola sesión estando ya documentado.

### Paso 2 · Comprobar el EFECTO, no el mensaje del build

```bash
# El .msg INSTALADO, con ruta absoluta (una relativa acabaría mirando el parásito)
grep -c reanudaciones_fallidas \
  /home/sphero/atriz_ws/install/atriz_rvr_msgs/share/atriz_rvr_msgs/msg/EstadoRobot.msg
# esperado: >= 1     ·     si da 0 o «no such file», el build NO instaló el mensaje

ls -d ~/atriz_ws/src/*/build 2>/dev/null && echo "🔴 hay workspace parásito: bórralo"
```

### Paso 3 · Arrancar y mirar que el latido AVANZA

```bash
sudo systemctl start atriz-robot     # o el launch a mano, con el servicio parado
sleep 5
ros2 topic echo /estado_robot --once
```

Esperado (**NO VERIFICADO**, es lo que el código pretende):

```
latido: <un numero>          parada_emergencia: false
rvr_responde: true           antiguedad_muestra_s: ~0.06
reanudaciones_fallidas: 0
```

🔴 **Que salga un mensaje no basta: hay que ver que `latido` AVANZA.** Es la mitad del sentido
del campo, y un `--once` no lo enseña.

```bash
for i in 1 2 3; do ros2 topic echo /estado_robot --once | grep latido; sleep 1; done
# esperado: tres numeros distintos, subiendo de uno en uno
```

📝 **No uses `ros2 topic echo --no-daemon`**: falla ~2 de cada 3 veces con «Could not determine
the type for the passed topic», con el topic publicando perfectamente.
📝 `ros2 topic hz /estado_robot` **sí vale** (debería dar ~1.0), pero **no canalices su salida**
(`| tail`): Python pasa a buffer de bloque y sale vacío, que se lee como «no mide». Usa
`stdbuf -oL` o míralo en la terminal.

### Paso 3 bis · 🔴 El tercer estado: `/odom` muerto con el enlace vivo

Es el que da sentido a `antiguedad_odom_s`, y **se puede provocar sin tocar el hardware**: basta con
que un componente de `/odom` deje de llegar. Con el robot sano las dos antigüedades tienen que ir
pegadas a ~0,06 s:

```bash
ros2 topic echo /estado_robot --once | grep -E "antiguedad_(muestra|odom)_s"
```

Y si algún día `/odom` se para con el robot aparentemente sano, **esto es lo primero que hay que
mirar**: si `antiguedad_muestra_s` sigue en ~0 y `antiguedad_odom_s` crece, faltan componentes y
reiniciar el streaming **no va a arreglar nada** — es otro fallo.

⚠️ **NO VERIFICADO**: nadie ha provocado la ausencia de un solo componente.

### Paso 4 · La parada de emergencia

```bash
ros2 topic pub --once /emergency_stop std_msgs/msg/Empty '{}'
ros2 topic echo /estado_robot --once | grep parada_emergencia     # esperado: true
ros2 service call /release_emergency_stop std_srvs/srv/Empty '{}'
ros2 topic echo /estado_robot --once | grep parada_emergencia     # esperado: false
```

### Paso 5 · 🔴 El caso que da sentido a `reanudaciones_fallidas`

**Este es el que hay que hacer sí o sí, y es el que no se pudo hacer sin robot.**

⚠️ Acción física: hay que **apagar el RVR con la Pi encendida** — el estado cotidiano de
«robot en el cargador». El LIDAR se alimenta del RVR, así que además reproduce el descriptor
muerto del nodo del lidar; eso es esperable y se arregla con `systemctl restart atriz-robot` al
terminar.

```bash
# En una terminal, mirando el estado a 1 Hz:
ros2 topic echo /estado_robot | grep -E 'latido|rvr_responde|reanudaciones|antiguedad'

# En otra: APAGA EL RVR (boton del robot) y espera ~30 s.
```

Esperado (**NO VERIFICADO**):

| | antes | ~5 s después de apagar | ~30 s después |
|---|---|---|---|
| `latido` | sube | **sigue subiendo** | **sigue subiendo** |
| `rvr_responde` | `true` | `false` | `false` |
| `antiguedad_muestra_s` | ~0.06 | crece | crece |
| `reanudaciones_fallidas` | `0` | `1` | **>2, subiendo ~1 cada 4 s** |

🔴 **Si `reanudaciones_fallidas` se queda en 0 o vuelve a 0 con el robot apagado, el parche está
MAL** — significa que el reseteo se está disparando con el reloj falso de la reanudación, que es
exactamente el fallo que intenta detectar.

Y al **volver a encender el RVR**: `reanudaciones_fallidas` debe caer a **0** en 1-2 s, y
`rvr_responde` a `true`.

### Paso 6 · Contra el caso real de una siesta (opcional, 5 min de espera)

```bash
ros2 launch atriz_rvr_bringup robot.launch.py keepalive_period:=0.0
# El RVR se duerme a los 300.6 s exactos (medido dos veces, 2026-07-31).
```
Esperado: `reanudaciones_fallidas` sube a **1** y vuelve a **0** — que es lo que distingue una
siesta de un robot apagado.

### Paso 7 · La lista blanca de rosbridge

🔴 **rosbridge deniega en silencio**: registra un `warn` y hace `return`, sin respuesta de error
al cliente. Una lista mal puesta se manifiesta como «la web no responde», no como un fallo.

```bash
python3 ~/atriz_migracion/00_auditoria/evidencia/mediciones_banco/probar_lista_blanca.py
```

Y desde el navegador, `03_operacion/probar_conexion_web.html`, comprobando que
`/estado_robot` **se puede leer** y que **NO se puede publicar** en él.

⚠️ **Al suscribirse desde la web, NO mandes campo `qos`.** En rosbridge el primer cliente que se
suscribe a un topic **impone el QoS a todos los demás**, y uno que pida un perfil incompatible
deja **mudas** a todas las pestañas de ese robot, sin aviso (medido el 2026-08-04, evidencia 68).

### Paso 8 · Que no se haya roto nada de lo de antes

```bash
bash ~/atriz_migracion/scripts/verificar_robot.sh --hardware
python3 ~/atriz_migracion/00_auditoria/evidencia/mediciones_banco/medir_ritmo_ros2.py
```

🔴 **Lo que hay que mirar es que `/odom` e `/imu` sigan a ~16,5 Hz.** El riesgo real de este
parche no es que `/estado_robot` no funcione: es que **se lleve por delante la telemetría**. El
2026-07-31 una `AttributeError` dentro de un manejador dejó `/odom` e `/imu` a cero **sin una
sola línea en el log**, con los topics existiendo y `/scan` funcionando. Por eso
`_publicar_estado` va entero dentro de un `try/except Exception` — pero **eso hay que
comprobarlo, no darlo por hecho**.

---

## 6 · Qué queda NO VERIFICADO

**Todo el comportamiento.** Sin excepciones. En concreto:

- ❌ Que el paquete de mensajes **compile** (no se ha ejecutado `colcon`; solo `py_compile` sobre
  los `.py`, que no toca el `.msg`).
- ❌ Que el topic **se publique**, y a 1 Hz.
- ❌ Que `latido` avance.
- ❌ Que `rvr_responde` pase a `false` cuando toca, y que el umbral de 3 s sea el correcto en la
  práctica (`/imu` tiene una dispersión del ±11 % sin explicar; no se ha comprobado si eso puede
  producir un `false` espurio).
- ❌ Que `antiguedad_muestra_s` dé `-1.0` durante el arranque y un valor sensato después.
- ❌ **Que `reanudaciones_fallidas` distinga de verdad CARGANDO de DORMIDO.** Es el campo con más
  lógica nueva y el único cuyo valor no es una copia de algo que ya existía.
- ❌ Que la lista blanca deje leer `/estado_robot` por rosbridge.
- ❌ Que **nada de esto degrade `/odom`, `/imu` ni la CPU** del driver (~29,5 % de un núcleo).

Lo único que se afirma es que **compila** y que **el diff solo añade**.
