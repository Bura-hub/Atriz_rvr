# 📐 Referencias — la API completa de `atriz.py`

Esta es la referencia de cada método, lo que devuelve, sus límites, y **por qué**
existe cada protección — con la medida que la respalda. Si buscas cómo empezar,
ve a `00_LEEME_PRIMERO.md`; si buscas el recorrido guiado, `GUIA_PASO_A_PASO.md`.

📖 Esta referencia cubre la API pensada para el curso. La documentación técnica
completa del driver y de por qué cada protección está donde está —incluida la
razón de fondo, con más detalle del que necesita un curso de 16 h— vive en el
documento `API_LABORATORIO.md` del repositorio de operación del laboratorio
(`atriz_migracion`), que es privado y no se enlaza desde aquí.

## Contenido
1. [Cómo se usa](#cómo-se-usa)
2. [Movimiento](#movimiento)
3. [Sensores](#sensores)
4. [Luces y parada de emergencia](#luces-y-parada-de-emergencia)
5. [Constantes del laboratorio](#constantes-del-laboratorio)
6. [Errores](#errores)
7. [Por qué `/cmd_vel_raw` y no `/cmd_vel`](#por-qué-cmd_vel_raw-y-no-cmd_vel)
8. [FAQ](#faq)

---

## Cómo se usa

```python
from atriz import Robot

with Robot() as robot:
    robot.avanzar(0.20, 3)
    robot.girar(90)
```

`Robot()` se conecta al construirse: enciende el barrido del LIDAR y comprueba si
el sensor de color está activo. Usa siempre `with`: al salir del bloque (aunque
sea por una excepción) el robot para y el barrido se apaga.

```python
Robot(velocidad_maxima=0.40)
```
El único parámetro del constructor limita la velocidad lineal máxima de **ese**
programa, por debajo del tope del laboratorio (`VEL_MAX`, ver más abajo). No se
puede subir por encima de él.

---

## Movimiento

### `robot.avanzar(velocidad, segundos)`

Avanza a `velocidad` m/s durante `segundos` y para. **Bloquea** hasta terminar.

- `velocidad` negativa: retrocede.
- Se recorta a `±0.40` m/s (`VEL_MAX`) y a `10.0` s (`TIEMPO_MAX`). Si pides más,
  el programa lo dice por pantalla y sigue con el valor recortado — no lanza
  ni se para en silencio.
- Republica la orden 10 veces por segundo. Es necesario: el driver tiene un
  watchdog que corta la velocidad a cero si no recibe una orden nueva en 0.3 s.

### `robot.girar(grados)`

Gira `grados` sobre el propio eje. Positivo = izquierda, negativo = derecha
(convención REP-103). **Devuelve** los grados que giró de verdad, medidos leyendo
la odometría mientras gira — no lo que se le pidió.

```python
logrado = robot.girar(90)      # gira ~90° y devuelve, p. ej., 87.3
```

Es un **lazo cerrado**: mide el rumbo real y para cuando llega, en vez de girar
durante un tiempo fijo calculado con una fórmula. La razón: pidiendo 90° por
tiempo, este robot da 86.6 / 86.2 / 87.7° (n=3, medido) — un déficit que no
desaparece subiendo la batería. `girar()` mide y corrige eso solo.

Se recorta a `±720°` (`GRADOS_MAX`). Tiene un tope de tiempo interno (proporcional
al ángulo pedido) para no girar indefinidamente si el robot se queda atascado; si
salta, avisa por pantalla con el ángulo que sí logró.

### `robot.girar_por_tiempo(velocidad, segundos)`

Gira a `velocidad` rad/s durante `segundos`, **sin medir nada** (lazo abierto).

⚠️ Existe **solo** para la práctica 4, que compara este lazo abierto contra el
lazo cerrado de `girar()`. Para girar de verdad, usa `girar(grados)`.

### `robot.parar()`

Velocidad cero, repetida cinco veces por si se pierde algún mensaje por el
camino.

### `robot.mover(velocidad, giro)`

Manda **una sola** orden de velocidad (lineal en m/s, angular en rad/s) y vuelve
enseguida — no bloquea ni republica.

⚠️ A diferencia de `avanzar()`, esta es para lazos de control propios: **tienes
que llamarla tú, en tu bucle, más de tres veces por segundo**. Si el hueco entre
dos llamadas supera 0.3 s, el watchdog del driver corta la velocidad a cero y el
robot va a tirones. Así se usa en `seguidor_linea_pid_demo.py`.

### `robot.rumbo()`

El rumbo actual en grados, leído de la odometría (`/odom`).

⚠️ La odometría deriva **~1°/30 s** los primeros minutos tras encender el RVR, y
baja a **0.001°/30 s** pasados unos siete minutos (medido). Sobre una práctica de
15 minutos con el robot recién encendido, eso son decenas de grados de error
acumulado. Deja pasar unos minutos antes de una medida que necesite precisión.

---

## Sensores

### `robot.color()`

Devuelve `(rojo, verde, azul, claro)`.

- El canal que de verdad discrimina es **`claro`**: ~181 sobre negro, ~2288 sobre
  blanco (12.6× de rango), frente a un RGB que apenas se mueve.
- Se normaliza por **verde**, el canal más sensible: sobre rojo, R/G sube de 0.48
  a 2.74; sobre azul, B/G sube a 0.86.
- 🔴 **Lanza `ErrorAtriz`** si la lectura falla de verdad (el driver contesta
  `success=False`). Es a propósito: sin esta comprobación, una lectura fallida
  y una lectura real sobre negro serían indistinguibles — los dos casos dan
  ceros.
- Si el robot arrancó **sin** el sensor de color activo (`robot.hay_color` es
  `False`), avisa por pantalla y los cuatro canales son ruido de fondo (oscilan
  entre 0 y 1), no una lectura real. El sensor de color se activa **en el
  arranque del robot**, no bajo demanda: si no está activo, hay que pedirle al
  profesor que reinicie el robot con `color_detection:=true`.

### `robot.hay_color`

Atributo (no un método): `True`/`False`, fijado al construir `Robot()`. Dice si
este robot arrancó con el sensor de color activo. Compruébalo antes de fiarte de
`color()`.

### `robot.distancia_frontal()`

Metros hasta el objeto más cercano en un cono de ±10° por delante del robot,
leído del LIDAR.

⚠️ Un solo barrido no ve bien un objeto fino: a 0.68 m el LIDAR tira un rayo cada
1.7 cm, así que algo de 5 cm de ancho puede dar solo 2-3 puntos y desaparecer en
algún barrido suelto. Para geometría fina hace falta acumular varios barridos, no
uno solo.

Lanza `ErrorAtriz` si no hay ningún punto válido en ese cono (por ejemplo, si el
robot mira hacia un espacio muy abierto y fuera de rango).

### `robot.bateria()`

🔴 **Voltios**, no porcentaje. El porcentaje del firmware llegó a marcar 100 % con
la batería a 8.29 V — a solo 1.29 V del umbral de «baja» del propio firmware
(7.0 V; crítica 6.5 V). Es una estimación gruesa; el voltaje no.

---

## Luces y parada de emergencia

### `robot.luces(rojo, verde, azul)`

Pone todos los faros del robot a un color. Cada canal es un **entero** de 0 a
255.

🔴 Lanza `ErrorAtriz` si algún canal no es exactamente eso — incluido si le pasas
`True`/`False` en vez de un número: en Python `bool` es subclase de `int`, así
que `luces(True, True, True)` pasaría silenciosamente como RGB `(1,1,1)`
(prácticamente apagado) si no se comprobara el tipo antes de convertir.

### `robot.parada_emergencia()`

Detiene el robot de forma explícita: el driver **descarta todo comando de
movimiento** hasta que alguien la libere.

🔴 **No se libera sola, y no la libera tu programa al terminar.** Liberarla es un
acto explícito del profesor:
```bash
ros2 service call /release_emergency_stop std_srvs/srv/Empty
```
Si tu programa la dispara y termina, **el robot se queda mudo para el siguiente
grupo que lo use** hasta que alguien la libere a mano. No es un botón de pausa.

---

## Constantes del laboratorio

Definidas en `atriz.py`, cada una con una medida detrás — no se cambian sin una
medida nueva:

| Constante | Valor | De dónde sale |
|---|---|---|
| `VEL_MAX` | 0.40 m/s | meseta real medida: 0.401 m/s comandando 0.40 |
| `VEL_GIRO_MAX` | 2.0 rad/s | 99–102 % del comandado en las cuatro velocidades medidas |
| `TIEMPO_MAX` | 10.0 s por llamada | tope de diseño, no una medida |
| `GRADOS_MAX` | 720.0 ° por llamada | ídem |
| `RITMO_HZ` | 10.0 Hz | el watchdog del driver corta a los **0.3 s** sin recibir una orden |
| `TOPIC_MANDO` | `/cmd_vel_raw` | ver la sección siguiente |

---

## Errores

Todo lo que puede fallar en un servicio o una espera lanza `ErrorAtriz`, con un
mensaje que dice qué comprobar (por ejemplo, si el robot está encendido). Ningún
método de `atriz.py` se queda esperando para siempre: cada espera tiene un
tiempo límite.

---

## Por qué `/cmd_vel_raw` y no `/cmd_vel`

`atriz.py` publica las órdenes de movimiento en `/cmd_vel_raw`, **no** en
`/cmd_vel`. No es un detalle de implementación: `/cmd_vel` es la **salida** de la
capa de seguridad (`collision_monitor`), no una entrada. Escribir directamente en
`/cmd_vel` funcionaría — el driver lo obedecería — pero **saltaría entera la capa
de seguridad**, sin ningún aviso. Por eso ningún método de esta biblioteca
publica ahí, y por eso tampoco deberías hacerlo tú a mano si escribes ROS 2
directamente contra este robot.

---

## FAQ

### ¿Por qué mi robot va mucho más despacio de lo que le pedí?

El polígono de precaución del `collision_monitor` frena al **40 %** si hay algo a
menos de 0.36 m por delante — **aunque el robot se esté alejando de eso**. Un
retroceso comandado de 30 cm puede medir solo 14 cm si hay algo delante (no
detrás). No es que el robot desobedezca: es la capa de seguridad actuando sobre
un polígono fijo, no sobre la dirección de movimiento.

### ¿Por qué `girar(90)` no da exactamente 90.0?

Porque mide y para, no calcula y confía. El número que devuelve es el ángulo real
medido con la odometría — normalmente unas décimas por encima o por debajo de lo
pedido, según cómo responda el robot en ese momento (batería, suelo, deriva de la
odometría).

### ¿Cómo detengo el robot ahora mismo, en el código?

`robot.parar()`. Fuera de eso, Ctrl-C está pensado para parar cualquier programa
de esta biblioteca de forma segura.

### ¿Cómo detengo un robot que se ha vuelto loco, sin acceso al código?

La parada de emergencia (ver arriba). Es la última opción: deja el robot mudo
hasta que el profesor la libere, así que no la uses como forma normal de terminar
un programa.

### El LIDAR debería ver este obstáculo bajo, y no lo ve

El LIDAR barre a **15.5 cm del suelo**. Cualquier cosa por debajo de esa altura es
invisible para `distancia_frontal()` y para la capa de seguridad. «Despejado a
ras de suelo» no es lo mismo que «despejado a la altura del LIDAR».
