# 📚 Guía paso a paso — el recorrido de las 16 horas

Una sección por práctica, en el orden en que están pensadas:
`01 → 02 → 03 → 04 → 05 → 10 → 11 → seguidor → 90 → 99`.

Antes de empezar, lee `00_LEEME_PRIMERO.md` si no lo has hecho — en particular el
punto 4 (por qué el robot no obedece sin `/scan`) y el punto 6 (los cuatro
síntomas que no son fallos).

---

## 🔴 Antes de nada: qué valor tienen los «Qué debería verse» de esta guía

Cada práctica de abajo trae una sección **«Qué debería verse»**. Léelas sabiendo
esto:

> **Ninguno de estos guiones se ha ejecutado nunca contra el robot moviéndose.**
> Las ocho secciones «Qué debería verse» describen lo que el código **manda
> hacer** — son aritmética de velocidad × tiempo leída del código, no un robot
> que alguien haya visto hacerlo. La lista completa de lo que está medido y lo
> que no está en el **punto 8 de `00_LEEME_PRIMERO.md`**.

No es un descargo de responsabilidad: es el encargo. **La primera medida real de
casi todo esto la vas a tomar tú**, y por eso casi todas las prácticas terminan
pidiéndote una cinta métrica o un transportador. Si tu medida no coincide con lo
que dice esta guía, **tu medida gana**. Anótala y díselo al profesor.

---

## Cuánto espacio necesita cada práctica (haz la cuenta antes de ejecutar)

| Práctica | Por delante | Otros |
|---|---|---|
| 01 Avanzar | **1.5 m** (el ejercicio 3 recorre 1.20 m) | **~1 m por detrás** para el ejercicio 2 |
| 02 Girar | — | 40 cm alrededor |
| 03 Cuadrado | — | cuadrado libre de ~1.5 m de lado |
| 04 Giro preciso | — | 40 cm alrededor + transportador |
| 05 Sensor de color | — | el robot no se mueve |
| 10 Patrulla | **3 m** en la dirección en que mire | — |
| 11 Parar sobre negro | **1 m** + cinta negra cruzando | **40 cm por detrás** para el ejercicio 3 |
| Seguidor de línea | pista con línea negra, **≥6 m de recorrido** | — |
| 90 Plantilla | **1 m** (avanza 40 cm y gira) | 40 cm alrededor |
| 99 Prueba de Ctrl-C | **1.5 m** | — |

⚠️ **Siempre despejado a la altura del LIDAR: 15.5 cm del suelo.** Una caja baja no
la ve ni el programa ni la capa de seguridad. Y **hacia atrás no hay capa de
seguridad**: el polígono del `collision_monitor` mira hacia delante.

---

## Práctica 01 — Avanzar

**Qué se aprende:** el ciclo básico de un programa contra `atriz.py`: `with
Robot()`, una orden, y listo.

**Qué hace falta en el suelo:** **1.5 m despejados por delante**, no 1 m — la
cuenta es del peor caso, que es el ejercicio 3: pedir 1.5 m/s se recorta a 0.40
m/s y sale `0.40 × 3 s = 1.20 m`, el doble que el caso base. Y para el ejercicio 2
hacen falta además **~1 m por detrás**. El LIDAR barre a 15.5 cm del suelo, así
que una caja baja no la detecta.

**Comando:**
```bash
python3 01_avanzar.py
```

**Qué debería verse:** el robot avanza y para solo. `0.20 m/s × 3 s` son 60 cm de
recorrido **comandado**; 🔴 lo que recorre de verdad **no está medido** (la rampa
de aceleración de ~0.5 s se come parte). Eso es justo lo que mides en el
ejercicio 1.

**Ejercicios:**
1. Cambia la velocidad a 0.30. Mide con una cinta: ¿avanzó la mitad más?
   (recorrido: 0.90 m). **Anota el número: es la primera medida real de esto.**
2. Pon una velocidad negativa. ¿Qué hace?
   🔴 **El robot va hacia ATRÁS 60 cm.** Comprueba que hay ~1 m detrás de él, y
   mira dónde estás tú. Detrás **no hay capa de seguridad**: el polígono del
   `collision_monitor` mira hacia delante.
3. Pide 1.5 m/s. El robot no llega ahí — mira lo que imprime el programa cuando
   recorta la velocidad al máximo del laboratorio. Recorre **1.20 m**: por eso la
   práctica pide 1.5 m de espacio y no 1 m.

---

## Práctica 02 — Girar

**Qué se aprende:** `robot.girar(grados)` — un giro que mide el rumbo real y
**devuelve** cuánto giró de verdad, no solo lo que se le pidió.

**Qué hace falta en el suelo:** unos 40 cm libres alrededor del robot.

**Comando:**
```bash
python3 02_girar.py
```

**Qué debería verse:** el robot gira 90° a la izquierda y el programa imprime el
ángulo real medido, que **no será exactamente 90.0**.

🔴 **Cuánto se desvía NO ESTÁ MEDIDO.** Una versión anterior de esta guía decía
«unas décimas», y ese número no salía de ningún sitio. Lo único que se puede
afirmar es aritmética: el lazo comprueba el rumbo cada 0.05 s y en el último tramo
va a 0.20 rad/s, así que **solo la granularidad del bucle ya son 0.573°** — y a eso
hay que sumarle lo que el robot siga rodando tras la orden de parar, que **nadie ha
medido**. O sea que «unas décimas» era, como poco, optimista.
**El ejercicio 2 de abajo es el que va a producir el primer dato real.** Si tu
transportador dice otra cosa que este párrafo, gana el transportador.

**Ejercicios:**
1. Gira -90. ¿Hacia dónde va?
2. Comprueba con un transportador cuánto giró. ¿Coincide con lo que imprime el
   programa?
3. ¿Por qué `girar()` devuelve un número en vez de no devolver nada?

---

## Práctica 03 — Un cuadrado

**Qué se aprende:** encadenar `avanzar()` y `girar()` para dibujar una figura.

**Qué hace falta en el suelo:** un cuadrado libre de ~1.5 m de lado.

**Comando:**
```bash
python3 03_cuadrado.py
```

**Qué debería verse:** cuatro lados y cuatro giros de 90°, con el ángulo real de
cada esquina impreso por pantalla. Cada lado son `0.20 m/s × 3 s` = **60 cm
comandados**; 🔴 el lado real será algo más corto por la rampa de aceleración de
~0.5 s, y **cuánto no está medido**. Mídelo en el ejercicio 1.

**Ejercicios:**
1. ¿Vuelve el robot al punto de partida? Márcalo con cinta y mide el error.
2. Suma los cuatro giros que imprime. ¿Cuánto se aleja de 360?
3. Haz un triángulo. ¿Cuántos grados hay que girar en cada esquina?

---

## Práctica 04 — Girar bien: lazo abierto contra lazo cerrado

**Qué se aprende:** la diferencia entre pedir un giro **por tiempo** (lazo
abierto, `girar_por_tiempo()`) y pedirlo **por ángulo** (lazo cerrado, `girar()`).

Este robot tiene un déficit medido girando en lazo abierto: 90° pedidos dieron
**86.6 / 86.2 / 87.7°** (n=3), y no depende de la batería (se midió del 55 % al
100 %).

🔴 **Pero esos tres números NO son los de esta práctica**, y decir que lo eran era
atribuir mal una medida. Se tomaron con el servicio `move_timed` del driver **a
1.0 rad/s**. `girar_por_tiempo()` es **otro mecanismo** (publica en
`/cmd_vel_raw` a 20 Hz) y esta práctica lo usa **a 0.8 rad/s**: otro camino, otra
velocidad, otro número. **Lo que dé este guion NO ESTÁ MEDIDO** — lo vas a medir
tú con el transportador. Lo que los 86.6/86.2/87.7 sí demuestran es que el déficit
del lazo abierto existe en este robot y es de varios grados, que es toda la razón
que hace falta para cerrar el lazo.

📝 Esto es, además, la lección de método de la práctica: **antes de atribuir un
número a una causa, comprueba que se midió con esa causa.** El propio manual del
laboratorio se lo saltó aquí.

**Qué hace falta en el suelo:** ~40 cm libres alrededor, y un transportador o una
cinta para marcar el rumbo. **Deja el robot encendido unos 10 minutos antes de
medir**: la odometría deriva ~1°/30 s los primeros minutos tras encender el RVR
(ver el punto 6 de `00_LEEME_PRIMERO.md`), y ese ruido se confundiría con el error
del lazo abierto.

**Comando:**
```bash
python3 04_giro_preciso.py
```

**Qué debería verse:** el programa **para cuatro veces** (`input()`): dos por cada
lazo — una para que marques el rumbo de partida y otra para que midas con el
transportador. Primero el lazo abierto, luego el cerrado, y al final compara los
dos errores.

**Ejercicios:**
1. ¿Cuál de los dos se acercó más? ¿Cuánto?
2. Repite los dos tres veces. ¿Cuál **repite** mejor? (no es lo mismo que acertar)
3. Pon el robot sobre una alfombra y repite. ¿Cuál aguanta mejor el cambio?
4. ¿Qué necesita el lazo cerrado que el abierto no tiene? (pista: un sensor)
5. Cambia `OBJETIVO` a 360. El robot da la vuelta entera y el programa imprime
   **~0** para el lazo abierto. **¿Por qué sale ~0 si el robot sí giró?**
   (pista: mira el comentario de la línea `logrado_abierto = robot.rumbo() -
   antes`. Resta dos rumbos **absolutos**, y `atan2` solo devuelve −180..180, así
   que una vuelta completa acaba donde empezó. La forma que no tiene este
   problema es acumular el **incremento** de rumbo — es lo que hace `acumular()`
   en `atriz.py`, y por eso `girar(360)` sí funciona.)

---

## Práctica 05 — El sensor de color

**No necesita nada especial.** El programa enciende la luz del sensor él solo:

```python
robot.sensor_color(True)
```

El sensor de color lleva su propia luz debajo del robot, y sin ella no ve nada:
el canal `claro` pasa de **~1320 con la luz encendida a 1 apagada**. El robot
arranca con esa luz apagada a propósito —es un LED blanco bajo el chasis y gasta
batería—, así que la enciendes cuando vas a medir. `cerrar()` la apaga sola.

Si se te olvida encenderla, `robot.hay_color` sale `False`, el script te avisa y
te dice cómo encenderla — no te devuelve oscuridad haciéndola pasar por «negro».

📝 **Hasta el 2026-08-06 esto pedía que el profesor reiniciara el robot.** Se dio
por medido durante seis días que la luz no se podía encender en caliente, y era
falso. Ver el `CHANGELOG` de esa fecha: es un buen ejemplo de cómo una medición
mal diseñada bloquea algo que siempre funcionó.

**Qué se aprende:** `robot.color()` devuelve `(rojo, verde, azul, claro)`. El
canal que de verdad discrimina es `claro`: va de ~181 sobre negro a ~2288 sobre
blanco (12.6×), mientras el RGB apenas se mueve.

**Comando:**
```bash
python3 05_sensor_color.py
```

**Qué debería verse:** una tabla en vivo con los cuatro canales y las razones
R/G y B/G. Prueba distintas superficies bajo el robot.

**Ejercicios:**
1. Prueba blanco, negro, rojo y azul. ¿Qué columna cambia más?
2. `claro` va de ~181 sobre negro a ~2288 sobre blanco. ¿Y R/G?
3. ¿Por qué dividimos por verde en vez de usar el rojo a secas?

---

## Práctica 10 — Tu propia clase: un robot que patrulla

**Qué se aprende:** construir una clase propia (`Patrulla`) encima de `Robot`,
usando `distancia_frontal()` para decidir cuándo girar. Es como se organiza el
código de un robot de verdad: capas, cada una apoyada en la de abajo.

**Qué hace falta en el suelo:** al menos **3 m** despejados en la dirección en que
mire el robot al arrancar. 12 tramos de `avanzar(0.20, 1)` sin girar nunca son
**2.4 m** en línea recta, y eso pasa si el robot no detecta nada a menos de 0.35 m
por delante. 🔴 Y 2.4 m de recorrido no son «2.5 m de sitio»: los 10 cm que
sobrarían son **menos que el propio robot**, que mide 19 cm de largo — la cuenta se
hace desde el morro, no desde el centro. Con 3 m quedan ~40 cm de margen real.

**Comando:**
```bash
python3 10_movimiento_completo.py
```

**Qué debería verse:** el robot avanza en tramos de 1 s, imprime la distancia
frontal en cada uno, y gira 90° cuando algo queda a menos de 0.35 m.

**Ejercicios:**
1. Cambia `distancia_minima` a 0.60. ¿Gira antes o después?
2. Haz que gire -90 en vez de 90. ¿Cambia el recorrido?
3. Añade un método que encienda las luces en rojo cuando vaya a girar.
4. ¿Por qué `un_tramo` avanza solo 1 segundo y no 10?

---

## Práctica 11 — Reaccionar a lo que ve: parar sobre negro

Enciende la luz del sensor él solo, igual que la práctica 05.

**Qué se aprende:** leer el sensor **mientras** se avanza, en vez de bloquear con
`avanzar()`. El umbral (`UMBRAL_NEGRO = 400`) no está a mitad de camino entre
negro (~181) y blanco (~2288): el suelo real del laboratorio, sin cinta encima,
ya da `claro=1275` — casi tan alto como el blanco. Un umbral a mitad de recorrido
confundiría «suelo normal» con «casi blanco».

**Qué hace falta en el suelo:** 1 metro despejado por delante, y una franja de
cinta negra cruzando el camino del robot.

**Comando:**
```bash
python3 11_sensor_avanzado.py
```

**Qué debería verse:** el robot avanza en tramos cortos (10 cm/s, 0.2 s) hasta que
`claro` baja de 400, y entonces para. Hay un **tope de 0.90 m de recorrido** por si
la cinta no está puesta — a 0.10 m/s son 45 tramos, ~32 s.

🔴 **El tope está en METROS, no en número de tramos, y es un arreglo de un defecto
real.** Antes era `MAX_TRAMOS = 60` fijo, calculado para 0.10 m/s. Pero el
ejercicio 2 manda subir la velocidad a 0.30 — y subir la velocidad **no cambiaba el
tope**, así que los mismos 60 tramos pasaban de 1.20 m a `60 × (0.30 × 0.2)` =
**3.60 m**: tres veces y media el metro que esta práctica pide despejado. Y el tope
existe precisamente para cuando la cinta **no** se detecta, o sea justo cuando el
robot va a recorrerlo entero. Derivándolo de la distancia, el ejercicio 2 se
protege solo.

**Ejercicios:**
1. Baja el umbral a 200. ¿Se le pasa la línea?
2. Sube `VELOCIDAD` a 0.30. ¿Qué le pasa a la distancia de parada? Mira además lo
   que imprime al arrancar: `MAX_TRAMOS` baja de 45 a 15 **solo**. ¿Por qué el tope
   está escrito en metros y no en número de tramos?
3. Haz que retroceda 20 cm después de encontrar el negro.
   🔴 **El robot va hacia ATRÁS.** Comprueba que hay al menos 40 cm detrás de él,
   contando que el robot mide 19 cm de largo. Detrás **no hay capa de seguridad**:
   el polígono del `collision_monitor` mira hacia delante — y por eso frenará
   igual aunque te estés alejando de la pared (medido: 30 cm comandados hacia
   atrás dieron 14 cm), pero de lo que tengas detrás no te avisa nadie.

---

## Proyecto final — Seguidor de línea (`seguidor_linea_pid_demo.py`)

Enciende la luz del sensor él solo, igual que las prácticas 05 y 11.

**Qué se aprende:** un PID de verdad, combinado con seguimiento de **borde** en
vez de intentar centrarse sobre la línea — la explicación completa está en
`SEGUIDOR_LINEA_EXPLICACION.md`.

**Qué hace falta en el suelo:** una pista con una línea negra sobre suelo claro, y
**espacio para al menos 6 m de recorrido** — que es el tope que trae el guion
(`distancia_max_m` en `seguidor_config.json`). A 0.08 m/s son unos 75 segundos.
🔴 Son metros de **camino**, no en línea recta: en una pista con curvas el robot
recorre más metros de los que avanza, así que un circuito cerrado de 2 m de lado
vale de sobra, pero una pista recta necesita 6 m de verdad.

Coloca el robot mirando en la dirección de avance, con el sensor **justo sobre el
borde derecho de la línea**: la línea (negro) a su izquierda, el suelo (claro) a
su derecha.

**Comando:**
```bash
python3 seguidor_linea_pid_demo.py
```

**Qué debería verse:** el robot avanza siguiendo el borde de la línea, corrigiendo
el giro en proporción a cuánto se aleja del centro de la banda negro/claro. Ctrl-C
para parar, o el tope de 6 m lo para solo y te dice qué mirar.

🔴 **El tope no es un adorno.** Este bucle era un `while True` sin ningún límite: si
la pista se acaba, si el sensor se desalinea o si la hipótesis del lado del borde es
la contraria, el robot se iba recto **y en silencio** hasta que alguien lo cogía. La
práctica 11 ya puso tope por esa misma razón; aquí vale igual, y encima este es el
guion que se deja corriendo más rato.

⚠️ **Y esto nunca ha seguido una línea real.** Solo sus funciones puras tienen
tests. Es el guion menos verificado de la carpeta.

**Ejercicios** (los mismos que trae el script):
1. Pon `Kd` a 0. ¿Qué le pasa al robot en las curvas?
2. Sube `Kp` hasta que oscile. Eso es la ganancia crítica.
3. Sube `VELOCIDAD` a 0.20. ¿Sigue valiendo el mismo PID?
4. Sube `PERIODO` a 0.5. ¿Por qué va a tirones? (pista: el watchdog corta a los
   0.3 s sin recibir una orden nueva)

---

## Herramientas — `90_template.py`

No es una práctica con solución: es el punto de partida para tu propio programa.

**Qué hace falta en el suelo:** 🔴 **tal como viene, la plantilla MUEVE EL ROBOT**:
avanza `0.20 m/s × 2 s` = **40 cm** y gira 90° sobre el eje. Deja **1 m despejado
por delante y 40 cm libres alrededor**, a la altura del LIDAR (15.5 cm del suelo).
Es el único fichero de la carpeta que se copia para modificarlo, así que es fácil
ejecutarlo «solo para ver si arranca» — y arranca moviéndose.

```bash
cp 90_template.py mi_programa.py
python3 mi_programa.py
```

Trae en su cabecera la misma tabla de la API que `00_LEEME_PRIMERO.md`. Cópiala,
borra el bloque de ejemplo del medio y escribe lo tuyo ahí.

🔴 **Y cuando escribas lo tuyo, vuelve a hacer la cuenta del espacio.** Ya no vale
esta sección: el espacio sale de **tu** código. Suma los `avanzar()` que encadenes
(metros = velocidad × segundos), acuérdate de que `girar()` necesita sitio a los
lados, y si mandas velocidades negativas cuenta también el espacio **detrás** —
ahí no vigila nadie.

---

## Prueba 99 — ¿para el robot cuando pulsas Ctrl-C?

No es una práctica del recorrido: es la prueba que comprueba **la garantía en la
que se apoya todo lo demás**. Pásala al menos una vez, y sobre todo si vas a dejar
al robot haciendo algo largo.

**Qué se aprende:** que un mecanismo de seguridad no se da por bueno con una
pasada verde. Este Ctrl-C **ya falló** en este laboratorio (`rclpy.init()` instalaba
su propio manejador de la señal e invalidaba su contexto), y el fallo era
**intermitente** — según dónde cayera la pulsación, a veces sí paraba. Por eso la
prueba se corre varias veces.

**Qué hace falta en el suelo:** **1.5 m despejados por delante**. `avanzar(0.15, 8)`
son 1.2 m si nadie pulsa Ctrl-C, y el guion contempla expresamente que no lo pulses;
los otros 30 cm son para lo que el robot recorra **después** de la última orden, que
es justo lo que esta prueba mide.

**Comando:**
```bash
python3 99_test_ctrl_c.py
```

**Qué debería verse:** el robot avanza; al pulsar Ctrl-C imprime que está parando y
sale. **Cuánto recorre después de la pulsación NO ESTÁ MEDIDO** — ese número lo
produces tú, y es el resultado de la prueba.

**Ejercicios:** los cinco que trae el guion. El 4 y el 5 son los importantes:
distinguen **parar el robot** (lo hace el watchdog del driver, siempre) de **apagar
el barrido del LIDAR** (solo si la señal se puede capturar). Con `kill -9` la
respuesta cambia, y no se puede arreglar desde el programa.

---

## Las cuatro cosas que sorprenden y no son fallos

Con estas cuatro se explican casi todos los «esto no funciona» del curso:

| Síntoma | Qué es de verdad |
|---|---|
| El robot no se mueve y no hay error | Falta `/scan`, o hay una parada de emergencia enganchada. La libera el profesor |
| Va mucho más despacio de lo pedido | El polígono de precaución frena al **40 %** si hay algo a menos de 0.36 m, **aunque el robot se aleje**: 30 cm comandados → 14 medidos |
| Los ángulos se van acumulando mal | La odometría deriva **~1 °/30 s** los primeros minutos tras encender el RVR, y 0.001 siete minutos después |
| El LIDAR no ve una caja baja | Barre a **15.5 cm del suelo**. «Despejado a ras de suelo» no basta |
