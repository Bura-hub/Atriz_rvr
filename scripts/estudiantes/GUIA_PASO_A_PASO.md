# 📚 Guía paso a paso — el recorrido de las 16 horas

Una sección por práctica, en el orden en que están pensadas:
`01 → 02 → 03 → 04 → 05 → 10 → 11 → seguidor → 90`.

Antes de empezar, lee `00_LEEME_PRIMERO.md` si no lo has hecho — en particular el
punto 4 (por qué el robot no obedece sin `/scan`) y el punto 6 (los cuatro
síntomas que no son fallos).

---

## Práctica 01 — Avanzar

**Qué se aprende:** el ciclo básico de un programa contra `atriz.py`: `with
Robot()`, una orden, y listo.

**Qué hace falta en el suelo:** al menos 1 metro despejado por delante del robot.
El LIDAR barre a 15.5 cm del suelo, así que una caja baja no la detecta.

**Comando:**
```bash
python3 01_avanzar.py
```

**Qué debería verse:** el robot avanza unos 60 cm (0.20 m/s durante 3 s) y para
solo.

**Ejercicios:**
1. Cambia la velocidad a 0.30. Mide con una cinta: ¿avanzó la mitad más?
2. Pon una velocidad negativa. ¿Qué hace?
3. Pide 1.5 m/s. El robot no llega ahí — mira lo que imprime el programa cuando
   recorta la velocidad al máximo del laboratorio.

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
ángulo real medido (no exactamente 90.0, casi siempre unas décimas de diferencia).

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
cada esquina impreso por pantalla.

**Ejercicios:**
1. ¿Vuelve el robot al punto de partida? Márcalo con cinta y mide el error.
2. Suma los cuatro giros que imprime. ¿Cuánto se aleja de 360?
3. Haz un triángulo. ¿Cuántos grados hay que girar en cada esquina?

---

## Práctica 04 — Girar bien: lazo abierto contra lazo cerrado

**Qué se aprende:** la diferencia entre pedir un giro **por tiempo** (lazo
abierto, `girar_por_tiempo()`) y pedirlo **por ángulo** (lazo cerrado,
`girar()`). En este robot, pedir 90° por tiempo da 86.6 / 86.2 / 87.7° medidos
(n=3) — un déficit que no depende de la batería.

**Qué hace falta en el suelo:** ~40 cm libres alrededor, y un transportador o una
cinta para marcar el rumbo. **Deja el robot encendido unos 10 minutos antes de
medir**: la odometría deriva ~1°/30 s los primeros minutos tras encender el RVR
(ver el punto 6 de `00_LEEME_PRIMERO.md`), y ese ruido se confundiría con el error
del lazo abierto.

**Comando:**
```bash
python3 04_giro_preciso.py
```

**Qué debería verse:** el programa para dos veces (`input()`) para que marques el
rumbo con el transportador; primero con el lazo abierto, luego con el cerrado, y
al final compara los dos errores.

**Ejercicios:**
1. ¿Cuál de los dos se acercó más? ¿Cuánto?
2. Repite los dos tres veces. ¿Cuál **repite** mejor? (no es lo mismo que acertar)
3. Pon el robot sobre una alfombra y repite. ¿Cuál aguanta mejor el cambio?
4. ¿Qué necesita el lazo cerrado que el abierto no tiene? (pista: un sensor)
5. Cambia `OBJETIVO` a 360. Ojo: ¿por qué el lazo abierto no da 0?

---

## Práctica 05 — El sensor de color

🔴 **Necesita un arranque especial del robot, que hace el profesor:**
```bash
sudo systemctl stop atriz-robot
ros2 launch atriz_rvr_bringup robot.launch.py color_detection:=true
```
El sensor de color lleva su propia luz debajo del robot, y sin ella no ve nada
(el canal `claro` pasa de 741 con la luz encendida a 4 apagada — 185 veces menos).
Esa luz se enciende antes de configurar el sensor y **no se puede encender
después**, así que se decide en el arranque. El arranque normal la deja apagada a
propósito, porque es un LED blanco encendido todo el rato bajo el chasis.

Si el robot arrancó normal, `robot.hay_color` sale `False` y el script te lo dice
y sale — no te devuelve ceros haciéndolos pasar por «negro».

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

**Qué hace falta en el suelo:** al menos 2.5 m despejados en la dirección en que
mire el robot al arrancar — no basta un cuadrado de 1.5 m: 12 tramos de
`avanzar(0.20, 1)` sin girar nunca son 2.4 m en línea recta si el robot no detecta
nada a menos de 0.35 m por delante.

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

🔴 **Necesita el mismo arranque especial que la práctica 05**
(`color_detection:=true`).

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
`claro` baja de 400, y entonces para. Hay un tope de 60 tramos (~42 s) por si la
cinta no está puesta.

**Ejercicios:**
1. Baja el umbral a 200. ¿Se le pasa la línea?
2. Sube la velocidad a 0.30. ¿Qué le pasa a la distancia de parada?
3. Haz que retroceda 20 cm después de encontrar el negro.

---

## Proyecto final — Seguidor de línea (`seguidor_linea_pid_demo.py`)

🔴 **Necesita el arranque con `color_detection:=true`**, igual que las prácticas
05 y 11.

**Qué se aprende:** un PID de verdad, combinado con seguimiento de **borde** en
vez de intentar centrarse sobre la línea — la explicación completa está en
`SEGUIDOR_LINEA_EXPLICACION.md`.

**Qué hace falta en el suelo:** una pista con una línea negra sobre suelo claro.
Coloca el robot mirando en la dirección de avance, con el sensor **justo sobre el
borde derecho de la línea**: la línea (negro) a su izquierda, el suelo (claro) a
su derecha.

**Comando:**
```bash
python3 seguidor_linea_pid_demo.py
```

**Qué debería verse:** el robot avanza siguiendo el borde de la línea, corrigiendo
el giro en proporción a cuánto se aleja del centro de la banda negro/claro. Ctrl-C
para parar.

**Ejercicios** (los mismos que trae el script):
1. Pon `Kd` a 0. ¿Qué le pasa al robot en las curvas?
2. Sube `Kp` hasta que oscile. Eso es la ganancia crítica.
3. Sube `VELOCIDAD` a 0.20. ¿Sigue valiendo el mismo PID?
4. Sube `PERIODO` a 0.5. ¿Por qué va a tirones? (pista: el watchdog corta a los
   0.3 s sin recibir una orden nueva)

---

## Herramientas — `90_template.py`

No es una práctica con solución: es el punto de partida para tu propio programa.

```bash
cp 90_template.py mi_programa.py
python3 mi_programa.py
```

Trae en su cabecera la misma tabla de la API que `00_LEEME_PRIMERO.md`. Cópiala,
borra el bloque de ejemplo del medio y escribe lo tuyo ahí.

---

## Las cuatro cosas que sorprenden y no son fallos

Con estas cuatro se explican casi todos los «esto no funciona» del curso:

| Síntoma | Qué es de verdad |
|---|---|
| El robot no se mueve y no hay error | Falta `/scan`, o hay una parada de emergencia enganchada. La libera el profesor |
| Va mucho más despacio de lo pedido | El polígono de precaución frena al **40 %** si hay algo a menos de 0.36 m, **aunque el robot se aleje**: 30 cm comandados → 14 medidos |
| Los ángulos se van acumulando mal | La odometría deriva **~1 °/30 s** los primeros minutos tras encender el RVR, y 0.001 siete minutos después |
| El LIDAR no ve una caja baja | Barre a **15.5 cm del suelo**. «Despejado a ras de suelo» no basta |
