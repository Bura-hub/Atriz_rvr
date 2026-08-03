# 🚀 ¡Léeme primero!

Bienvenido al laboratorio Atriz. Esto es lo que necesitas para arrancar hoy mismo.

---

## 1. Qué es esto

Un robot Sphero RVR con un LIDAR, gobernado por la Raspberry Pi que lleva encima.
Programas en Python contra la biblioteca `atriz.py`: tú escribes *qué* tiene que
hacer el robot, y ella se encarga de todo lo demás (ROS 2, sensores, seguridad) por
debajo. No hace falta instalar nada ni saber ROS de antemano.

---

## 2. Conectarte al robot

El robot está en la red del laboratorio (`Atriz-server`).

🔴 **La contraseña de esa red y la del usuario `sphero` no van en ningún documento
de este repositorio: te las da el profesor.** Este repositorio es público.

```bash
ssh sphero@rvr-NN.local
```

Cambia `NN` por el número de tu robot (por ejemplo `rvr-03.local`).

---

## 3. Tu primer programa

Sin explicar nada todavía — eso viene en `GUIA_PASO_A_PASO.md`:

🔴 **Antes de ejecutarlo, despeja el suelo: 1.5 m por delante del robot.** El caso
base recorre 0.20 m/s × 3 s, y el ejercicio 3 de esa práctica llega a 0.40 m/s × 3 s
= **1.20 m**. Y despejado **a la altura del LIDAR (15.5 cm del suelo)**, no solo a
ras de suelo.

```bash
cd ~/atriz_ws/src/Atriz_rvr/scripts/estudiantes
python3 01_avanzar.py
```

El robot debería avanzar y parar solo (velocidad × tiempo son 60 cm; 🔴 **no está
medido con cinta** — lo mides tú en el ejercicio 1).

---

## 4. Qué hace el robot al empezar (léelo antes de asustarte)

Al construir `Robot()`, la biblioteca **enciende el barrido del LIDAR**
automáticamente. Es necesario: **sin `/scan` el robot no obedece ninguna orden de
movimiento** — la capa de seguridad (`collision_monitor`) bloquea todo mientras no
sepa qué hay alrededor.

Esto es lo primero con lo que un curso tropieza: el programa arranca, imprime
`Conectando con el robot...`, tarda unos segundos en decir `Robot listo.` (está
esperando el primer barrido de verdad, no solo que el servicio conteste), y **si
todo eso pasó y aun así el robot no se mueve, no está roto** — sigue el punto 6 de
abajo.

Al terminar tu programa (con `with`, como en el ejemplo de arriba), la biblioteca
para el robot y **apaga el barrido otra vez**: si no, el LIDAR seguiría girando a
toda velocidad las 24 horas.

---

## 5. La tabla de la API

Lo que le puedes pedir al robot — la misma que trae la cabecera de `90_template.py`:

```python
robot.avanzar(velocidad, segundos)   # m/s (máx 0.40) durante segundos
robot.girar(grados)                  # + izquierda, - derecha; devuelve los reales
robot.parar()
robot.rumbo()                        # grados
robot.distancia_frontal()            # metros hasta lo que tienes delante
robot.color()                        # (rojo, verde, azul, claro)
robot.bateria()                      # voltios
robot.luces(rojo, verde, azul)       # 0-255 cada canal
robot.parada_emergencia()            # el profesor tiene que liberarla
```

La referencia completa, con lo que hay detrás de cada límite, está en
`REFERENCIAS.md`.

---

## 6. Cuando algo no va

Antes de pensar que el robot está averiado, mira si es uno de estos cuatro casos.
Los cuatro son comportamiento normal, no fallos:

| Síntoma | Qué es de verdad |
|---|---|
| El robot no se mueve y no hay ningún error | Falta `/scan` (mira el punto 4), o hay una parada de emergencia enganchada de una ejecución anterior — solo la libera el profesor |
| Va mucho más despacio de lo que pediste | La capa de seguridad frena al **40 %** si hay algo a menos de 0.36 m por delante, **aunque el robot se esté alejando de ello**: pedir 30 cm de retroceso puede dar solo 14 cm medidos |
| Los ángulos se acumulan mal en los primeros minutos | La odometría deriva **~1 °/30 s** los primeros minutos tras encender el RVR, y baja a 0.001 °/30 s pasados **siete minutos**. Deja el robot encendido un rato antes de medir ángulos con precisión |
| El LIDAR no ve una caja o un escalón bajo | El LIDAR barre a **15.5 cm del suelo**: «despejado a ras de suelo» no es lo mismo que «despejado a la altura del LIDAR» |

---

## 7. Al terminar

Usa siempre `with Robot() as robot:` — así el robot se para y el barrido se apaga
aunque tu programa falle a mitad de camino.

Si tu programa se queda colgado, **Ctrl-C**, **una sola vez y espera**.

🔴 **No vuelvas a pulsar.** El cierre tarda unos segundos (manda velocidad cero,
llama a `/stop_scan` y suelta los recursos de ROS), y durante ese rato la
biblioteca **ignora a propósito** las pulsaciones nuevas: te lo dirá por pantalla
(`SIGINT otra vez: YA estoy cerrando, espera`). Interrumpir un cierre en marcha no
adelanta nada y antes dejaba el LIDAR girando a toda velocidad.

Cerrar la terminal o perder la conexión SSH también para el robot y apaga el
barrido: la biblioteca escucha las tres señales que puede escuchar (`SIGINT`,
`SIGTERM` y `SIGHUP`).

⚠️ **Lo único que no se puede cubrir es `kill -9` y quedarse sin corriente.** Esas
dos no se pueden capturar desde Python: el robot se para igual (el driver tiene un
watchdog que corta la velocidad a los 0.3 s sin recibir órdenes), pero **el barrido
del LIDAR se queda encendido** hasta que el profesor lo apague con
`atriz-escaneo off`. El watchdog es de los motores; del LIDAR no sabe nada.

---

## 8. 🔴 Qué de todo esto está MEDIDO, y qué no

Este laboratorio tiene una regla: *nada se documenta sin haberse ejecutado*. Se
aplica también a lo que estás leyendo, así que aquí va la parte incómoda.

**Ningún guion de esta carpeta se ha ejecutado nunca contra el robot moviéndose.**
La biblioteca está escrita, revisada y con 76 tests automáticos, pero esos tests
comprueban las **funciones puras** (aritmética de ángulos, límites, la secuencia de
apagado) sin robot. Lo que **no** está medido:

| Afirmación que verás por ahí | Estado |
|---|---|
| «avanza unos 60 cm» (0.20 m/s × 3 s) | 🔴 **NO MEDIDO con cinta.** Es velocidad × tiempo; la rampa de aceleración de ~0.5 s se come parte |
| «`girar(90)` se queda a unas décimas» | 🔴 **NO MEDIDO con transportador.** Ver la práctica 02 |
| «Ctrl-C para el robot antes de salir» | 🔴 **NO MEDIDO con el robot en movimiento.** Lo comprobado son líneas de log con el robot **quieto**. Para eso está `99_test_ctrl_c.py`: lo mides tú |
| «cuánto recorre el robot después del Ctrl-C» | 🔴 **NO MEDIDO.** Es el ejercicio 2 de `99_test_ctrl_c.py` |
| Las secciones «Qué debería verse» de `GUIA_PASO_A_PASO.md` | 🔴 Describen lo que el código **manda hacer**, no un robot que alguien haya visto hacerlo |
| El seguidor de línea | 🔴 **Nunca ha seguido una línea real.** Solo sus funciones puras tienen tests |
| Que el ángulo 0 de `/scan` sea exactamente «delante» | 🔴 **NO CONTRASTADO con cinta.** Afecta a `distancia_frontal()` |

**Esto no es una advertencia de que el robot sea peligroso** — la capa de seguridad
y los límites de velocidad sí están medidos y verificados. Es que **eres tú quien va
a tomar la primera medida**, y por eso casi todas las prácticas terminan pidiéndote
una cinta métrica o un transportador. Si tu medida no coincide con lo que dice un
documento, **anótala: es más fiable que el documento**, y díselo al profesor.

---

## Siguiente paso

`GUIA_PASO_A_PASO.md` — el recorrido completo de las prácticas, en orden.
