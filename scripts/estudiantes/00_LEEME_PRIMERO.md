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

```bash
cd ~/atriz_ws/src/Atriz_rvr/scripts/estudiantes
python3 01_avanzar.py
```

El robot debería avanzar unos 60 cm y parar solo. Deja al menos 1 metro libre por
delante antes de ejecutarlo.

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
| Los ángulos se acumulan mal en la primera media hora | La odometría deriva **~1 °/30 s** los primeros minutos tras encender el RVR, y baja a 0.001 °/30 s pasados siete minutos. Deja el robot encendido un rato antes de medir ángulos con precisión |
| El LIDAR no ve una caja o un escalón bajo | El LIDAR barre a **15.5 cm del suelo**: «despejado a ras de suelo» no es lo mismo que «despejado a la altura del LIDAR» |

---

## 7. Al terminar

Usa siempre `with Robot() as robot:` — así el robot se para y el barrido se apaga
aunque tu programa falle a mitad de camino.

Si tu programa se queda colgado, **Ctrl-C**. Está probado que para el robot antes
de salir (no siempre a la primera pulsación: si no reacciona, espera un segundo y
vuelve a pulsar).

---

## Siguiente paso

`GUIA_PASO_A_PASO.md` — el recorrido completo de las prácticas, en orden.
