# Seguidor de línea con un solo sensor de color

Este documento explica el diseño, el cálculo del giro, y la recuperación del
seguidor de línea implementado en:

- `scripts/estudiantes/seguidor_linea_pid_demo.py`

Es un PID combinado con seguimiento de **borde** (edge-following), pensado para
un robot que solo tiene **un** sensor de color mirando hacia abajo.

---

## 1) Arquitectura

El script **no habla ROS directamente**: usa la biblioteca `atriz.py`, igual que
el resto de las prácticas. Todo lo que necesita son dos llamadas:

```python
from atriz import Robot

with Robot() as robot:
    _, _, _, claro = robot.color()   # lee el sensor
    robot.mover(velocidad, giro)     # manda UNA orden y no bloquea
```

`robot.mover()` no republica: hay que llamarla en el propio bucle, más de tres
veces por segundo (el watchdog del driver corta a los 0.3 s sin una orden nueva).
El script lo hace con un `sleep` calculado para mantener 10 Hz — ver la sección 7.

🔴 **Necesita que el robot haya arrancado con `color_detection:=true`** (lo hace
el profesor). Sin eso, `robot.hay_color` es `False` y el script avisa y sale antes
de moverse.

---

## 2) Señal de entrada

La señal es el canal **`claro`** de `robot.color()`, directamente, en su escala
nativa — **no** una media `(R+G+B)/3` sobre 0-255. Dos medidas lo anclan
(evidencia 37, `00_auditoria/evidencia_24_04/37_sensores_opticos.txt`):

| Superficie | `claro` |
|---|---|
| Negro (la línea) | 181 |
| Suelo real del laboratorio (sin cinta) | 1275 |

De ahí salen los dos umbrales del seguidor:

```python
UMBRAL_NEGRO = 400    # claro <= esto: sobre la línea
UMBRAL_CLARO = 1000   # claro >= esto: sobre el suelo
```

`400` y `1000` **no son una tercera medida**: son un margen razonado desde esos
dos anclajes — `400` deja más del doble de holgura sobre el `181` medido, y
`1000` deja un 22 % por debajo del `1275` medido. No hay en la evidencia lecturas
repetidas del mismo punto que permitan calcular el ruido real del sensor, así que
el margen exacto (por qué 400 y no 350 o 450) queda **NO VERIFICADO** — los dos
anclajes sí lo están.

⚠️ **Limitación conocida:** la misma evidencia trae `azul = 396` (una superficie
azul, no la línea) — por **debajo** de `UMBRAL_NEGRO = 400`. Con este umbral, un
objeto azul bajo el sensor se clasifica igual que la línea negra: este seguidor
**no distingue el azul del negro**. No se cambia el umbral sin una medida nueva.

---

## 3) Por qué borde y no centro

Un solo sensor mirando hacia abajo **no puede saber hacia qué lado se ha
desviado** el robot: el sensor solo entrega un número (`claro`), y ese número
sale igual de alto tanto si el robot deriva a la izquierda de la línea como si
deriva a la derecha — en los dos casos deja de ver negro y empieza a ver suelo
claro.

Por eso este seguidor **no intenta centrarse** sobre la línea. En su lugar sigue
siempre **el mismo borde**, como quien camina apoyando la mano en una pared: el
robot arranca con el sensor justo sobre el límite negro/claro (línea a un lado,
suelo al otro) y corrige para mantenerse ahí.

---

## 4) Cómo se decide el giro: magnitud y signo, por separado

El **PID** (la clase `PID`, sin cambios respecto a lo que se estudia en teoría)
decide **cuánto** corregir. **Nunca** decide hacia qué lado: eso lo fija un
estado que se arrastra entre vueltas del bucle, `lado_borde` (+1 o -1), no una
lectura instantánea del sensor.

Cuatro funciones puras hacen el trabajo (fuera de la clase `PID`, así que se
pueden probar sin conectar a ningún robot):

```python
clasificar(claro, ...)              # 'negro' / 'borde' / 'claro'
signo_correccion(claro, lado_borde, ...)   # +1 / -1 — NUNCA sale del PID
magnitud_correccion(claro, pid, ...)       # el PID, siempre >= 0
decidir_giro(claro, lado_borde, pid, ...)  # signo * magnitud
```

**El signo y la magnitud miden desde el mismo punto**: el centro entre
`UMBRAL_NEGRO` y `UMBRAL_CLARO` (700 con los valores por defecto). Eso es a
propósito y es importante: si signo y magnitud usaran fronteras distintas, hay
una banda intermedia donde el signo dice un lado y la magnitud ya está creciendo
hacia el otro — realimentación positiva, el robot se aleja del borde en vez de
volver. Con las dos midiendo desde el centro, el giro es continuo y cambia de
signo exactamente donde la magnitud pasa por cero.

`clasificar()` **no decide el signo del giro**. Sirve para otra cosa: saber
cuándo el robot lleva demasiado tiempo sin ver el borde (sección siguiente).

⚠️ El margen de `clasificar()` (`MARGEN_HISTERESIS`) **no es histéresis de
verdad**: la función no tiene estado, no recuerda la clasificación anterior.
Solo ensancha la zona `'borde'` hacia dentro de los dos umbrales — da un colchón
antes de declarar `'negro'` o `'claro'` en firme, pero una lectura oscilando
justo en el borde de esa zona seguiría cambiando de clasificación en cada
muestra, igual que con un umbral simple.

---

## 5) Recuperación al perder el borde

Este es el punto en el que este documento **ya no describe lo mismo que hacía la
versión anterior del script**: la recuperación se simplificó.

Mientras el sensor sigue viendo `'negro'` o `'borde'`, el giro normal (sección 4)
ya corrige lo bastante: cuanto más lejos del centro, más fuerte corrige el PID,
en las tres clasificaciones por igual. No hace falta una maniobra aparte para
«nos pasamos un poco hacia el suelo».

Solo hay una recuperación explícita, y es para el caso de estar **realmente
perdidos**: si el sensor lleva más de `tiempo_perdido_max` segundos seguidos
viendo `'claro'` (por defecto, 1.0 s), se asume que la hipótesis de hacia dónde
está la línea era la equivocada, y se **invierte `lado_borde`** para probar la
contraria. El robot sigue avanzando hacia delante mientras tanto, corrigiendo
cada vez más fuerte — no hay una fase de retroceso ni de escaneo en el sitio.

No se reprodujo tampoco ningún filtrado por buffer sobre las lecturas: el canal
`claro` no se ha caracterizado por ruido (mismo hueco de evidencia que el margen
de la sección 2), así que añadir un filtro sin saber si hace falta habría sido
inventar una calibración.

**Esto es una simplificación deliberada, y no está verificada sobre el robot**
más allá de las pruebas automáticas de las funciones puras. Si al moverlo el
robot se pierde de forma persistente, o `tiempo_perdido_max` resulta
insuficiente, es el primer sitio donde mirar.

---

## 6) Parámetros

Viven en `seguidor_config.json`, junto al script:

```json
{
  "velocidad": 0.08,
  "umbral_negro": 400,
  "umbral_claro": 1000,
  "margen_histeresis": 50,
  "lado_borde": 1,
  "tiempo_perdido_max": 1.0,
  "pid": {
    "kp": 0.5,
    "ki": 0.0,
    "kd": 0.3,
    "limite": 1.5
  }
}
```

| Parámetro | Qué hace |
|---|---|
| `velocidad` | m/s hacia delante, constante mientras el PID corrige el giro |
| `umbral_negro`, `umbral_claro` | los dos anclajes de la sección 2 |
| `margen_histeresis` | ensancha la zona `'borde'` de `clasificar()` — ver la sección 4 |
| `lado_borde` | convención de arranque: `+1` = línea a la izquierda del robot, suelo a la derecha |
| `tiempo_perdido_max` | segundos en `'claro'` antes de invertir `lado_borde` |
| `pid.kp`, `pid.ki`, `pid.kd` | las tres ganancias del PID de siempre |
| `pid.limite` | tope de saturación de la salida del PID (rad/s) |

Si el fichero no existe, el script usa estos mismos valores por defecto.

---

## 7) El ritmo del bucle

```python
PERIODO = 0.1   # s -> 10 Hz
```

Cada vuelta hace: `robot.color()` (13-21 ms medidos), el PID (aritmética, nada) y
`robot.mover()` (una publicación, nada). A 10 Hz, `color()` consume como mucho el
21 % del período — cabe de sobra.

⚠️ Si `PERIODO` supera 0.3 s, el hueco entre dos llamadas a `mover()` es mayor que
el timeout del watchdog del driver: el robot arranca al recibir la orden, el
watchdog lo para a los 0.3 s, y queda quieto el resto del período. Eso es «ir a
tirones» — es el ejercicio 4 del script.

---

## 8) Flujo del bucle, tal como está hoy

```
mientras True:
    leer claro con robot.color()
    clasificar(claro)                        -> 'negro' / 'borde' / 'claro'
    si 'claro' sostenido > tiempo_perdido_max:
        invertir lado_borde
    giro = decidir_giro(claro, lado_borde, pid)
    robot.mover(velocidad, giro)
    dormir lo que falte del período
```

No hay una fase de recuperación con retroceso ni escaneo en el sitio: la única
rama especial es el contador de tiempo perdido.

---

## 9) Errores comunes

- **«Se sale en las curvas»**: baja `velocidad`, o sube el límite del PID
  (`pid.limite`) para que pueda corregir más fuerte.
- **«Oscila demasiado»**: baja `pid.kp`, o revisa si `pid.kd` está en 0
  (ejercicio 1 del script).
- **«No reengancha tras perderse»**: baja `tiempo_perdido_max`, o revisa que
  `lado_borde` inicial sea el que corresponde a cómo colocaste el robot.
- **El robot confunde un objeto azul con la línea**: es una limitación conocida
  (sección 2), no un fallo del script.

---

## 10) Referencias prácticas

- Script: `scripts/estudiantes/seguidor_linea_pid_demo.py`
- Parámetros: `scripts/estudiantes/seguidor_config.json`
- Lo que usa de la biblioteca del laboratorio: `robot.color()` y
  `robot.mover()` — ver `REFERENCIAS.md` para su firma completa.
- La evidencia de los umbrales: `00_auditoria/evidencia_24_04/37_sensores_opticos.txt`
  (repositorio `atriz_migracion`, privado — citado por nombre, no enlazado).
