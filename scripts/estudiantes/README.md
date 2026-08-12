# 🤖 Scripts para estudiantes — Sphero RVR

Todo lo que necesitas para el curso de 16 horas vive en esta carpeta.

## Por dónde empezar

👉 **[`00_LEEME_PRIMERO.md`](00_LEEME_PRIMERO.md)** — ábrelo primero, siempre.

## Índice de la carpeta

| Archivo | Qué es |
|---|---|
| **[00_LEEME_PRIMERO.md](00_LEEME_PRIMERO.md)** | Empieza aquí: conexión, tu primer programa, y los cuatro síntomas que no son fallos |
| **[GUIA_PASO_A_PASO.md](GUIA_PASO_A_PASO.md)** | El recorrido completo, práctica por práctica |
| **[REFERENCIAS.md](REFERENCIAS.md)** | La referencia de la API: cada método, sus límites y el porqué |
| **[SEGUIDOR_LINEA_EXPLICACION.md](SEGUIDOR_LINEA_EXPLICACION.md)** | El PID y el seguimiento de borde del proyecto final, explicados |
| `atriz.py` | La biblioteca del laboratorio; no hace falta instalar nada |
| `01_avanzar.py` … `11_sensor_avanzado.py` | Las prácticas, en el orden de `GUIA_PASO_A_PASO.md` |
| `20_identificarse.py` … `24_dispersion.py` | 🆕 **Prácticas de infrarrojos. NECESITAN DOS ROBOTS** — ver abajo |
| `seguidor_linea_pid_demo.py` | El proyecto final: PID + seguimiento de borde |
| `seguidor_config.json` | Los parámetros del seguidor de línea (velocidad, umbrales, PID) |
| `90_template.py` | Plantilla para tu propio programa: cópiala y edítala |
| `99_test_ctrl_c.py` | Comprueba que Ctrl-C para el robot de verdad, varias veces seguidas |

## Inicio rápido

🔴 **`01_avanzar.py` MUEVE EL ROBOT.** Antes de lanzarlo, deja **1.5 m despejados
por delante** (el caso base recorre 60 cm; el ejercicio 3 de esa práctica llega a
1.20 m), y despejados **a la altura del LIDAR: 15.5 cm del suelo**, no solo a ras
de suelo — una caja baja no la ve ni el programa ni la capa de seguridad.

```bash
cd ~/atriz_ws/src/Atriz_rvr/scripts/estudiantes
python3 01_avanzar.py
```

Si algo no responde como esperas, mira primero el punto 6 de
`00_LEEME_PRIMERO.md`: casi siempre es uno de los cuatro comportamientos
conocidos, no un robot averiado.

🔴 Y lee el **punto 8** de `00_LEEME_PRIMERO.md` antes de fiarte de un número de
esta documentación: **ningún guion de esta carpeta se ha ejecutado todavía contra
el robot moviéndose**. Las distancias y los ángulos que verás escritos son
aritmética, no medidas — las medidas las vas a tomar tú.


---

## 🆕 Las prácticas de infrarrojos (20-24) — necesitan DOS robots

El IR del RVR es **luz entre robots**: no usa la red, así que funciona aunque el WiFi se caiga.

| | qué hace | ¿mueve el robot? |
|---|---|---|
| `20_identificarse.py` | cada robot emite su código y oye los de los demás | no |
| `21_mensajeria.py` | un vocabulario de ocho mensajes entre dos robots | no |
| `22_marco_polo.py` | uno emite, el otro adivina por dónde le llega | no |
| `23_tren_de_robots.py` | uno hace de locomotora y otro le sigue | 🔴 **sí, solo** |
| `24_dispersion.py` | todos huyen unos de otros y se separan | 🔴 **sí, solo** |

### 🔴 Lo que hay que saber antes de las dos últimas

`23` y `24` activan modos del **firmware** del RVR: el robot conduce solo, **sin pasar por
`/cmd_vel`**. Y eso significa que **el `collision_monitor` no lo ve y el watchdog tampoco**.

Lo único que los para es la parada de emergencia y `parar_ir()`. La biblioteca los apaga sola al
cerrar, incluso con Ctrl-C — pero eso **no sustituye a mirar**. 👤 Espacio despejado, suelo continuo
sin escalones, y no dejar la sesión sola.

### Los límites del hardware, medidos el 2026-08-11 con dos robots

No son avisos de precaución: son resultados de medir (evidencias 99 y 100).

- **Sólo ocho códigos** (0-7) y **dieciséis robots**: dos del aula tendrán por fuerza el mismo, y
  entonces son indistinguibles.
- **Un mensaje es un número.** No lleva datos dentro.
- **La detección es intermitente.** Una sola lectura puede decir «no hay nadie» habiéndolo. Todas
  estas prácticas muestrean varias veces por eso.
- **La dirección tiene tres estados, no cuatro:** izquierda, detrás, y «delante o derecha» sin
  separarlos. 🔴 La documentación de Sphero dice que son cuatro esquinas, pero **esa documentación
  es del BOLT**, que es otro robot — medirlo fue lo que lo descubrió.
- **El infrarrojo rebota** en paredes y suelo. «Lo tengo a la izquierda» puede ser un robot a la
  derecha reflejado.
- ⏳ **Con más de dos robots no está medido.** Si lo pruebas con ocho, anota lo que pase: es
  información que el proyecto no tiene.
