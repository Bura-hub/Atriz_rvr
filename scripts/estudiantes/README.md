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
