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

```bash
cd ~/atriz_ws/src/Atriz_rvr/scripts/estudiantes
python3 01_avanzar.py
```

Si algo no responde como esperas, mira primero el punto 6 de
`00_LEEME_PRIMERO.md`: casi siempre es uno de los cuatro comportamientos
conocidos, no un robot averiado.
