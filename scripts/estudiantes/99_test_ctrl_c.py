#!/usr/bin/env python3
"""Prueba 99 - ¿Para el robot cuando pulsas Ctrl-C?

    python3 99_test_ctrl_c.py

⚠️ MUEVE EL ROBOT. Necesita al menos 1.2 m despejados por delante:
avanzar(0.15, 8) son 0.15 m/s x 8 s = 1.2 m si nadie pulsa Ctrl-C. Y
despejados a la altura del LIDAR (15.5 cm desde el suelo), no solo a ras
de suelo: es lo que usa el collision_monitor para frenar, y un obstaculo
bajo que el LIDAR SI ve podria parar el robot antes de que llegues a
probar el Ctrl-C, confundiendo la prueba.

═══════════════════════════════════════════════════════════════════════════════
POR QUE ESTA PRUEBA EXISTE Y NO ES UNA CURIOSIDAD
═══════════════════════════════════════════════════════════════════════════════
En este laboratorio, Ctrl-C YA FALLO. `rclpy.init()` instala su propio manejador
de la senal e invalida su contexto: el codigo que intenta parar el robot muere
con «publisher's context is invalid». Medido: 0 lineas de parada con el defecto,
5 con la opcion correcta.

Y el fallo es INTERMITENTE — segun donde caiga el Ctrl-C, a veces si para. Por
eso esta prueba se corre VARIAS VECES: una pasada verde sobre un fallo
intermitente es indistinguible de que no haya fallo.

Debajo, el driver tiene un watchdog que corta a los 0.3 s sin recibir ordenes.
Asi que aunque mataras el programa a lo bruto, el robot se para. Ctrl-C es el
primer cinturon; el watchdog, el segundo.
"""
from atriz import Robot

print(__doc__)
with Robot() as robot:
    input('Marca la posicion del robot y pulsa Enter...')
    print('Avanzando 8 s. Pulsa Ctrl-C cuando quieras y MIDE cuanto recorre')
    print('el robot DESPUES de que lo pulses.\n')
    robot.avanzar(0.15, 8)
    print('Llegue al final sin que pulsaras Ctrl-C.')

# EJERCICIOS
#   1. Repitelo cinco veces. ¿Paro las cinco?
#   2. Mide el recorrido posterior. ¿Cuanto varia entre corridas?
#   3. Prueba a cerrar la terminal en vez de pulsar Ctrl-C. ¿Que pasa? ¿Por que?
