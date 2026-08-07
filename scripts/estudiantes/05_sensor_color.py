#!/usr/bin/env python3
"""Practica 5 — El sensor de color.

    python3 05_sensor_color.py

El sensor de color lleva su PROPIA LUZ debajo del robot, y sin ella no ve nada:
el canal claro pasa de ~1320 con la luz encendida a 1 apagada. Mas de mil veces.

Por eso este programa hace lo primero de todo:

    robot.sensor_color(True)

El robot arranca con esa luz APAGADA a proposito —es un LED blanco encendido
bajo el chasis, y gasta bateria—, asi que la enciendes tu cuando vas a medir.
`cerrar()` la apaga sola al terminar.

📝 Hasta el 2026-08-06 esta practica pedia que el profesor reiniciase el robot
   con un arranque especial. Resulto que no hacia falta: la luz se puede
   encender en caliente, y siempre se pudo.
"""
import time

from atriz import Robot

with Robot() as robot:
    # Sin esto, todas las lecturas serian oscuridad — y la oscuridad se parece
    # muchisimo a «negro», que es justo lo que esta practica quiere distinguir.
    robot.sensor_color(True)
    print('Luz del sensor ENCENDIDA (miralo: hay un LED blanco bajo el robot).')

    print('Pon distintas superficies bajo el robot. Ctrl-C para salir.\n')
    print(f'{"rojo":>6} {"verde":>6} {"azul":>6} {"claro":>6}   R/G    B/G')
    while True:
        rojo, verde, azul, claro = robot.color()
        # Se normaliza por VERDE porque es el canal mas sensible: sobre rojo,
        # R/G sube de 0.48 a 2.74; sobre azul, B/G sube a 0.86.
        rg = rojo / verde if verde else 0.0
        bg = azul / verde if verde else 0.0
        print(f'{rojo:6d} {verde:6d} {azul:6d} {claro:6d}  {rg:5.2f}  {bg:5.2f}')
        time.sleep(0.5)

# EJERCICIOS
#   1. Prueba blanco, negro, rojo y azul. ¿Que columna cambia mas?
#   2. `claro` va de ~181 sobre negro a ~2288 sobre blanco. ¿Y R/G?
#   3. ¿Por que dividimos por verde en vez de usar el rojo a secas?
