#!/usr/bin/env python3
"""Practica 5 — El sensor de color.

    python3 05_sensor_color.py

🔴 ESTA PRACTICA NECESITA UN ARRANQUE ESPECIAL DEL ROBOT. Lo hace el profesor:

    sudo systemctl stop atriz-robot
    ros2 launch atriz_rvr_bringup robot.launch.py color_detection:=true

Por que: el sensor de color lleva su PROPIA LUZ debajo del robot, y sin ella no
ve nada — el canal claro pasa de 741 encendida a 4 apagada, 185 veces menos. Esa
luz se enciende ANTES de configurar el sensor y NO se puede encender despues, asi
que se decide en el arranque. El arranque normal la deja apagada a proposito,
porque es un LED blanco encendido todo el rato bajo el chasis.

Si arrancas normal, este programa te lo dira y saldra: no te devolvera ceros
haciendolos pasar por «negro».
"""
import sys
import time

from atriz import Robot

with Robot() as robot:
    if not robot.hay_color:
        print('\nEl sensor de color esta apagado. Lee la cabecera de este fichero.')
        sys.exit(1)

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
