#!/usr/bin/env python3
"""Practica 11 — Reaccionar a lo que ve: parar sobre negro.

    python3 11_sensor_avanzado.py

🔴 NECESITA EL ARRANQUE CON color_detection:=true. Lee la cabecera de la
   practica 5.

Antes de ejecutarlo: 1 metro despejado por delante, y una franja de cinta negra
cruzando el camino del robot.
"""
import sys

from atriz import Robot

# El canal claro va de ~181 sobre negro a ~2288 sobre blanco (evidencia 37). El
# umbral NO esta a la mitad de ese recorrido (seria ~1235): el suelo real, sin
# cinta encima, ya da clear=1275 — casi tan alto como el blanco. Un umbral a
# mitad de camino confundiria «suelo normal» con «casi blanco» y no serviria
# para nada. UMBRAL_NEGRO=400 se queda muy por debajo del suelo (1275) y bien
# por encima del negro (181): esa es la separacion que hace falta.
UMBRAL_NEGRO = 400

with Robot() as robot:
    if not robot.hay_color:
        print('\nEl sensor de color esta apagado. Lee la practica 5.')
        sys.exit(1)

    print('Avanzando hasta encontrar negro...')
    # No se usa avanzar(), que bloquea los segundos que le pidas: aqui hay que
    # mirar el sensor MIENTRAS se avanza.
    while True:
        _, _, _, claro = robot.color()
        if claro < UMBRAL_NEGRO:
            print(f'Negro detectado (claro={claro}). Parando.')
            robot.parar()
            break
        robot.avanzar(0.10, 0.2)     # tramos cortos: 10 cm/s durante 0.2 s

# EJERCICIOS
#   1. Baja el umbral a 200. ¿Se le pasa la linea?
#   2. Sube la velocidad a 0.30. ¿Que le pasa a la distancia de parada?
#   3. Haz que retroceda 20 cm despues de encontrar el negro.
