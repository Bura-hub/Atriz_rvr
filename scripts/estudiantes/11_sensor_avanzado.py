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
#
# ⚠️ El azul mide clear=396 (evidencia 37) — 4 unidades por DEBAJO de este
#    umbral. Si pruebas con cinta azul en vez de negra, el robot la clasifica
#    como negro igual: no es un fallo del programa, es que 400 separa negro de
#    SUELO, no negro de cualquier otro color. No se sube el umbral sin una
#    medida nueva: la practica pide cinta negra a proposito.
UMBRAL_NEGRO = 400

# Tope de seguridad: el bucle de abajo no tiene forma de saber si la cinta esta
# puesta, torcida, o si el sensor esta mal alineado, y sin tope seguiria
# avanzando en silencio. La cuenta, con lo medido en la Tarea 9:
#   - cada tramo avanza ~2 cm nominales (10 cm/s durante 0.2 s) y tarda ~0.7 s
#     de verdad (0.2 s moviendo + 0.5 s de parar() insistiendo en velocidad
#     cero) — medido leyendo avanzar()/parar() en atriz.py.
#   - el collision_monitor frena solo a ~9.9-10.7 cm de un obstaculo (medido a
#     0.25 y 0.40 m/s, 2026-07-31); con el metro despejado que pide esta
#     practica, eso deja ~90 cm de recorrido util ANTES de que el robot quede
#     pegado a la pared. 🔴 A 0.10 m/s (la velocidad de esta practica) esa
#     frenada NO esta medida: es una extrapolacion desde 0.25/0.40 m/s, no una
#     medida nueva.
#   - 90 cm / 2 cm por tramo = 45 tramos para agotar el espacio despejado. Se
#     deja margen (lecturas que no caen justo en el borde de la cinta, tramos
#     mas cortos de lo nominal) y se sube a 60 tramos (~42 s) — sin ese margen,
#     un tope demasiado ajustado cortaria el intento incluso con la cinta bien
#     puesta.
MAX_TRAMOS = 60

with Robot() as robot:
    if not robot.hay_color:
        print('\nEl sensor de color esta apagado. Lee la practica 5.')
        sys.exit(1)

    print(f'Avanzando hasta encontrar negro (maximo {MAX_TRAMOS} tramos, '
          f'~{MAX_TRAMOS * 0.7:.0f} s)...')
    # No se usa avanzar(), que bloquea los segundos que le pidas: aqui hay que
    # mirar el sensor MIENTRAS se avanza.
    tramos = 0
    while True:
        _, _, _, claro = robot.color()
        if claro < UMBRAL_NEGRO:
            print(f'Negro detectado (claro={claro}) tras {tramos} tramos. Parando.')
            robot.parar()
            break
        if tramos >= MAX_TRAMOS:
            robot.parar()
            print(f'\nAVISO: {MAX_TRAMOS} tramos sin encontrar negro (ultima '
                  f'lectura: claro={claro}, umbral={UMBRAL_NEGRO}). Parando.\n'
                  f'       Revisa, en este orden:\n'
                  f'         1. ¿esta puesta la cinta negra y cruza el camino '
                  f'del robot?\n'
                  f'         2. ¿el robot arranco con color_detection:=true? '
                  f'(si no, claro nunca baja de verdad)\n'
                  f'         3. ¿el sensor mira al suelo, o el robot quedo '
                  f'atascado contra algo antes de llegar?')
            sys.exit(1)
        if tramos % 10 == 0:
            print(f'  tramo {tramos}: claro={claro} (sigue buscando negro)')
        robot.avanzar(0.10, 0.2)     # tramos cortos: 10 cm/s durante 0.2 s
        tramos += 1

# EJERCICIOS
#   1. Baja el umbral a 200. ¿Se le pasa la linea?
#   2. Sube la velocidad a 0.30. ¿Que le pasa a la distancia de parada?
#   3. Haz que retroceda 20 cm despues de encontrar el negro.
