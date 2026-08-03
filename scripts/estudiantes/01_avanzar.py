#!/usr/bin/env python3
"""Practica 1 — Avanzar.

    python3 01_avanzar.py

El robot avanza 3 segundos y para.

Antes de ejecutarlo: deja 1 metro despejado por delante del robot. El LIDAR
barre a 15.5 cm del suelo, asi que una caja baja NO la ve.
"""
from atriz import Robot

# `with` se encarga de parar el robot y apagar el barrido pase lo que pase:
# aunque tu programa falle a la mitad, o aunque pulses Ctrl-C.
with Robot() as robot:
    print('Avanzando...')
    robot.avanzar(0.20, 3)      # 0.20 metros por segundo, durante 3 segundos
    print('Listo.')

# EJERCICIOS
#   1. Cambia la velocidad a 0.30. Mide con una cinta: ¿avanzo la mitad mas?
#   2. Pon una velocidad negativa. ¿Que hace?
#   3. Pide 1.5 m/s. El robot no llega ahi: mira lo que imprime el programa.
