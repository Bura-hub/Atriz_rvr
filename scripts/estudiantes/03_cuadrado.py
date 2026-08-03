#!/usr/bin/env python3
"""Practica 3 — Un cuadrado.

    python3 03_cuadrado.py

Cuatro lados y cuatro giros de 90 grados.

Antes de ejecutarlo: necesitas un cuadrado libre de ~1.5 m de lado.
"""
from atriz import Robot

LADO_SEGUNDOS = 3      # a 0.20 m/s son unos 60 cm

with Robot() as robot:
    for lado in range(1, 5):
        print(f'Lado {lado} de 4...')
        robot.avanzar(0.20, LADO_SEGUNDOS)
        logrado = robot.girar(90)
        print(f'  esquina: {logrado:.1f} grados')

# EJERCICIOS
#   1. ¿Vuelve el robot al punto de partida? Marcalo con cinta y mide el error.
#   2. Suma los cuatro giros que imprime. ¿Cuanto se aleja de 360?
#   3. Haz un triangulo. ¿Cuantos grados hay que girar en cada esquina?
