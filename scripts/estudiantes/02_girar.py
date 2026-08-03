#!/usr/bin/env python3
"""Practica 2 — Girar.

    python3 02_girar.py

El robot gira 90 grados a la izquierda.

Antes de ejecutarlo: deja unos 40 cm libres alrededor del robot.
"""
from atriz import Robot

with Robot() as robot:
    print('Girando 90 grados a la izquierda...')
    logrado = robot.girar(90)       # positivo = izquierda
    print(f'Giro {logrado:.1f} grados de verdad.')

# EJERCICIOS
#   1. Gira -90. ¿Hacia donde va?
#   2. Comprueba con un transportador cuanto giro. ¿Coincide con lo que imprime?
#   3. ¿Por que girar() devuelve un numero en vez de no devolver nada?
