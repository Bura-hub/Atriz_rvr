#!/usr/bin/env python3
"""Practica 3 — Un cuadrado.

    python3 03_cuadrado.py

Cuatro lados y cuatro giros de 90 grados.

Antes de ejecutarlo: necesitas un cuadrado libre de ~1.5 m de lado.
"""
from atriz import Robot

# 0.20 m/s x 3 s = 0.60 m de recorrido COMANDADO. El lado real sera algo mas
# corto: el robot tarda ~0.5 s en llegar a la velocidad pedida (rampa de
# aceleracion medida el 2026-07-31), y ese tramo cuenta como tiempo pero no
# como 20 cm/s. 🔴 CUANTO mas corto NO ESTA MEDIDO: no se cambia un numero sin
# medir por otro numero sin medir. Lo mides tu en el ejercicio 1.
LADO_SEGUNDOS = 3

with Robot() as robot:
    for lado in range(1, 5):
        print(f'Lado {lado} de 4...')
        robot.avanzar(0.20, LADO_SEGUNDOS)
        logrado = robot.girar(90)
        print(f'  esquina: {logrado:.1f} grados')

# EJERCICIOS
#   1. ¿Vuelve el robot al punto de partida? Marcalo con cinta y mide el error.
#      Mide tambien UN LADO: son 0.60 m comandados, pero la rampa de ~0.5 s se
#      come parte. Lo que salga es la primera medida que existe de esto.
#   2. Suma los cuatro giros que imprime. ¿Cuanto se aleja de 360?
#   3. Haz un triangulo. ¿Cuantos grados hay que girar en cada esquina?
