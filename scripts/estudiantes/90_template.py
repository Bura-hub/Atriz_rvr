#!/usr/bin/env python3
"""Plantilla — copia este fichero para empezar tu propio programa.

    cp 90_template.py mi_programa.py
    python3 mi_programa.py

Lo que puedes pedirle al robot:

    robot.avanzar(velocidad, segundos)   # m/s (max 0.40) durante segundos
    robot.girar(grados)                  # + izquierda, - derecha; devuelve los reales
    robot.parar()
    robot.rumbo()                        # grados
    robot.distancia_frontal()            # metros hasta lo que tienes delante
    robot.color()                        # (rojo, verde, azul, claro)
    robot.bateria()                      # voltios
    robot.luces(rojo, verde, azul)       # 0-255 cada canal
    robot.parada_emergencia()            # el profesor tiene que liberarla
"""
from atriz import Robot

with Robot() as robot:

    # ── Tu programa va aqui ─────────────────────────────────────────────────
    robot.avanzar(0.20, 2)
    robot.girar(90)
    # ────────────────────────────────────────────────────────────────────────

    print('Bateria:', robot.bateria(), 'V')
