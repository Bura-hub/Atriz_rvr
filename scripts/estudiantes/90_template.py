#!/usr/bin/env python3
"""Plantilla — copia este fichero para empezar tu propio programa.

    cp 90_template.py mi_programa.py
    python3 mi_programa.py

🔴 TAL COMO ESTA, ESTE FICHERO MUEVE EL ROBOT: avanza 0.20 m/s x 2 s = 40 cm y
   gira 90 grados sobre el eje. Antes de ejecutarlo deja al menos 1 m despejado
   por delante y 40 cm libres alrededor, a la altura del LIDAR (15.5 cm del
   suelo), no solo a ras de suelo.

🔴 Y CUANDO ESCRIBAS LO TUYO, VUELVE A HACER LA CUENTA. El espacio que hace
   falta sale de tu codigo, no de esta cabecera: suma los `avanzar()` que
   encadenes (metros = velocidad x segundos) y acuerdate de que `girar()`
   necesita sitio a los lados. Si mandas velocidades negativas, cuenta tambien
   el espacio DETRAS — ahi la capa de seguridad no vigila, porque su poligono
   mira hacia delante.

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
