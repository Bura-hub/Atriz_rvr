#!/usr/bin/env python3
"""Practica 1 — Avanzar.

    python3 01_avanzar.py

El robot avanza 3 segundos y para.

🔴 ANTES DE EJECUTARLO, HAZ LA CUENTA DEL ESPACIO:
      caso base    0.20 m/s x 3 s = 0.60 m
      ejercicio 1  0.30 m/s x 3 s = 0.90 m
      ejercicio 3  se recorta a 0.40 m/s x 3 s = 1.20 m   <- el peor caso
   => deja 1.5 m DESPEJADOS POR DELANTE, no 1 m.
   Y el ejercicio 2 va hacia ATRAS: deja tambien ~1 m DETRAS del robot.

El LIDAR barre a 15.5 cm del suelo, asi que una caja baja NO la ve — ni este
programa ni la capa de seguridad. «Despejado a ras de suelo» no basta.

⚠️ Y hacia atras la capa de seguridad NO te protege: su poligono mira hacia
   DELANTE. Detras del robot no hay nada vigilando.

🔴🔴 `avanzar(0.20, 3)` NO SIGNIFICA 60 cm. SIGNIFICA «3 SEGUNDOS».
   Si hay algo cerca, la capa de seguridad frena el robot al 40 % y recorreras
   MUCHO MENOS, sin ningun mensaje. Medido el 2026-08-08 con esta misma
   practica, dos pasadas seguidas sin tocar nada:

       26.4 cm   <- con algo cerca
       59.5 cm   <- despejado

   El poligono de precaucion es de 60 cm de largo x **40 de ANCHO**, centrado en
   el robot. Como el robot mide 21.7 de ancho, **cualquier cosa a menos de ~9 cm
   de un COSTADO** lo frena: una pata de silla, un zocalo, tu propio pie.

   📌 Si vas a medir con cinta -- y el ejercicio 1 te lo pide -- **despeja
      tambien los lados**, no solo el frente. Si no, mediras la capa de
      seguridad en vez de tu programa.
   📝 No es un fallo: es el `collision_monitor` haciendo su trabajo. Pero
      conviene saber que existe antes de creer que tu cuenta esta mal.
      Evidencia 85.
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
#      (recorrido: 0.90 m — cabe en el 1.5 m que pide la cabecera)
#   2. Pon una velocidad negativa. ¿Que hace?
#      🔴 EL ROBOT VA HACIA ATRAS 60 cm. Comprueba que hay ~1 m DETRAS de el
#         antes de ejecutarlo, y mira hacia donde vas a estar tu.
#   3. Pide 1.5 m/s. El robot no llega ahi: mira lo que imprime el programa.
#      Se recorta a 0.40 m/s (VEL_MAX), asi que recorre 1.20 m, el DOBLE que
#      el caso base. Esa es la razon de pedir 1.5 m de espacio y no 1 m.
