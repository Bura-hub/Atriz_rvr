#!/usr/bin/env python3
"""Practica 4 — Girar bien: lazo abierto contra lazo cerrado.

    python3 04_giro_preciso.py

Antes de ejecutarlo: ~40 cm libres alrededor, y un transportador o una cinta
para marcar el rumbo.

⚠️ Y deja el robot encendido unos 10 minutos antes de medir. La odometria deriva
   ~1 grado cada 30 s los primeros minutos tras encender el RVR, y 0.001 siete
   minutos despues: midiendo en frio no sabrias si el error es del lazo o de eso.

═══════════════════════════════════════════════════════════════════════════════
LA IDEA
═══════════════════════════════════════════════════════════════════════════════
Girar «durante el tiempo justo» es un LAZO ABIERTO: mandas la orden y confias.
En este robot, pidiendo 90 grados asi salen 86.6 / 86.2 / 87.7 — un deficit de
unos 3 grados que NO depende de la bateria (se midio del 55 % al 100 %).

La salida barata seria multiplicar por 1.04. Funcionaria hoy, en este suelo, con
este robot y esta bateria. `robot.girar()` hace otra cosa: MIDE el rumbo
mientras gira y para cuando llega. Eso es un LAZO CERRADO.

Este programa hace los dos y te deja comparar.
"""
import math
import time

from atriz import Robot

OBJETIVO = 90.0
VELOCIDAD_GIRO = 0.8        # rad/s

with Robot() as robot:

    # ── Lazo abierto ────────────────────────────────────────────────────────
    # Cuanto «deberia» tardar: el angulo en radianes partido por la velocidad.
    segundos = math.radians(OBJETIVO) / VELOCIDAD_GIRO

    input('\n[1/2] LAZO ABIERTO. Marca el rumbo actual y pulsa Enter...')
    antes = robot.rumbo()
    robot.girar_por_tiempo(VELOCIDAD_GIRO, segundos)
    time.sleep(0.5)                  # el robot sigue rodando un poco
    # 📝 Resta rumbos ABSOLUTOS a proposito, no un descuido: es correcta para
    #    90 grados, pero para OBJETIVO=360 el rumbo vuelve al punto de partida
    #    y esto da ~0 — eso es justo lo que destapa el ejercicio 5 de abajo.
    #    La forma que no tiene este problema es acumular el INCREMENTO de
    #    rumbo (mira `acumular()` en atriz.py), no restar dos absolutos.
    logrado_abierto = robot.rumbo() - antes
    print(f'      pedido {OBJETIVO:.0f}, /odom dice {logrado_abierto:.1f}')
    input('      Mide con el transportador y pulsa Enter...')

    # ── Lazo cerrado ────────────────────────────────────────────────────────
    input('\n[2/2] LAZO CERRADO. Marca el rumbo actual y pulsa Enter...')
    logrado_cerrado = robot.girar(OBJETIVO)
    print(f'      pedido {OBJETIVO:.0f}, logrado {logrado_cerrado:.1f}')
    input('      Mide con el transportador y pulsa Enter...')

    print(f'\nError del lazo abierto: {abs(OBJETIVO - logrado_abierto):.1f} grados')
    print(f'Error del lazo cerrado: {abs(OBJETIVO - logrado_cerrado):.1f} grados')

# EJERCICIOS
#   1. ¿Cual de los dos se acerco mas? ¿Cuanto?
#   2. Repite los dos tres veces. ¿Cual REPITE mejor? (no es lo mismo que acertar)
#   3. Pon el robot sobre una alfombra y repite. ¿Cual aguanta el cambio?
#   4. ¿Que necesita el lazo cerrado que el abierto no? (pista: un sensor)
#   5. Cambia OBJETIVO a 360. Ojo: ¿por que no sale 0?
