#!/usr/bin/env python3
"""Practica 10 - Tu propia clase: un robot que patrulla.

    python3 10_movimiento_completo.py

Antes de ejecutarlo: al menos 3 m despejados en la direccion en que mire el
robot al arrancar. La cuenta: 12 tramos de avanzar(0.20, 1) sin girar nunca
son 12 x 0.20 m = 2.4 m en linea recta, y eso pasa si el robot nunca detecta
nada a menos de 0.35 m por delante -- por ejemplo, si mira hacia un pasillo o
hacia la diagonal del recinto en vez de a una pared.

🔴 Y 2.4 m de recorrido NO significa «2.5 m de sitio». Los 10 cm que sobrarian
   son MENOS que el propio robot, que mide 19 cm de largo: la cuenta hay que
   hacerla desde el MORRO, no desde el centro. Con 3 m quedan ~40 cm de margen
   de verdad, que es lo que hace falta para el chasis mas la frenada del
   collision_monitor (~10 cm medidos a 0.25 m/s).

La idea: `Robot` te da las ordenes basicas. Aqui construyes ENCIMA una clase
con el comportamiento que tu quieres. Es como se organiza el codigo de un
robot de verdad: capas, cada una apoyada en la de abajo.
"""
from atriz import Robot


class Patrulla:
    """Recorre un recinto y avisa cuando se acerca demasiado a algo."""

    def __init__(self, robot, distancia_minima=0.35):
        self.robot = robot
        self.distancia_minima = distancia_minima
        self.giros = 0
        self.distancia_actual = None

    def hay_sitio(self):
        """¿Cabe seguir avanzando?

        Guarda la lectura en `self.distancia_actual`: `patrullar()` la reusa
        para el print, en vez de volver a preguntarle al sensor. Con dos
        lecturas separadas el print podria no coincidir con la distancia que
        de verdad decidio el tramo -- aqui solo hay una.
        """
        self.distancia_actual = self.robot.distancia_frontal()
        return self.distancia_actual > self.distancia_minima

    def un_tramo(self):
        """Avanza mientras haya sitio; si no, gira."""
        if self.hay_sitio():
            self.robot.avanzar(0.20, 1)
        else:
            print(f'  algo a menos de {self.distancia_minima} m: giro')
            self.robot.girar(90)
            self.giros += 1

    def patrullar(self, tramos):
        for numero in range(1, tramos + 1):
            self.un_tramo()
            print(f'Tramo {numero} de {tramos}  '
                  f'(frontal: {self.distancia_actual:.2f} m)')
        print(f'Fin. Giro {self.giros} veces.')


with Robot() as robot:
    Patrulla(robot).patrullar(tramos=12)

# EJERCICIOS
#   1. Cambia `distancia_minima` a 0.60. ¿Gira antes o despues?
#   2. Haz que gire -90 en vez de 90. ¿Cambia el recorrido?
#   3. Anade un metodo que encienda las luces en rojo cuando vaya a girar.
#   4. ¿Por que `un_tramo` avanza solo 1 segundo y no 10?
