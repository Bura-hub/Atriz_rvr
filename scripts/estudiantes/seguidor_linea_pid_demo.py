#!/usr/bin/env python3
"""Seguidor de linea con control PID.

    python3 seguidor_linea_pid_demo.py

🔴 NECESITA EL ARRANQUE CON color_detection:=true. Lee la practica 5.

Antes de ejecutarlo: una linea negra sobre suelo claro, y el robot encima de
ella mirando en la direccion de avance.

El PID de este fichero es el mismo que antes: es lo que se estudia. Lo que
cambia es de donde sale el color y a donde va la velocidad.
"""
import json
import sys
import time
from pathlib import Path

from atriz import Robot


class PID:
    """Control proporcional-integral-derivativo.

    error -> salida. Kp reacciona a lo que pasa AHORA, Ki a lo acumulado, Kd a
    lo rapido que cambia.
    """

    def __init__(self, kp=0.5, ki=0.0, kd=0.3, limite=1.5):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.limite = limite
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.error_anterior = 0.0
        self.instante_anterior = None

    def calcular(self, error):
        ahora = time.monotonic()
        dt = 0.1 if self.instante_anterior is None else ahora - self.instante_anterior
        self.instante_anterior = ahora
        if dt <= 0.0:
            dt = 1e-3

        self.integral += error * dt
        derivada = (error - self.error_anterior) / dt
        self.error_anterior = error

        salida = self.kp * error + self.ki * self.integral + self.kd * derivada
        return max(-self.limite, min(self.limite, salida))


CONFIG = Path(__file__).parent / 'seguidor_config.json'
ajustes = json.loads(CONFIG.read_text()) if CONFIG.exists() else {}

VELOCIDAD = ajustes.get('velocidad', 0.08)          # m/s
UMBRAL = ajustes.get('umbral_claro', 400)           # claro por debajo = linea
PERIODO = 0.1                                       # s -> 10 Hz, y el watchdog
                                                    #     corta a los 0.3

with Robot() as robot:
    if not robot.hay_color:
        print('\nEl sensor de color esta apagado. Lee la practica 5.')
        sys.exit(1)

    pid = PID(**ajustes.get('pid', {}))
    print('Siguiendo la linea. Ctrl-C para parar.\n')

    while True:
        inicio = time.monotonic()
        _, _, _, claro = robot.color()

        # El error: cuanto se aleja el sensor de estar sobre la linea.
        # Normalizado para que el PID no dependa de la escala del sensor.
        error = (claro - UMBRAL) / UMBRAL

        robot.mover(VELOCIDAD, -pid.calcular(error))

        # 🔴 El ritmo lo marca este sleep, y tiene que ser menor que 0.3 s: es
        #    lo que tarda el watchdog del driver en cortar. Leer el color cuesta
        #    13-20 ms, asi que cabe de sobra.
        time.sleep(max(0.0, PERIODO - (time.monotonic() - inicio)))

# EJERCICIOS
#   1. Pon Kd a 0. ¿Que le pasa al robot en las curvas?
#   2. Sube Kp hasta que oscile. Eso es la ganancia critica.
#   3. Sube VELOCIDAD a 0.20. ¿Sigue valiendo el mismo PID?
#   4. Sube PERIODO a 0.5. ¿Por que va a tirones? (pista: el watchdog)
