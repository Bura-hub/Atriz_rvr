#!/usr/bin/env python3
"""Seguidor de linea: PID + seguimiento de BORDE.

    python3 seguidor_linea_pid_demo.py

🔴 NECESITA EL ARRANQUE CON color_detection:=true. Lee la practica 5.

Coloca el robot mirando en la direccion de avance, con el sensor JUSTO
sobre el borde derecho de la linea: la LINEA (negro) a su izquierda y el
SUELO (claro) a su derecha.

Por que "borde" y no "centro": el sensor de color de este robot es UNO
SOLO y mira hacia abajo. Devuelve un unico numero (`claro`): mas alto
cuanto mas claro hay debajo. Ese numero NO dice hacia que lado se ha
desviado el robot -- sale igual de alto si el robot se fue a la izquierda
de la linea que si se fue a la derecha. Con un solo sensor centrado no
hay forma de distinguirlo (ver SEGUIDOR_LINEA_EXPLICACION.md). Por eso
este seguidor no intenta centrarse: sigue SIEMPRE el mismo borde, como
quien camina apoyando la mano en una pared.

El PID sigue siendo el que se estudia, sin tocar: decide la MAGNITUD del
giro, a partir de cuanto se aleja el sensor del borde. La DIRECCION
(izquierda o derecha) la decide `lado_borde` -- un estado que se arrastra
entre vueltas del bucle, fijado por la convencion de arranque y revisado
si nos perdemos -- nunca la lectura de un solo instante.

Umbrales: el canal `claro`, no una media RGB. Los dos puntos de anclaje
son medidas de la evidencia 37
(00_auditoria/evidencia_24_04/37_sensores_opticos.txt, 2026-08-01):
negro=181, suelo real=1275. Los margenes hasta UMBRAL_NEGRO/UMBRAL_CLARO
son una decision razonada a partir de esos dos puntos, no una tercera
medida -- no hay datos de ruido del sensor (lecturas repetidas) en esa
evidencia. Ver seguidor_config.json.
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

VELOCIDAD = ajustes.get('velocidad', 0.08)                # m/s
# claro (evidencia 37): negro=181, suelo real=1275. Margen razonado desde
# esos dos anclajes -- ver el docstring de arriba.
UMBRAL_NEGRO = ajustes.get('umbral_negro', 400)           # claro <= esto: sobre la linea
UMBRAL_CLARO = ajustes.get('umbral_claro', 1000)          # claro >= esto: sobre el suelo
MARGEN_HISTERESIS = ajustes.get('margen_histeresis', 50)  # evita parpadeo cerca de un umbral
LADO_INICIAL = ajustes.get('lado_borde', 1)               # +1: linea a la izquierda, suelo a la derecha
TIEMPO_PERDIDO_MAX = ajustes.get('tiempo_perdido_max', 1.0)  # s en 'claro' antes de invertir la hipotesis
PERIODO = 0.1                                              # s -> 10 Hz, y el watchdog
                                                            #     corta a los 0.3


def clasificar(claro, umbral_negro=UMBRAL_NEGRO, umbral_claro=UMBRAL_CLARO,
                margen=MARGEN_HISTERESIS):
    """'negro' / 'borde' / 'claro', con histeresis para no parpadear cerca
    de un umbral."""
    if claro <= umbral_negro + margen:
        return 'negro'
    if claro >= umbral_claro - margen:
        return 'claro'
    return 'borde'


def signo_correccion(estado, lado_borde):
    """+1 o -1: hacia donde girar para volver al borde.

    Si estamos en 'claro' (nos pasamos hacia el suelo) hay que girar hacia
    la linea; si estamos en 'negro' (nos pasamos hacia dentro de la linea)
    hay que girar hacia el suelo -- son giros CONTRARIOS. Cual de los dos
    es "izquierda" y cual "derecha" en el mundo lo fija `lado_borde`, no
    esta lectura: por eso el signo depende del LADO, no del sensor.
    """
    return lado_borde if estado == 'claro' else -lado_borde


def magnitud_correccion(claro, pid, umbral_negro=UMBRAL_NEGRO, umbral_claro=UMBRAL_CLARO):
    """El PID decide CUANTO corregir. Nunca hacia donde.

    El error que recibe es siempre >= 0: la distancia normalizada entre
    `claro` y el punto medio de la histeresis. Cerca del borde el error es
    chico (es donde queremos estar); muy dentro de la linea o muy en el
    suelo, grande -- en los dos casos por igual, porque lo que importa
    aqui es SOLO cuanto, no en que direccion.
    """
    centro = (umbral_negro + umbral_claro) / 2.0
    error = abs(claro - centro) / centro
    return max(0.0, pid.calcular(error))


def decidir_giro(claro, lado_borde, pid, umbral_negro=UMBRAL_NEGRO, umbral_claro=UMBRAL_CLARO):
    """El giro que se manda al robot: magnitud del PID, signo del lado del
    borde. Es la funcion que hace que esto pueda funcionar con un solo
    sensor -- pruebala con el mismo `claro` y `lado_borde` contrario."""
    estado = clasificar(claro, umbral_negro, umbral_claro)
    signo = signo_correccion(estado, lado_borde)
    magnitud = magnitud_correccion(claro, pid, umbral_negro, umbral_claro)
    return signo * magnitud


def main():
    with Robot() as robot:
        if not robot.hay_color:
            print('\nEl sensor de color esta apagado. Lee la practica 5.')
            sys.exit(1)

        pid = PID(**ajustes.get('pid', {}))
        lado_borde = LADO_INICIAL
        perdido_desde = None
        print('Siguiendo el borde de la linea. Ctrl-C para parar.\n')

        while True:
            inicio = time.monotonic()
            _, _, _, claro = robot.color()
            estado = clasificar(claro)

            if estado == 'claro':
                if perdido_desde is None:
                    perdido_desde = inicio
                elif inicio - perdido_desde > TIEMPO_PERDIDO_MAX:
                    # Mas de un segundo sin reencontrar el borde por este
                    # lado: la hipotesis de hacia donde esta la linea
                    # estaba mal. Se invierte y se prueba la otra.
                    lado_borde = -lado_borde
                    perdido_desde = inicio
            else:
                perdido_desde = None

            giro = decidir_giro(claro, lado_borde, pid)
            robot.mover(VELOCIDAD, giro)

            # 🔴 El ritmo lo marca este sleep, y tiene que ser menor que 0.3 s: es
            #    lo que tarda el watchdog del driver en cortar. Leer el color cuesta
            #    13-20 ms, asi que cabe de sobra.
            time.sleep(max(0.0, PERIODO - (time.monotonic() - inicio)))


if __name__ == '__main__':
    main()

# EJERCICIOS
#   1. Pon Kd a 0. ¿Que le pasa al robot en las curvas?
#   2. Sube Kp hasta que oscile. Eso es la ganancia critica.
#   3. Sube VELOCIDAD a 0.20. ¿Sigue valiendo el mismo PID?
#   4. Sube PERIODO a 0.5. ¿Por que va a tirones? (pista: el watchdog)
