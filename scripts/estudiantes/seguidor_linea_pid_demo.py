#!/usr/bin/env python3
"""Seguidor de linea: PID + seguimiento de BORDE.

    python3 seguidor_linea_pid_demo.py

Enciende el solo la luz del sensor de color y la apaga al terminar.

🔴 ANTES DE EJECUTARLO — ESPACIO: una pista con linea negra sobre suelo
   claro, y sitio para al menos 6 METROS DE RECORRIDO, que es el tope que
   trae este guion (`distancia_max_m` en seguidor_config.json). A 0.08 m/s
   son unos 75 segundos. Son metros de CAMINO, no en linea recta: en un
   circuito con curvas el robot recorre mas metros de los que avanza, asi
   que un circuito cerrado de 2 m de lado vale de sobra, pero una pista
   recta necesita 6 m de verdad.
   ⚠️ Este aviso va AQUI ARRIBA a proposito: los mensajes que imprime el
      programa salen DESPUES de que Robot() haya conectado y encendido el
      LIDAR, o sea demasiado tarde para despejar el suelo.

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
# 🔴 OJO: la misma tabla de evidencia 37 trae azul=396 (superficie azul,
# no la linea) -- POR DEBAJO de UMBRAL_NEGRO=400. Con este margen, un
# objeto azul bajo el sensor se clasifica como 'negro', indistinguible de
# la linea real. No se cambia el umbral sin una medida nueva (evidencia
# 37 no trae ruido repetido para afinar el margen), pero queda anotado:
# este seguidor NO discrimina el azul de la linea negra.
UMBRAL_NEGRO = ajustes.get('umbral_negro', 400)           # claro <= esto: sobre la linea
UMBRAL_CLARO = ajustes.get('umbral_claro', 1000)          # claro >= esto: sobre el suelo
# 🔴 Se llamaba MARGEN_HISTERESIS y NO hay histeresis: `clasificar()` no tiene
#    estado ni recuerda la lectura anterior. Lo unico que hace este numero es
#    ensanchar la zona 'borde' hacia dentro de los dos umbrales. El nombre viejo
#    prometia un disparador de Schmitt que no existe.
MARGEN_BORDE = ajustes.get('margen_borde', 50)            # ensancha 'borde'; ver clasificar()
LADO_INICIAL = ajustes.get('lado_borde', 1)               # +1: linea a la izquierda, suelo a la derecha
TIEMPO_PERDIDO_MAX = ajustes.get('tiempo_perdido_max', 1.0)  # s en 'claro' antes de invertir la hipotesis
PERIODO = 0.1                                              # s -> 10 Hz, y el watchdog
                                                            #     corta a los 0.3

# 🔴 TOPE DE RECORRIDO. Este bucle era un `while True` a 0.08 m/s SIN NINGUN
#    tope: si la pista se acaba, si el sensor se desalinea o si la hipotesis del
#    lado del borde es la contraria, el robot se va recto y en silencio hasta
#    que alguien lo coge. La practica 11 ya puso tope razonando que «sin tope
#    seguiria avanzando en silencio»; aqui vale exactamente igual, y encima
#    este guion es el proyecto FINAL, el que se deja corriendo mas rato.
#    A 0.08 m/s, 6 m son ~75 s. Es recorrido de CAMINO (lo que anda el robot),
#    no distancia en linea recta: en una pista con curvas el robot recorre mas
#    metros de los que avanza.
DISTANCIA_MAX_M = ajustes.get('distancia_max_m', 6.0)


def clasificar(claro, umbral_negro=UMBRAL_NEGRO, umbral_claro=UMBRAL_CLARO,
                margen=MARGEN_BORDE):
    """'negro' / 'borde' / 'claro'.

    🔴 `margen` NO es histeresis de verdad -- esta funcion no tiene
    estado ni recuerda la clasificacion anterior. Por eso la constante se
    llama MARGEN_BORDE y no MARGEN_HISTERESIS, que era el nombre viejo. Lo
    unico que hace es ensanchar la zona 'borde' hacia dentro de los dos
    umbrales. Una lectura oscilando justo en `umbral_negro + margen`
    cambia de 'negro' a 'borde' en cada muestra, exactamente igual que
    con un umbral simple sin margen -- solo que el punto de corte queda
    mas lejos de `umbral_negro`. Histeresis de verdad (tipo disparador de
    Schmitt) necesitaria usar un umbral distinto segun cual fue el
    estado anterior, y esta funcion no lo hace.
    """
    if claro <= umbral_negro + margen:
        return 'negro'
    if claro >= umbral_claro - margen:
        return 'claro'
    return 'borde'


def signo_correccion(claro, lado_borde, umbral_negro=UMBRAL_NEGRO, umbral_claro=UMBRAL_CLARO):
    """+1 o -1: hacia donde girar para volver al borde.

    Cambia de signo exactamente donde `claro` cruza el CENTRO (el punto
    medio entre `umbral_negro` y `umbral_claro`) -- la MISMA frontera que
    usa `magnitud_correccion()`. Si estamos pasados hacia el suelo hay que
    girar hacia la linea; si estamos pasados hacia dentro de la linea hay
    que girar hacia el suelo -- son giros CONTRARIOS. Cual de los dos es
    "izquierda" y cual "derecha" en el mundo lo fija `lado_borde`, no esta
    lectura: por eso el signo depende del LADO, no del sensor.

    🔴 Ronda de arreglo 2: antes el signo salia de `clasificar()`, que
    tiene SUS PROPIAS fronteras (ensanchadas por `margen`, para decidir
    cuando estamos perdidos). Entre el centro y esa frontera el signo y la
    magnitud se contradecian -- realimentacion positiva, el robot se
    alejaba del borde en vez de volver. `clasificar()` sigue sirviendo
    para la logica de perdida/reenganche en `main()`, pero YA NO decide el
    signo de la correccion continua.
    """
    centro = (umbral_negro + umbral_claro) / 2.0
    return lado_borde if claro >= centro else -lado_borde


def magnitud_correccion(claro, pid, umbral_negro=UMBRAL_NEGRO, umbral_claro=UMBRAL_CLARO):
    """El PID decide CUANTO corregir. Nunca hacia donde.

    El error que recibe es siempre >= 0: la distancia normalizada entre
    `claro` y el mismo CENTRO que usa `signo_correccion()`. Cerca del
    centro el error es chico (es donde queremos estar); muy dentro de la
    linea o muy en el suelo, grande -- en los dos casos por igual, porque
    lo que importa aqui es SOLO cuanto, no en que direccion.
    """
    centro = (umbral_negro + umbral_claro) / 2.0
    error = abs(claro - centro) / centro
    return max(0.0, pid.calcular(error))


def decidir_giro(claro, lado_borde, pid, umbral_negro=UMBRAL_NEGRO, umbral_claro=UMBRAL_CLARO):
    """El giro que se manda al robot: magnitud del PID, signo del lado del
    borde -- las dos funciones miden desde el MISMO centro, para que el
    giro sea continuo y cambie de signo justo donde la magnitud pasa por
    cero. Es la funcion que hace que esto pueda funcionar con un solo
    sensor -- pruebala con el mismo `claro` y `lado_borde` contrario."""
    signo = signo_correccion(claro, lado_borde, umbral_negro, umbral_claro)
    magnitud = magnitud_correccion(claro, pid, umbral_negro, umbral_claro)
    return signo * magnitud


def main():
    with Robot() as robot:
        # La luz del sensor, lo primero: sin ella no hay linea que seguir.
        robot.sensor_color(True)

        pid = PID(**ajustes.get('pid', {}))
        lado_borde = LADO_INICIAL
        perdido_desde = None
        recorrido = 0.0
        arranque = time.monotonic()
        print(f'Siguiendo el borde de la linea (tope: {DISTANCIA_MAX_M:.1f} m de '
              f'camino, ~{DISTANCIA_MAX_M / VELOCIDAD:.0f} s). Ctrl-C para parar.\n')

        while True:
            inicio = time.monotonic()

            # El tope de recorrido, ANTES de mandar nada mas.
            recorrido = VELOCIDAD * (inicio - arranque)
            if recorrido >= DISTANCIA_MAX_M:
                robot.parar()
                print(f'\nAVISO: {DISTANCIA_MAX_M:.1f} m de camino recorridos sin '
                      f'que nadie parase esto. Parando.\n'
                      f'       Si el robot iba bien, sube `distancia_max_m` en '
                      f'seguidor_config.json.\n'
                      f'       Si no, revisa en este orden:\n'
                      f'         1. ¿el sensor arranco JUSTO sobre el borde de la '
                      f'linea, con el negro al lado que dice `lado_borde`?\n'
                      f'         2. ¿se encendio la luz del sensor? (LED blanco bajo el robot)\n'
                      f'         3. ¿la pista tiene curvas mas cerradas de lo que '
                      f'este PID puede seguir a {VELOCIDAD} m/s?')
                sys.exit(1)
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
            #    20.6-20.8 ms (medido, n=200), asi que cabe de sobra en los 100 ms
            #    de PERIODO.
            time.sleep(max(0.0, PERIODO - (time.monotonic() - inicio)))


if __name__ == '__main__':
    main()

# EJERCICIOS
#   1. Pon Kd a 0. ¿Que le pasa al robot en las curvas?
#   2. Sube Kp hasta que oscile. Eso es la ganancia critica.
#   3. Sube VELOCIDAD a 0.20. ¿Sigue valiendo el mismo PID?
#   4. Sube PERIODO a 0.5. ¿Por que va a tirones? (pista: el watchdog)
