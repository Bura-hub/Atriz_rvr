#!/usr/bin/env python3
"""Practica 11 — Reaccionar a lo que ve: parar sobre negro.

    python3 11_sensor_avanzado.py

🔴 NECESITA EL ARRANQUE CON color_detection:=true. Lee la cabecera de la
   practica 5.

Antes de ejecutarlo: 1 metro despejado por delante, y una franja de cinta negra
cruzando el camino del robot.
"""
import sys

from atriz import Robot

# El canal claro va de ~181 sobre negro a ~2288 sobre blanco (evidencia 37). El
# umbral NO esta a la mitad de ese recorrido (seria ~1235): el suelo real, sin
# cinta encima, ya da clear=1275 — casi tan alto como el blanco. Un umbral a
# mitad de camino confundiria «suelo normal» con «casi blanco» y no serviria
# para nada. UMBRAL_NEGRO=400 se queda muy por debajo del suelo (1275) y bien
# por encima del negro (181): esa es la separacion que hace falta.
#
# ⚠️ El azul mide clear=396 (evidencia 37) — 4 unidades por DEBAJO de este
#    umbral. Si pruebas con cinta azul en vez de negra, el robot la clasifica
#    como negro igual: no es un fallo del programa, es que 400 separa negro de
#    SUELO, no negro de cualquier otro color. No se sube el umbral sin una
#    medida nueva: la practica pide cinta negra a proposito.
UMBRAL_NEGRO = 400

# Lo que avanza cada tramo. Estas dos son las que toca el ejercicio 2.
VELOCIDAD = 0.10             # m/s
DURACION_TRAMO = 0.2         # s
AVANCE_POR_TRAMO = VELOCIDAD * DURACION_TRAMO      # m

# 🔴 EL TOPE ES POR DISTANCIA, NO POR NUMERO DE TRAMOS, y esto es el arreglo de
#    un defecto real. Antes habia un `MAX_TRAMOS = 60` fijo, calculado para
#    0.10 m/s. Pero el ejercicio 2 manda subir la velocidad a 0.30 — y subir la
#    velocidad NO cambiaba el tope, asi que los mismos 60 tramos pasaban de
#    1.20 m a 60 x (0.30 x 0.2) = 3.60 m: TRES VECES Y MEDIA el metro que esta
#    practica pide despejado. El tope existe precisamente para cuando la cinta
#    no se detecta, o sea justo cuando el robot va a recorrerlo entero.
#    Derivandolo de la distancia, el ejercicio 2 se protege solo: a 0.30 m/s
#    salen 15 tramos, y el recorrido maximo sigue siendo el mismo.
#
# De donde sale la distancia:
#   - esta practica pide 1 m despejado por delante.
#   - el collision_monitor frena solo a ~9.9-10.7 cm de un obstaculo (medido a
#     0.25 y 0.40 m/s, 2026-07-31), asi que del metro quedan ~90 cm utiles.
#     🔴 A 0.10 m/s esa frenada NO esta medida: es una extrapolacion desde
#     0.25/0.40 m/s. Va del lado seguro (mas lento = frena antes), pero es
#     extrapolacion, no medida.
DISTANCIA_MAX_M = 0.90

# Tramos que caben en esa distancia. Se redondea hacia ABAJO, nunca hacia
# arriba: el tope no puede prometer mas recorrido del que hay sitio.
# El `1e-9` no es adorno: 0.90 / 0.02 da 44.999... en coma flotante, asi que un
# `int()` a secas se comia un tramo entero (44 en vez de 45) sin que se notara.
MAX_TRAMOS = int(DISTANCIA_MAX_M / AVANCE_POR_TRAMO + 1e-9)

# ~0.7 s por tramo de verdad (0.2 s moviendo + 0.5 s de parar() insistiendo en
# velocidad cero) — leido de avanzar()/parar() en atriz.py, no medido con
# cronometro.
SEGUNDOS_POR_TRAMO = DURACION_TRAMO + 0.5

with Robot() as robot:
    if not robot.hay_color:
        print('\nEl sensor de color esta apagado. Lee la practica 5.')
        sys.exit(1)

    print(f'Avanzando a {VELOCIDAD} m/s hasta encontrar negro '
          f'(maximo {MAX_TRAMOS} tramos = {MAX_TRAMOS * AVANCE_POR_TRAMO:.2f} m, '
          f'~{MAX_TRAMOS * SEGUNDOS_POR_TRAMO:.0f} s)...')
    # 🔴 `avanzar()` SI se usa, pero a trocitos. Es la idea de toda la practica:
    #    `avanzar(0.10, 5)` bloquearia 5 segundos enteros, y durante ese rato
    #    nadie mira el sensor — el robot cruzaria la cinta negra sin enterarse.
    #    Troceando el avance en tramos de 0.2 s, entre tramo y tramo se lee el
    #    color: eso es el lazo sentir-actuar. Lo que NO se usa es una sola
    #    llamada larga a `avanzar()`.
    #    (Si necesitaras leer el sensor SIN parar entre medias, la herramienta
    #     es `robot.mover()`, que no bloquea; asi lo hace el seguidor de linea.)
    tramos = 0
    while True:
        _, _, _, claro = robot.color()
        if claro < UMBRAL_NEGRO:
            print(f'Negro detectado (claro={claro}) tras {tramos} tramos. Parando.')
            robot.parar()
            break
        if tramos >= MAX_TRAMOS:
            robot.parar()
            print(f'\nAVISO: {MAX_TRAMOS} tramos '
                  f'({MAX_TRAMOS * AVANCE_POR_TRAMO:.2f} m) sin encontrar negro (ultima '
                  f'lectura: claro={claro}, umbral={UMBRAL_NEGRO}). Parando.\n'
                  f'       Revisa, en este orden:\n'
                  f'         1. ¿esta puesta la cinta negra y cruza el camino '
                  f'del robot?\n'
                  f'         2. ¿el robot arranco con color_detection:=true? '
                  f'(si no, claro nunca baja de verdad)\n'
                  f'         3. ¿el sensor mira al suelo, o el robot quedo '
                  f'atascado contra algo antes de llegar?')
            sys.exit(1)
        if tramos % 10 == 0:
            print(f'  tramo {tramos}: claro={claro} (sigue buscando negro)')
        # Tramos cortos, para poder leer el sensor entre uno y otro. Usa las
        # constantes de arriba a proposito: el tope de distancia sale de las
        # MISMAS dos, asi que subir la velocidad en el ejercicio 2 baja el
        # numero de tramos y el recorrido maximo no se mueve.
        robot.avanzar(VELOCIDAD, DURACION_TRAMO)
        tramos += 1

# EJERCICIOS
#   1. Baja el umbral a 200. ¿Se le pasa la linea?
#   2. Sube VELOCIDAD a 0.30. ¿Que le pasa a la distancia de parada?
#      Mira ademas lo que imprime al arrancar: MAX_TRAMOS baja de 45 a 15 solo.
#      ¿Por que el tope esta escrito en METROS y no en numero de tramos?
#      (pista: el tope existe para cuando la cinta NO se detecta, o sea justo
#       cuando el robot va a recorrerlo entero. Con un tope fijo de 60 tramos,
#       este ejercicio recorreria 3.60 m con 1 m despejado.)
#   3. Haz que retroceda 20 cm despues de encontrar el negro.
#      🔴 EL ROBOT VA HACIA ATRAS. Comprueba que hay sitio DETRAS de el: al
#         menos 40 cm, contando que el robot mide 19 cm de largo. Detras no
#         hay capa de seguridad — el poligono del collision_monitor mira hacia
#         DELANTE, asi que frenara igual aunque te estes alejando de la pared
#         (medido: 30 cm comandados hacia atras dieron 14 cm), pero de lo que
#         tengas detras no te avisa nadie.
