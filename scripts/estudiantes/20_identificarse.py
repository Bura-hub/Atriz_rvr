#!/usr/bin/env python3
"""Practica 20 — Identificarse por infrarrojos. NECESITA DOS ROBOTS.

    En cada robot:  python3 20_identificarse.py <TU_CODIGO>

Cada robot emite un numero del 0 al 7 y escucha los de los demas. Es la practica
mas sencilla del IR y la base de las cuatro siguientes.

NO MUEVE EL ROBOT. Solo enciende los emisores de infrarrojos, que no se ven.

🔴 SOLO HAY OCHO CODIGOS (0-7), y el laboratorio tiene DIECISEIS robots. O sea
   que dos robots del aula tendran por fuerza el mismo codigo, y entonces son
   indistinguibles entre si. No es un fallo del programa: es el limite del
   hardware, y conviene saberlo antes de diseñar nada encima.

⚠️ LA DETECCION ES INTERMITENTE. Medido el 2026-08-11: emitiendo cada 0.4 s con
   un registro que se borra al segundo, aun asi aparecen lecturas vacias entre
   medias. Por eso este programa escucha en bucle en vez de mirar una vez: UNA
   SOLA LECTURA PUEDE DECIR «no hay nadie» HABIENDOLO.
"""
import sys
import time

from atriz import Robot

if len(sys.argv) != 2 or not sys.argv[1].isdigit() or not 0 <= int(sys.argv[1]) <= 7:
    print('Uso: python3 20_identificarse.py <TU_CODIGO>   (un numero del 0 al 7)')
    print('Pon un codigo DISTINTO en cada robot, o no podras distinguirlos.')
    sys.exit(1)

mi_codigo = int(sys.argv[1])

with Robot() as robot:
    print(f'Soy el robot con codigo {mi_codigo}. Escuchando a los demas...')
    print('Ctrl-C para terminar.\n')

    visto = None
    try:
        while True:
            robot.emitir_ir(mi_codigo)
            otro = robot.escuchar_ir()

            # Solo se anuncia cuando CAMBIA, o la pantalla seria ilegible.
            if otro != visto:
                if otro is None:
                    print('  ...no oigo a nadie')
                elif otro == mi_codigo:
                    print(f'  🔴 oigo el codigo {otro}, que es EL MIO. '
                          'O me estoy oyendo por rebote, o hay otro robot con mi codigo.')
                else:
                    print(f'  ✅ hay un robot con el codigo {otro}')
                visto = otro

            time.sleep(0.3)
    except KeyboardInterrupt:
        print('\nHasta luego.')
