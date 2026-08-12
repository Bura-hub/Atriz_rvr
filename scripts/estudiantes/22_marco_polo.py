#!/usr/bin/env python3
"""Practica 22 — Marco Polo: buscar a otro robot por infrarrojos. DOS ROBOTS.

    En el que se esconde:  python3 22_marco_polo.py marco
    En el que busca:       python3 22_marco_polo.py polo

Uno emite y no se mueve. El otro usa sus sensores para adivinar por donde le
llega la señal.

👤 EL QUE BUSCA NO SE MUEVE SOLO: te dice hacia donde cree que esta el otro y tu
   lo mueves. Es a proposito — asi el alumno ve la lectura y decide, en vez de
   mirar un robot que hace cosas.

🔴 Y AQUI ESTA LA LECCION DE LA PRACTICA, que no es el juego:

   Los cuatro sensores del RVR NO dan cuatro direcciones. Esta medido con los
   dos robots el 2026-08-11, girandolos 360 grados (evidencia 100):

       [1] solo          ->  el otro esta a la IZQUIERDA
       [1,3] / [1,2,3]   ->  esta DETRAS
       [2,3]             ->  esta DELANTE **o** A LA DERECHA   <- no se separan
       sensor_0          ->  NUNCA lleva datos, en ninguno de los dos robots

   La documentacion del fabricante dice que son cuatro esquinas... pero esa
   documentacion es del Sphero BOLT, que es otro robot. Medirlo fue lo que lo
   descubrio.

⚠️ Y ademas REBOTA. En interior el infrarrojo se refleja en paredes y suelo, asi
   que «lo tengo a la izquierda» puede ser un robot a la derecha reflejado.
"""
import sys
import time

from atriz import Robot


def interpretar(lecturas):
    """De los cuatro bytes al unico enunciado que se puede sostener."""
    VACIO = 255
    activos = {i for i, v in enumerate(lecturas) if v != VACIO}
    if not activos:
        return 'no oigo nada'
    if activos == {1}:
        return 'lo tengo a la IZQUIERDA'
    if 1 in activos and 3 in activos:
        return 'lo tengo DETRAS'
    if activos <= {2, 3}:
        return 'lo tengo DELANTE o A LA DERECHA (no puedo separarlos)'
    return f'oigo algo, pero el patron {sorted(activos)} no es de los medidos'


if len(sys.argv) != 2 or sys.argv[1] not in ('marco', 'polo'):
    print('Uso: python3 22_marco_polo.py marco|polo')
    sys.exit(1)

with Robot() as robot:
    if sys.argv[1] == 'marco':
        print('Soy MARCO: emito y no me muevo. Ctrl-C para parar.\n')
        try:
            while True:
                robot.emitir_ir(5)
                print('\r  ...marco', end='', flush=True)
                time.sleep(0.3)
        except KeyboardInterrupt:
            print()
    else:
        print('Soy POLO: busco a Marco. Muevemelo tu segun lo que diga.')
        print('Ctrl-C para parar.\n')
        try:
            while True:
                # Tres muestras, no una: la deteccion falla a ratos y una sola
                # lectura diria «no oigo nada» con Marco delante.
                muestras = []
                for _ in range(3):
                    try:
                        muestras.append(robot.quien_hay_cerca())
                    except Exception as e:              # noqa: BLE001
                        print(f'  no puedo leer los sensores: {e}')
                        raise SystemExit(1)
                    time.sleep(0.15)
                # Se queda el que mas se repite: el ruido no gana por mayoria.
                elegida = max(muestras, key=muestras.count)
                print(f'  {interpretar(elegida)}     {elegida}')
                time.sleep(0.4)
        except KeyboardInterrupt:
            pass
    print('\nHasta luego.')
