#!/usr/bin/env python3
"""Practica 23 — Un tren de robots: uno sigue a otro. DOS ROBOTS O MAS.

    En la locomotora:  python3 23_tren_de_robots.py baliza 0 1
    En el vagon:       python3 23_tren_de_robots.py seguidor 0 1

Los dos numeros son los codigos LEJOS y CERCA, y tienen que ser LOS MISMOS en
los dos robots o no se encontraran.

🔴🔴 ESTA PRACTICA MUEVE EL ROBOT, Y NO LA PROTEGE LA CAPA DE SEGURIDAD.

   `seguir_a_otro()` activa un modo del FIRMWARE del RVR: el robot conduce solo
   al detectar infrarrojos, SIN pasar por `/cmd_vel`. Eso significa que:

       · el `collision_monitor` NO lo ve — no frenara ante un obstaculo
       · el watchdog del driver NO lo ve — no le corta si dejas de mandarle nada

   Lo unico que lo para es la parada de emergencia y `parar_ir()`. Esta
   biblioteca los apaga sola al cerrar, incluso con Ctrl-C, PERO ESO NO
   SUSTITUYE A MIRAR: 👤 espacio despejado, suelo continuo, y no te vayas.

CADENA DE VARIOS: el seguidor puede a su vez hacer de baliza con OTRO par de
codigos, y un tercer robot seguirle a el. Con ocho codigos salen tres o cuatro
eslabones antes de que se solapen.

📝 POR QUE HAY DOS CODIGOS: el firmware usa uno para lejos (3 m o mas) y otro
   para cerca (menos de 1 m), y con los dos deduce distancia y direccion. Es lo
   unico que la documentacion de Sphero dice de forma cuantitativa sobre el IR.
"""
import sys
import time

from atriz import Robot

if len(sys.argv) != 4 or sys.argv[1] not in ('baliza', 'seguidor'):
    print('Uso: python3 23_tren_de_robots.py baliza|seguidor <LEJOS> <CERCA>')
    print('Ejemplo:  python3 23_tren_de_robots.py baliza 0 1')
    sys.exit(1)

papel = sys.argv[1]
try:
    lejos, cerca = int(sys.argv[2]), int(sys.argv[3])
except ValueError:
    print('LEJOS y CERCA tienen que ser numeros del 0 al 7.')
    sys.exit(1)

with Robot() as robot:
    if papel == 'baliza':
        print(f'Soy la LOCOMOTORA (codigos {lejos}/{cerca}). No me muevo solo:')
        print('empujame o condúceme tu, y el vagon me seguira.')
        robot.emitir_como_baliza(lejos, cerca)
    else:
        print(f'Soy el VAGON (codigos {lejos}/{cerca}).')
        print('🔴 Voy a CONDUCIR SOLO en cuanto detecte a la locomotora.')
        robot.seguir_a_otro(lejos, cerca)

    print('\nCtrl-C para parar. Al salir se apaga solo.\n')
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    print('Parando el IR...')
