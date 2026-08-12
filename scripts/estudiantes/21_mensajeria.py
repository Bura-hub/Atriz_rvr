#!/usr/bin/env python3
"""Practica 21 — Un protocolo de mensajes entre robots. NECESITA DOS ROBOTS.

    En el que avisa:   python3 21_mensajeria.py emisor
    En el que escucha: python3 21_mensajeria.py receptor

Con ocho codigos se puede montar un lenguaje. No hace falta red: esto es luz
infrarroja entre robots, y funciona aunque el WiFi se caiga.

NO MUEVE EL ROBOT.

🔴 UN MENSAJE ES UN NUMERO, y nada mas. No lleva datos dentro: no puedes mandar
   «ve a la posicion 3,4». Puedes mandar «he llegado». La gracia de la practica
   es justo esa: diseñar un vocabulario util con ocho palabras.

🔴 Y NO SABES QUIEN TE HABLA. El mensaje trae el codigo, no la identidad del que
   lo manda. Si dos robots usan el mismo codigo, son la misma voz.
"""
import sys
import time

from atriz import Robot

#: El vocabulario. Cambialo: es el ejercicio.
VOCABULARIO = {
    0: 'hola, estoy aqui',
    1: 'he llegado a la meta',
    2: 'he encontrado la linea',
    3: 'me toca a mi',
    4: 'he terminado',
    5: 'necesito ayuda',
    6: 'aparta, voy a pasar',
    7: 'paramos todos',
}

if len(sys.argv) != 2 or sys.argv[1] not in ('emisor', 'receptor'):
    print('Uso: python3 21_mensajeria.py emisor|receptor')
    sys.exit(1)

with Robot() as robot:
    if sys.argv[1] == 'emisor':
        print('Vocabulario disponible:')
        for c, texto in VOCABULARIO.items():
            print(f'   {c} = {texto}')
        print('\nEscribe un numero y ENTER para mandarlo. Ctrl-C para salir.\n')
        try:
            while True:
                try:
                    entrada = input('mensaje> ').strip()
                except EOFError:
                    break
                if not entrada.isdigit() or int(entrada) not in VOCABULARIO:
                    print('  Escribe un numero del 0 al 7.')
                    continue
                codigo = int(entrada)
                # 🔴 Se repite TRES veces, y no es por gusto: la deteccion es
                #    intermitente (medido el 2026-08-11). Un mensaje mandado una
                #    sola vez se pierde a menudo.
                for _ in range(3):
                    robot.emitir_ir(codigo)
                    time.sleep(0.2)
                print(f'  enviado {codigo}: «{VOCABULARIO[codigo]}»')
        except KeyboardInterrupt:
            pass
    else:
        print('Escuchando mensajes. Ctrl-C para salir.\n')
        ultimo = None
        try:
            while True:
                codigo = robot.escuchar_ir(caducidad=1.5)
                if codigo is not None and codigo != ultimo:
                    texto = VOCABULARIO.get(codigo, '(codigo sin significado)')
                    print(f'  📨 {codigo}: «{texto}»')
                ultimo = codigo
                time.sleep(0.2)
        except KeyboardInterrupt:
            pass
    print('\nHasta luego.')
