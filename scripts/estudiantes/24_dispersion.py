#!/usr/bin/env python3
"""Practica 24 — Dispersion: los robots se separan solos. DOS ROBOTS O MAS.

    En TODOS los robots:  python3 24_dispersion.py

Cada robot emite y a la vez huye de lo que oye. Puestos juntos, se separan.

🔴🔴 ESTA PRACTICA MUEVE EL ROBOT, Y NO LA PROTEGE LA CAPA DE SEGURIDAD.
   `huir_de_otro()` es un modo del FIRMWARE: el robot conduce solo, sin pasar por
   `/cmd_vel`, asi que ni el `collision_monitor` ni el watchdog intervienen.
   👤 Espacio despejado, suelo continuo y sin escalones, y no te vayas.

   ⚠️ Y con VARIOS robots huyendo a la vez, el conjunto se expande. Necesitas
      mas sitio del que parece al empezar.

⏳ LO QUE NO SABEMOS, y hay que decirlo: esto se ha probado con DOS robots. Con
   ocho emitiendo a la vez puede haber colisiones de codigos e interferencias, y
   NO ESTA MEDIDO. Si lo pruebas con mas, anota lo que pase — es informacion que
   el proyecto no tiene.
"""
import time

from atriz import Robot

with Robot() as robot:
    print('🔴 Voy a emitir Y a huir de quien oiga. CONDUZCO SOLO.')
    print('   Espacio despejado. Ctrl-C para parar.\n')

    # Emitir primero: si todos huyen y nadie emite, no pasa nada.
    #
    # ⏳ Y AQUI HAY UNA SUPOSICION SIN COMPROBAR, que conviene que sepas porque
    #    es el ejercicio de verdad: NO ESTA DOCUMENTADO EN NINGUNA PARTE si el
    #    firmware del RVR admite EMITIR Y EVADIR A LA VEZ. Se buscó en el SDK
    #    entero el 2026-08-11 y no lo dice ni el codigo ni la documentacion de
    #    Sphero.
    #
    #    Si al ejecutarlo los robots NO se separan, la primera sospecha no debe
    #    ser tu programa: puede ser que el segundo modo apague al primero. Se
    #    comprueba mirando `modo` en /estado_ir despues de arrancar los dos.
    robot.emitir_como_baliza(0, 1)
    time.sleep(0.5)
    robot.huir_de_otro(0, 1)

    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    print('\nParando el IR...')
