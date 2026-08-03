#!/usr/bin/env python3
"""Prueba 99 - ¿Para el robot cuando pulsas Ctrl-C?

    python3 99_test_ctrl_c.py

⚠️ MUEVE EL ROBOT. Necesita al menos 1.5 m despejados por delante:
avanzar(0.15, 8) son 0.15 m/s x 8 s = 1.2 m si nadie pulsa Ctrl-C, y esta
prueba contempla EXPRESAMENTE que no lo pulses (imprime «Llegue al final
sin Ctrl-C»). Pedir justo 1.2 m dejaria CERO margen para lo que el robot
recorre despues de la ultima orden — que es justo lo que esta prueba
existe para medir. 1.5 m deja ~30 cm para eso.

Y despejados a la altura del LIDAR (15.5 cm desde el suelo), no solo a ras
de suelo: es lo que usa el collision_monitor para frenar, y un obstaculo
bajo que el LIDAR SI ve podria parar el robot antes de que llegues a
probar el Ctrl-C, confundiendo la prueba.

═══════════════════════════════════════════════════════════════════════════════
POR QUE ESTA PRUEBA EXISTE Y NO ES UNA CURIOSIDAD
═══════════════════════════════════════════════════════════════════════════════
En este laboratorio, Ctrl-C YA FALLO. `rclpy.init()` instala su propio manejador
de la senal e invalida su contexto: el codigo que intenta parar el robot muere
con «publisher's context is invalid». Medido: 0 lineas de parada con el defecto,
5 con la opcion correcta.

Y el fallo es INTERMITENTE — segun donde caiga el Ctrl-C, a veces si para. Por
eso esta prueba se corre VARIAS VECES: una pasada verde sobre un fallo
intermitente es indistinguible de que no haya fallo.

Debajo, el driver tiene un watchdog que corta a los 0.3 s sin recibir ordenes.
Asi que aunque mataras el programa a lo bruto, el robot se para. Ctrl-C es el
primer cinturon; el watchdog, el segundo.

🔴 PERO EL WATCHDOG ES SOLO DE LOS MOTORES. Del LIDAR no sabe nada. Son dos
   cosas distintas y conviene tenerlo claro antes del ejercicio 3:

     senal            ¿la para el robot?   ¿apaga el barrido del LIDAR?
     ---------------  ------------------   ---------------------------
     Ctrl-C (SIGINT)  si (cierre limpio)   SI  <- atriz.py lo captura
     cerrar terminal  si (cierre limpio)   SI  <- SIGHUP, capturado
     SSH caido        si (cierre limpio)   SI  <- SIGHUP, capturado
     kill (SIGTERM)   si (cierre limpio)   SI  <- capturado
     kill -9          si, por el watchdog  NO  <- no se puede capturar
     sin corriente    si, no hay corriente NO

   Las cuatro primeras las captura `atriz.py` y llama a /stop_scan. Las dos
   ultimas NO SE PUEDEN capturar desde Python: ahi el barrido se queda
   encendido (11.8 Hz en vez de 2.7, indefinidamente) y solo lo apaga el
   profesor con `atriz-escaneo off`.

   🔴 Esta tabla es de MECANISMO (que codigo corre), no de MEDIDA: lo de
      «apaga el barrido» esta comprobado provocando las señales sobre un
      proceso, sin robot. Cuanto recorre el robot despues de cada una NO ESTA
      MEDIDO — es lo que vas a medir tu.
"""
from atriz import Robot

print(__doc__)
with Robot() as robot:
    input('Marca la posicion del robot y pulsa Enter...')
    print('Avanzando 8 s. Pulsa Ctrl-C cuando quieras y MIDE cuanto recorre')
    print('el robot DESPUES de que lo pulses.\n')
    robot.avanzar(0.15, 8)
    print('Llegue al final sin que pulsaras Ctrl-C.')

# EJERCICIOS
#   1. Repitelo cinco veces. ¿Paro las cinco?
#   2. Mide el recorrido posterior. ¿Cuanto varia entre corridas?
#   3. Pulsa Ctrl-C y, sin esperar, pulsalo OTRA VEZ enseguida. ¿Que imprime?
#      (deberia decirte que ya esta cerrando y que esperes). ¿Por que crees que
#      la biblioteca IGNORA la segunda pulsacion en vez de salir mas rapido?
#   4. Prueba a cerrar la terminal en vez de pulsar Ctrl-C. ¿Para el robot?
#      ¿Y se apaga el barrido del LIDAR? Comprueba las DOS cosas por separado:
#        - que para -> miralo, o mide el recorrido con la cinta
#        - que el barrido se apago -> `atriz-escaneo estado`
#      Pista: son dos mecanismos distintos. Mira la tabla de la cabecera y di
#      cual de los dos actua en cada caso. 🔴 El watchdog de 0.3 s NO es la
#      respuesta a la segunda pregunta: es de los motores.
#   5. Repite el 4 con `kill -9 <pid>` desde otra terminal. Ahora la respuesta
#      a las dos preguntas es distinta. ¿Cual cambia, y por que no se puede
#      arreglar desde el programa?
