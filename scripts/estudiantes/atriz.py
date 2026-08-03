#!/usr/bin/env python3
"""La biblioteca del laboratorio Atriz — lo que usan las prácticas del curso.

    from atriz import Robot

    with Robot() as robot:
        robot.avanzar(0.20, 3)      # m/s durante segundos
        robot.girar(90)             # grados; positivo = a la izquierda

No hace falta instalar nada: este fichero vive junto a los scripts.

═══════════════════════════════════════════════════════════════════════════════
POR QUÉ EXISTE, EN UNA LÍNEA
═══════════════════════════════════════════════════════════════════════════════
Un programa escrito contra `rclpy` a pelo tiene que acertar, cada vez y sin
ayuda, en siete cosas que este laboratorio ha aprendido a base de fallos. Aquí
se aciertan una vez, y el alumno escribe robótica.

Están documentadas una a una en `03_operacion/API_LABORATORIO.md`.
"""
import math
import signal
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy)
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import BatteryState, LaserScan
from std_msgs.msg import Empty
from std_srvs.srv import Empty as EmptySrv

# ═══════════════════════════════════════════════════════════════════════════
# CONSTANTES — cada una tiene una medida detrás. No se cambian sin otra.
# ═══════════════════════════════════════════════════════════════════════════

# 🔴 EL TOPIC. `/cmd_vel` es la SALIDA del collision_monitor: publicar ahí
#    FUNCIONA y salta la capa de seguridad entera, sin un solo aviso. Es el
#    agujero más silencioso del sistema, y los diez scripts de ROS 1 lo hacían.
TOPIC_MANDO = '/cmd_vel_raw'

VEL_MAX = 0.40        # m/s — meseta REAL medida: 0.401 comandando 0.40 (2026-07-31)
VEL_GIRO_MAX = 2.0    # rad/s — 99-102 % del comandado en las cuatro medidas
TIEMPO_MAX = 10.0     # s por llamada — decisión de diseño, no una medida
GRADOS_MAX = 720.0    # ° por llamada — ídem

# 🔴 El watchdog del driver corta a los 0.3 s sin `cmd_vel`. Un `sleep(3)` entre
#    dos publicaciones deja al robot PARADO casi todo el tiempo, y el alumno ve
#    un robot que «no obedece». Hay que republicar más rápido que eso.
RITMO_HZ = 10.0


class ErrorAtriz(Exception):
    """Algo del laboratorio no está como debería. El mensaje dice qué hacer."""


# ═══════════════════════════════════════════════════════════════════════════
# FUNCIONES PURAS — sin ROS, sin robot. Tienen tests en atriz_migracion.
# ═══════════════════════════════════════════════════════════════════════════

def limitar(valor, tope, nombre, unidad):
    """Recorta `valor` a ±`tope`. Devuelve (valor, aviso o None).

    Recorta en vez de lanzar, y AVISA en vez de recortar en silencio: un
    programa que se muere a mitad deja el robot conduciendo, y uno que recorta
    calladito enseña al alumno que su número se aplicó.
    """
    if abs(valor) <= tope:
        return valor, None
    recortado = math.copysign(tope, valor)
    return recortado, (
        f'AVISO: {nombre} {valor:g} {unidad} pasa del limite del laboratorio '
        f'({tope:g} {unidad}); se usa {recortado:g}.')


def normalizar(rad):
    """Lleva un ángulo al intervalo (−π, π]."""
    angulo = math.fmod(rad, 2.0 * math.pi)
    if angulo > math.pi:
        angulo -= 2.0 * math.pi
    elif angulo <= -math.pi:
        angulo += 2.0 * math.pi
    return angulo


def acumular(yaw_anterior, yaw_actual, acumulado):
    """Suma el INCREMENTO de rumbo, normalizado. Nunca el yaw absoluto.

    🔴 `atan2` devuelve −π..π, así que una vuelta entera leída en absoluto
       vuelve al punto de partida y se lee como 0°. Acumular el incremento
       normalizado es lo que hace que 360° sean 360°.
    """
    return acumulado + normalizar(yaw_actual - yaw_anterior)


def yaw_de_cuaternion(x, y, z, w):
    """El rumbo (giro alrededor de Z) de un cuaternión de ROS, en radianes."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def alcanzado(acumulado, objetivo_rad):
    """¿Se llegó al objetivo? Con signo: girar(−90) termina en −π/2.

    Comparar valores absolutos daría por buena una vuelta en el sentido
    contrario, que es exactamente el fallo que no se vería en un pasillo.
    """
    if objetivo_rad >= 0.0:
        return acumulado >= objetivo_rad
    return acumulado <= objetivo_rad


def velocidad_giro(restante_rad):
    """Rad/s para lo que queda de giro: rápido lejos, lento cerca.

    Es la rampa que hace que el lazo cerrado no se pase de largo. El signo lo
    pone quien llama, no esta función.
    """
    restante = abs(restante_rad)
    if restante > math.radians(30.0):
        return 0.80
    if restante > math.radians(8.0):
        return 0.40
    return 0.20


# ═══════════════════════════════════════════════════════════════════════════
# EL ROBOT
# ═══════════════════════════════════════════════════════════════════════════

class Robot:
    """El robot del laboratorio. Se conecta al construirlo.

        with Robot() as robot:
            robot.avanzar(0.20, 3)

    Usa `with`: así el robot se para y el barrido se apaga aunque tu programa
    falle a la mitad.
    """

    def __init__(self, velocidad_maxima=VEL_MAX):
        self._vel_max = min(abs(float(velocidad_maxima)), VEL_MAX)
        self._cerrado = False

        # 🔴 signal_handler_options=NO, Y NO ES OPCIONAL.
        #    `rclpy.init()` instala SU manejador de SIGINT e invalida su propio
        #    contexto: el `except KeyboardInterrupt` que intenta parar el robot
        #    muere con «publisher's context is invalid». Medido el 2026-08-02:
        #    0 lineas de parada con el defecto, 5 con esta opcion. Y ES
        #    INTERMITENTE, que es lo que lo hizo pasar desapercibido.
        if not rclpy.ok():
            rclpy.init(args=None,
                       signal_handler_options=SignalHandlerOptions.NO)

        self._nodo = Node('atriz_alumno')

        # QoS de la telemetria: BEST_EFFORT. Un suscriptor RELIABLE NO RECIBE
        # NADA, sin error — DDS no empareja. Es la misma trampa de QoS que costo
        # la parada de emergencia.
        sensor = QoSProfile(depth=10,
                            reliability=QoSReliabilityPolicy.BEST_EFFORT)
        # La bateria es la excepcion: el driver la publica RELIABLE +
        # TRANSIENT_LOCAL cada 30 s. Pidiendo lo mismo, el ultimo valor llega al
        # suscribirse en vez de esperar medio minuto.
        latch = QoSProfile(depth=1,
                           reliability=QoSReliabilityPolicy.RELIABLE,
                           durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self._odom = None
        self._scan = None
        self._bateria = None
        self._nodo.create_subscription(
            Odometry, '/odom', lambda m: setattr(self, '_odom', m), sensor)
        self._nodo.create_subscription(
            LaserScan, '/scan', lambda m: setattr(self, '_scan', m), sensor)
        self._nodo.create_subscription(
            BatteryState, '/battery_state',
            lambda m: setattr(self, '_bateria', m), latch)

        self._pub_mando = self._nodo.create_publisher(Twist, TOPIC_MANDO, 1)
        # La parada: RELIABLE + VOLATILE. Es el QoS que costo el tercer fallo de
        # este boton — TRANSIENT_LOCAL en el suscriptor solo RESTRINGE.
        self._pub_parada = self._nodo.create_publisher(
            Empty, '/emergency_stop',
            QoSProfile(depth=1,
                       reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.VOLATILE))

        self._cli_iniciar = self._nodo.create_client(EmptySrv, '/start_scan')
        self._cli_parar_barrido = self._nodo.create_client(EmptySrv, '/stop_scan')
        self._cli_param = self._nodo.create_client(
            GetParameters, '/rvr_driver/get_parameters')

        # 🔴 EJECUTOR PROPIO Y PERSISTENTE, en un hilo de fondo.
        #    `rclpy.spin_once(nodo)` en bucle engancha y desengancha el nodo del
        #    ejecutor global en cada llamada, y en ese hueco se PIERDEN mensajes:
        #    11.3 Hz medidos sobre un robot que iba a 16.5.
        #    Y por eso este fichero NO llama a `rclpy.spin_*` en ningun sitio:
        #    mezclarlo con un ejecutor propio ya costo una hora de diagnostico
        #    falso en este proyecto.
        self._ejecutor = SingleThreadedExecutor()
        self._ejecutor.add_node(self._nodo)
        self._hilo = threading.Thread(target=self._ejecutor.spin, daemon=True,
                                      name='atriz-ejecutor')
        self._hilo.start()

        signal.signal(signal.SIGINT, self._al_ctrl_c)

        print('Conectando con el robot...')
        try:
            self._encender_barrido()
            self.hay_color = self._comprobar_color()
        except Exception:
            # 🔴 Si esto revienta, `/start_scan` puede haber tenido EXITO ya
            #    (el motor del X2 esta a 11.8 Hz) y solo fallo esperar el
            #    `/scan` de verdad. Una excepcion aqui sale de `__init__` ANTES
            #    de que Python entre en el `with`: `__enter__` no se llama, y
            #    por tanto `__exit__`/`cerrar()` TAMPOCO. Sin este try, el
            #    barrido se queda girando para siempre — exactamente lo que
            #    `cerrar()` dice evitar en su comentario, y que aqui SI pasaba.
            #    Reproducido y verificado el 2026-08-02: ver la Ronda de
            #    arreglo 2 en la evidencia de la tarea 2.
            self.cerrar()
            raise
        print('Robot listo.')

    # ── Puertas de entrada y salida ─────────────────────────────────────────
    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.cerrar()
        return False

    def cerrar(self):
        """Para el robot y apaga el barrido. Se puede llamar dos veces."""
        if self._cerrado:
            return
        self._cerrado = True
        # 🔴 DOS `try` separados, a proposito. Parar el robot y apagar el
        #    barrido NO son un solo paso: si `_mandar()` revienta (por
        #    ejemplo publicando sobre un contexto ya invalido), un unico
        #    `try` habria dejado SIN EJECUTAR la llamada a `/stop_scan` de
        #    abajo. El barrido se tiene que intentar apagar pase lo que pase
        #    con la parada de velocidad.
        try:
            self._mandar(0.0, 0.0, repeticiones=5)
        except Exception as e:                               # noqa: BLE001
            print(f'AVISO al parar: {e}')
        try:
            self._llamar(self._cli_parar_barrido, EmptySrv.Request(),
                         timeout=5.0, que='/stop_scan')
        except Exception as e:                               # noqa: BLE001
            print(f'AVISO al cerrar: {e}')
        finally:
            # El barrido se apaga SIEMPRE que se pueda: si no, el X2 se queda
            # girando a 11.8 Hz en vez de 2.7, 24/7 y por 16 robots.
            self._ejecutor.shutdown()
            # 🔴 `shutdown()` SEÑALA al hilo de `spin()`, no lo ESPERA. Medido el
            #    2026-08-02: el hilo seguia `is_alive()` justo despues de
            #    `shutdown()` en 14/14 corridas (tardaba ~0.5 ms mas en unirse).
            #    Sin este `join()`, `destroy_node()`/`rclpy.shutdown()` se
            #    ejecutaban en paralelo con ese hilo todavia tocando el nodo:
            #    SIGABRT intermitente (2 de cada 3 corridas medidas, sin este
            #    join).
            self._hilo.join(timeout=3.0)
            # 🔴 Si el hilo NO se unio en 3 s, NO se sigue a destroy_node().
            #    Seguir habria sido la MISMA secuencia sin sincronizar que
            #    provocaba el SIGABRT — estrechar la carrera no es cerrarla.
            #    Lo importante para la seguridad ya se intento arriba (cero
            #    velocidad + /stop_scan), asi que se prefiere dejar el nodo
            #    sin destruir (fuga de recursos DDS que el SO recupera al
            #    terminar el proceso) antes que arriesgar el abort. 3 s es
            #    generoso frente a lo medido (uniones reales de ~0.5 ms); si
            #    esto se dispara, es señal de que algo va realmente mal.
            if self._hilo.is_alive():
                print('AVISO: el hilo del ejecutor no se unio en 3 s. '
                      'NO se destruye el nodo para no reproducir la carrera '
                      'que provocaba el SIGABRT (el robot ya recibio orden '
                      'de parar y de apagar el barrido, por encima).')
                return
            self._nodo.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    def _al_ctrl_c(self, _signum, _frame):
        """Ctrl-C: para el robot y sale. NO dispara la parada de emergencia.

        🔴 Y es a proposito. La parada SE QUEDA ENGANCHADA hasta que alguien
           llame a /release_emergency_stop, asi que un Ctrl-C que la disparara
           dejaria el SIGUIENTE script del alumno sin funcionar y sin
           explicacion. El camino correcto para terminar es el normal: cero, y
           el watchdog de 0.3 s por debajo.
        """
        print('\nCtrl-C: parando el robot...')
        self.cerrar()
        sys.exit(130)

    # ── Arranque ────────────────────────────────────────────────────────────
    def _encender_barrido(self):
        """🔴 Sin /scan el robot NO OBEDECE, y parece averiado.

        El barrido arranca apagado a proposito (si no, el X2 gira a 11.8 Hz
        24/7). Sin `/scan` el collision_monitor bloquea el movimiento: medido
        0.0 cm contra 9.9 del control. Desde fuera es identico a un robot roto.
        """
        self._llamar(self._cli_iniciar, EmptySrv.Request(),
                     timeout=10.0, que='/start_scan')
        # Que el servicio conteste no prueba que lleguen barridos: se espera al
        # EFECTO, que es un /scan de verdad.
        self._ultimo('_scan', timeout=8.0, que='/scan')

    def _comprobar_color(self):
        """¿Se arranco el robot con el sensor de color encendido?

        🔴 No se puede encender bajo demanda: con el streaming ya montado,
           `enable_color_detection` NO HACE NADA (481 mensajes, todos ceros). Se
           decide en el arranque, y el servicio systemd usa el defecto: false.
           Sin esta comprobacion, `color()` devuelve negro y parece un dato.
        """
        peticion = GetParameters.Request()
        peticion.names = ['color_detection']
        try:
            resp = self._llamar(self._cli_param, peticion, timeout=5.0,
                                que='/rvr_driver/get_parameters')
            activo = bool(resp.values[0].bool_value)
        except Exception:                                    # noqa: BLE001
            print('AVISO: no se pudo consultar color_detection. '
                  'color() puede devolver ceros.')
            return False
        if not activo:
            print('AVISO: el sensor de color esta APAGADO en este robot.\n'
                  '       color() devolvera ceros. Para usarlo, el robot tiene\n'
                  '       que arrancar asi (lo hace el profesor):\n'
                  '         sudo systemctl stop atriz-robot\n'
                  '         ros2 launch atriz_rvr_bringup robot.launch.py '
                  'color_detection:=true')
        return activo

    # ── Fontaneria ──────────────────────────────────────────────────────────
    def _llamar(self, cliente, peticion, timeout, que):
        """Llama a un servicio y espera SONDEANDO el futuro.

        🔴 No se usa `rclpy.spin_until_future_complete`: este nodo ya esta en un
           ejecutor propio, y mezclarlos deja de atender las suscripciones. Aqui
           el futuro lo completa el hilo de fondo; este solo mira si ya esta.
        """
        if not cliente.wait_for_service(timeout_sec=timeout):
            raise ErrorAtriz(
                f'{que} no aparece. Comprueba que el robot esta encendido:\n'
                f'  systemctl is-active atriz-robot')
        futuro = cliente.call_async(peticion)
        limite = time.monotonic() + timeout
        while not futuro.done() and time.monotonic() < limite:
            time.sleep(0.01)
        if not futuro.done():
            raise ErrorAtriz(f'{que} no contesto en {timeout:.0f} s.')
        return futuro.result()

    def _ultimo(self, atributo, timeout, que):
        """El ultimo mensaje recibido de un topic, esperando si aun no llego."""
        limite = time.monotonic() + timeout
        while getattr(self, atributo) is None and time.monotonic() < limite:
            time.sleep(0.02)
        mensaje = getattr(self, atributo)
        if mensaje is None:
            raise ErrorAtriz(
                f'no llega nada por {que} en {timeout:.0f} s. El topic puede '
                f'existir y estar mudo: mira el RITMO, no la lista de topics.')
        return mensaje

    def mover(self, velocidad, giro):
        """Manda UNA orden de velocidad y vuelve enseguida. Para lazos de control.

        📝 A diferencia de `avanzar()`, esta NO bloquea ni republica: la tienes
           que llamar tu en tu bucle, MAS DE TRES VECES POR SEGUNDO. Si no, el
           watchdog del driver corta a los 0.3 s y el robot ira a tirones.
        """
        velocidad, aviso = limitar(velocidad, self._vel_max, 'velocidad', 'm/s')
        if aviso:
            print(aviso)
        giro, aviso = limitar(giro, VEL_GIRO_MAX, 'giro', 'rad/s')
        if aviso:
            print(aviso)
        orden = Twist()
        orden.linear.x = float(velocidad)
        orden.angular.z = float(giro)
        self._pub_mando.publish(orden)

    def _mandar(self, lineal, angular, repeticiones=1):
        """Publica una velocidad en cmd_vel_raw, con el ritmo del watchdog.

        Se apoya en `mover()` para que exista UNA sola regla de construccion de
        la orden. Lo que anade es el RITMO: republicar a RITMO_HZ, que es lo que
        el watchdog de 0.3 s del driver necesita.
        """
        for _ in range(repeticiones):
            self.mover(lineal, angular)
            time.sleep(1.0 / RITMO_HZ)
