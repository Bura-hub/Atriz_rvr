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
import atexit
import math
import signal
import sys
import threading
import time

import rclpy
from atriz_rvr_msgs.srv import GetRGBCSensorValues, SetLeds
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

# 🔴 TODAS las señales que terminan un programa y SE PUEDEN capturar. Con solo
#    SIGINT, cerrar la terminal (SIGHUP) o perder el SSH dejaba el barrido del
#    X2 girando a 11.8 Hz para siempre: el watchdog de 0.3 s del driver para los
#    MOTORES, pero no hay nada que apague el LIDAR. Verificado provocando las
#    tres señales sobre un proceso real (bloque A del informe final).
#    ⚠️ SIGKILL (`kill -9`) y un corte de corriente NO se pueden capturar: en
#       esos dos casos el barrido se queda encendido y solo lo apaga el
#       profesor con `atriz-escaneo off`. Está dicho así en 00_LEEME_PRIMERO.md.
SENALES_DE_CIERRE = tuple(
    senal for senal in (getattr(signal, nombre, None)
                        for nombre in ('SIGINT', 'SIGTERM', 'SIGHUP'))
    if senal is not None)


class ErrorAtriz(Exception):
    """Algo del laboratorio no está como debería. El mensaje dice qué hacer."""


# ═══════════════════════════════════════════════════════════════════════════
# FUNCIONES PURAS — sin ROS, sin robot. Tienen tests en atriz_migracion.
# ═══════════════════════════════════════════════════════════════════════════

def secuencia_de_cierre(parar, apagar_barrido, desmontar, avisar=print):
    """Los tres pasos del cierre, y los tres se INTENTAN pase lo que pase.

    Es el corazón de la promesa de esta biblioteca: *el barrido del LIDAR se
    apaga siempre*. Vive aquí, separada de ROS, para que se pueda probar
    provocando los fallos de verdad en vez de razonarlos.

    🔴 Cada paso cuelga del `finally` del anterior. Antes había DOS `try` y un
       solo `finally`: si `parar()` reventaba, `/stop_scan` no se llamaba.

    🔴 Y se captura `BaseException`, no `Exception`. `SystemExit` y
       `KeyboardInterrupt` NO son `Exception`: el SEGUNDO Ctrl-C entra por el
       manejador de señal y lanza `SystemExit` desde dentro de este cierre.
       Con `except Exception` esa excepción se llevaba por delante los pasos
       que faltaban — `/stop_scan` no se llegaba a llamar y el X2 se quedaba
       girando a 11.8 Hz en vez de 2.7, indefinidamente. Verificado
       provocándolo, no razonándolo.
    """
    try:
        parar()
    except BaseException as e:                               # noqa: BLE001
        avisar(f'AVISO al parar el robot: {e!r}')
    finally:
        try:
            apagar_barrido()
        except BaseException as e:                           # noqa: BLE001
            avisar(f'AVISO al apagar el barrido del LIDAR: {e!r}')
        finally:
            try:
                desmontar()
            except BaseException as e:                       # noqa: BLE001
                avisar(f'AVISO al soltar los recursos de ROS: {e!r}')


def limitar(valor, tope, nombre, unidad):
    """Recorta `valor` a ±`tope`. Devuelve (valor, aviso o None).

    Recorta en vez de lanzar, y AVISA en vez de recortar en silencio: un
    programa que se muere a mitad deja el robot conduciendo, y uno que recorta
    calladito enseña al alumno que su número se aplicó.

    🔴 PERO `NaN` e `inf` NO se recortan: se RECHAZAN con `ErrorAtriz`.
       `math.copysign(tope, nan)` devuelve el TOPE, así que la función cuyo
       trabajo es «llevar a un valor seguro» convertía la entrada menos fiable
       de todas en la velocidad MÁXIMA — y combinada con el tope de tiempo,
       en la distancia máxima: `avanzar(nan, nan)` conducía 0.40 m/s durante
       10.0 s = 4.0 METROS. Un `NaN` no es «un número demasiado grande» que
       tenga sentido recortar: es la señal de que el cálculo del alumno se
       rompió antes de llegar aquí, y seguir conduciendo con él es lo peor
       que se puede hacer.
       📝 Mismo criterio que `aceptacion_nucleo.delta_angulo()`, que ya
          rechaza lo no finito antes de normalizar: se falla alto y con una
          excepción que el llamador puede distinguir de un valor válido.
    """
    if isinstance(valor, bool) or not isinstance(valor, (int, float)):
        raise ErrorAtriz(
            f'{nombre}={valor!r}: tiene que ser un numero, no '
            f'{type(valor).__name__}.')
    if not math.isfinite(valor):
        raise ErrorAtriz(
            f'{nombre}={valor:g}: no es un numero finito. NO se recorta al '
            f'limite ({tope:g} {unidad}) a proposito — recortar un NaN o un '
            f'infinito daria justo el valor MAXIMO. Mira de donde sale ese '
            f'valor: casi siempre es una division por cero o una resta de '
            f'lecturas que aun no habian llegado.')
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


def validar_canal_led(valor, nombre):
    """Un canal de LED válido: entero, y de 0 a 255. Si no, lanza ErrorAtriz.

    🔴 Valida el valor TAL COMO LLEGA, no su truncamiento. `int(True) == 1` y
       `int(-0.5) == 0` son los dos ENTEROS validos: si se comprueba el rango
       DESPUES de convertir a int, `luces(True, True, True)` pasa como
       RGB (1,1,1) —practicamente apagado— sin avisar, porque en Python
       `bool` es subclase de `int`. Es el mismo error de tipo que confunde
       "encendido/apagado" con 0/1 en un curso de 16 h.
    """
    if isinstance(valor, bool) or not isinstance(valor, int):
        raise ErrorAtriz(
            f'{nombre}={valor!r}: tiene que ser un entero (int) de 0 a 255, '
            f'no {type(valor).__name__}.')
    if not 0 <= valor <= 255:
        raise ErrorAtriz(f'{nombre}={valor}: cada canal va de 0 a 255.')


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
        self._cerrando = False

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
        self._cli_color = self._nodo.create_client(
            GetRGBCSensorValues, '/get_rgbc_sensor_values')
        self._cli_luces = self._nodo.create_client(SetLeds, '/set_leds')

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

        # 🔴 LAS TRES SEÑALES, no solo SIGINT. Cerrar la terminal manda SIGHUP
        #    y `systemctl stop`/`kill` mandan SIGTERM: con el manejador solo en
        #    SIGINT, los dos mataban el proceso sin pasar por `cerrar()` y el
        #    barrido del X2 se quedaba encendido. El watchdog de 0.3 s del
        #    driver para los MOTORES; del LIDAR no sabe nada.
        for senal in SENALES_DE_CIERRE:
            signal.signal(senal, self._al_senal)
        # 🔴 Y la ultima red: salida normal SIN `with`, `sys.exit()`, o una
        #    excepcion que llega arriba del todo. En esos tres casos no hay
        #    `__exit__` ni señal, y hasta ahora tampoco habia cierre.
        atexit.register(self.cerrar)

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
        """Para el robot y apaga el barrido. Se puede llamar dos veces.

        El orden y la garantía viven en `secuencia_de_cierre()`, que es pura y
        tiene tests que PROVOCAN cada fallo (incluido el segundo Ctrl-C).
        """
        if self._cerrado:
            return
        self._cerrado = True
        # 🔴 Mientras dure el cierre, las señales se IGNORAN: es lo que impide
        #    que un segundo Ctrl-C aborte el apagado del barrido.
        self._cerrando = True
        try:
            secuencia_de_cierre(
                parar=lambda: self._mandar(0.0, 0.0, repeticiones=5),
                apagar_barrido=self._apagar_barrido,
                desmontar=self._desmontar)
        finally:
            self._cerrando = False

    def _apagar_barrido(self):
        """/stop_scan. Si no se llama, el X2 se queda girando a 11.8 Hz en vez
        de 2.7, 24/7 y multiplicado por 16 robots."""
        self._llamar(self._cli_parar_barrido, EmptySrv.Request(),
                     timeout=5.0, que='/stop_scan')

    def _desmontar(self):
        """Suelta el ejecutor, el hilo y el nodo. Va SIEMPRE el ultimo: lo que
        importa para la seguridad ya se intento antes."""
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
        #    Se prefiere dejar el nodo sin destruir (fuga de recursos DDS que
        #    el SO recupera al terminar el proceso) antes que arriesgar el
        #    abort. Esta rama esta escrita y NO EJERCITADA: asi consta.
        if self._hilo.is_alive():
            print('AVISO: el hilo del ejecutor no se unio en 3 s. '
                  'NO se destruye el nodo para no reproducir la carrera '
                  'que provocaba el SIGABRT (el robot ya recibio orden '
                  'de parar y de apagar el barrido, por encima).')
            return
        self._nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def _al_senal(self, signum, _frame):
        """SIGINT (Ctrl-C), SIGTERM (`kill`) y SIGHUP (cerrar la terminal o
        perder el SSH): para el robot, apaga el barrido y sale.

        🔴 NO dispara la parada de emergencia, a proposito. La parada SE QUEDA
           ENGANCHADA hasta que alguien llame a /release_emergency_stop, asi que
           un Ctrl-C que la disparara dejaria el SIGUIENTE script del alumno sin
           funcionar y sin explicacion. El camino correcto para terminar es el
           normal: cero, y el watchdog de 0.3 s por debajo.

        🔴 Y si YA se esta cerrando, esta señal se IGNORA. Antes hacia
           `sys.exit(130)` incondicionalmente: en un segundo Ctrl-C, `cerrar()`
           reentraba, salia por `if self._cerrado: return`, y el `SystemExit`
           escapaba desde dentro del cierre a medias — `/stop_scan` NO se
           llamaba. Interrumpir un cierre en curso no adelanta nada (dura como
           mucho unos segundos) y deja el LIDAR encendido.
        """
        nombre = getattr(signal.Signals(signum), 'name', str(signum))
        if self._cerrando:
            print(f'\n{nombre} otra vez: YA estoy cerrando, espera. '
                  f'Interrumpir ahora dejaria el barrido del LIDAR encendido.')
            return
        print(f'\n{nombre}: parando el robot y apagando el barrido...')
        self.cerrar()
        sys.exit(128 + signum)

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

    # ── Movimiento ──────────────────────────────────────────────────────────
    def avanzar(self, velocidad, segundos):
        """Avanza a `velocidad` m/s durante `segundos`.

            robot.avanzar(0.20, 3)     # 20 cm/s durante 3 segundos
            robot.avanzar(-0.20, 3)    # hacia ATRAS 3 segundos

        🔴 Lo que hace ir hacia atras es la VELOCIDAD negativa, no el tiempo.
           El tiempo pasa por `abs()`, asi que `avanzar(0.20, -3)` avanza hacia
           DELANTE 3 segundos — no retrocede ni va «menos tres segundos». Si
           quieres retroceder, el signo va en el primer argumento.

        ⚠️ Y hacia atras no hay capa de seguridad: el poligono del
           collision_monitor mira hacia DELANTE. Ademas frena igual aunque el
           robot se este alejando del obstaculo (medido: 30 cm comandados hacia
           atras dieron 14 cm), asi que un retroceso puede quedarse corto Y no
           avisar de lo que tiene detras.

        Republica la orden a 10 Hz: el driver corta a los 0.3 s sin recibir
        nada, asi que un `sleep` largo entre publicaciones deja al robot
        parandose y arrancando.
        """
        velocidad, aviso = limitar(velocidad, self._vel_max, 'velocidad', 'm/s')
        if aviso:
            print(aviso)
        segundos, aviso = limitar(abs(segundos), TIEMPO_MAX, 'tiempo', 's')
        if aviso:
            print(aviso)

        limite = time.monotonic() + segundos
        while time.monotonic() < limite:
            self._mandar(velocidad, 0.0)
        self.parar()

    def parar(self):
        """Para el robot: velocidad cero, repetida por si se pierde un mensaje."""
        self._mandar(0.0, 0.0, repeticiones=5)

    def rumbo(self):
        """El rumbo actual en grados, leido de /odom."""
        q = self._ultimo('_odom', timeout=5.0, que='/odom').pose.pose.orientation
        return math.degrees(yaw_de_cuaternion(q.x, q.y, q.z, q.w))

    def girar(self, grados):
        """Gira `grados` sobre el eje. Positivo = a la IZQUIERDA (REP-103).

            robot.girar(90)      # un cuarto de vuelta a la izquierda
            robot.girar(-90)     # a la derecha

        Devuelve los grados que giro DE VERDAD, medidos en /odom.

        ═══════════════════════════════════════════════════════════════════════
        POR QUE ESTO ES UN LAZO CERRADO Y NO UNA CONSTANTE
        ═══════════════════════════════════════════════════════════════════════
        Girar por tiempo es un lazo ABIERTO: mandas la orden y confias. Este
        robot tiene un deficit medido haciendolo asi — 90 grados pedidos dieron
        86.6 / 86.2 / 87.7 (n=3, evidencia 48).

        🔴 PERO ESOS TRES NUMEROS NO SON LOS DE `girar_por_tiempo()`, y decir
           que lo eran era atribuir mal una medida. Se tomaron con el servicio
           `move_timed` del driver, a 1.0 rad/s. `girar_por_tiempo()` es OTRO
           mecanismo (publica en /cmd_vel_raw a 20 Hz) y va a OTRA velocidad
           (0.8 rad/s en la practica 4): otro camino, otra velocidad, otro
           numero. **Cuanto se queda corto ESTE camino NO ESTA MEDIDO.**
           Lo que los 86.6/86.2/87.7 si demuestran es que el deficit del lazo
           abierto EXISTE en este robot y es de varios grados — y esa es toda
           la razon que hace falta para cerrar el lazo.

        La salida barata seria multiplicar por una constante. Aqui se mide el
        rumbo real y se para al llegar: la constante se equivoca en cuanto
        cambie el suelo, la carga o el robot; el lazo, no.

        ⚠️ Con que precision acierta este lazo TAMPOCO esta medido con
           transportador. Lo unico que se puede afirmar es aritmetica: el lazo
           comprueba el rumbo cada 0.05 s, y en el ultimo tramo de la rampa va
           a 0.20 rad/s, asi que solo la granularidad del bucle ya son
           0.20 x 0.05 = 0.01 rad = 0.573 grados. A eso hay que sumarle lo que
           el robot siga rodando tras la orden de parada, que NADIE HA MEDIDO.
           No digas «unas decimas»: dilo cuando lo midas.

        🔴 Y se acumula el INCREMENTO de rumbo, nunca el yaw absoluto: `atan2`
           devuelve -pi..pi, asi que una vuelta entera leida en absoluto vuelve
           al punto de partida y `girar(360)` terminaria sin haberse movido.
        """
        grados, aviso = limitar(grados, GRADOS_MAX, 'giro', 'grados')
        if aviso:
            print(aviso)
        objetivo = math.radians(grados)
        if abs(objetivo) < math.radians(0.5):
            return 0.0

        sentido = 1.0 if objetivo >= 0.0 else -1.0
        anterior = math.radians(self.rumbo())
        acumulado = 0.0

        # Tope de tiempo: lo que tardaria al ritmo mas lento de la rampa, con
        # margen. Sin el, un robot atascado gira para siempre.
        limite = time.monotonic() + abs(objetivo) / 0.20 + 5.0

        # Detectar si /odom se queda congelado: contar iteraciones sin cambio de timestamp
        ultimo_timestamp = None
        sin_cambio = 0
        MAX_SIN_CAMBIO = 5  # ~0.25 s a 20 Hz

        while not alcanzado(acumulado, objetivo):
            if time.monotonic() > limite:
                print(f'AVISO: el giro se quedo en {math.degrees(acumulado):.1f} '
                      f'de {grados:g} grados. Robot atascado o algo lo frena.')
                break

            # 🔴 Publica a 20 Hz (0.05 s), no a 10. Y lo que eso compra es MENOS
            #    de lo que decia este comentario, que afirmaba «para reducir
            #    sobregiro» a secas. Simulado con la rampa real, ya con el
            #    `parar()` del lazo bien modelado (scripts/simular_girar.py):
            #
            #        90°  -> 10 Hz y 20 Hz dan LO MISMO (90.5273). Ventaja: 0.
            #        180° -> 0.573° menos de sobregiro a 20 Hz
            #        360° -> 0.573° menos
            #        720° -> 0.573° menos
            #
            #    🔴 O sea que A 90 GRADOS NO COMPRA NADA — y 90° es el angulo de
            #       las practicas 2, 3, 4 y 10, el unico que el curso usa. La
            #       «ventaja de +0.573° a 90°» que se reportaba antes era un
            #       ARTEFACTO del simulador, que seguia integrando 0.20 rad/s
            #       despues de la orden de parada; ese paso de mas vale
            #       0.20 x dt, o sea el doble a 10 Hz que a 20.
            #
            #    Se deja en 20 Hz igualmente, por dos razones que si se
            #    sostienen: (1) en angulos grandes el sobregiro baja de verdad,
            #    y (2) `girar_por_tiempo()` publica tambien a 20 Hz para que la
            #    practica 4 compare UNA sola variable (el sensor) y no dos.
            #    ⚠️ Y ojo con el limite real: /odom llega a 16.5 Hz, asi que por
            #       encima de eso quien fija el paso ya no es este bucle sino la
            #       odometria. Nada de esto esta medido sobre el robot.
            self.mover(0.0, sentido * velocidad_giro(objetivo - acumulado))
            time.sleep(0.05)

            q = self._ultimo('_odom', timeout=2.0,
                             que='/odom').pose.pose.orientation

            # Comprobar si la muestra de /odom cambio
            timestamp_actual = self._odom.header.stamp
            if timestamp_actual == ultimo_timestamp:
                sin_cambio += 1
                if sin_cambio >= MAX_SIN_CAMBIO:
                    print(f'AVISO: /odom no se actualiza hace ~{MAX_SIN_CAMBIO * 0.05:.2f} s. '
                          f'Odometría perdida o desconectada.')
                    self.parar()
                    return math.degrees(acumulado)
            else:
                sin_cambio = 0
                ultimo_timestamp = timestamp_actual

            actual = yaw_de_cuaternion(q.x, q.y, q.z, q.w)
            acumulado = acumular(anterior, actual, acumulado)
            anterior = actual

        self.parar()
        # El robot sigue rodando un poco tras el ultimo comando: se espera y se
        # vuelve a medir, para devolver lo que paso de verdad y no lo que se
        # habia mandado.
        time.sleep(0.5)
        q = self._ultimo('_odom', timeout=2.0, que='/odom').pose.pose.orientation
        acumulado = acumular(anterior, yaw_de_cuaternion(q.x, q.y, q.z, q.w),
                             acumulado)
        return math.degrees(acumulado)

    def girar_por_tiempo(self, velocidad, segundos):
        """Gira a `velocidad` rad/s durante `segundos`. LAZO ABIERTO.

        📝 Existe SOLO para la practica 4, que compara el lazo abierto con el
           cerrado. Para girar de verdad usa `girar(grados)`: mide el rumbo y
           acierta, y esta no.

        🔴 CUANTO se queda corto ESTA funcion NO ESTA MEDIDO. Los
           86.6 / 86.2 / 87.7 grados que circulan por la documentacion son del
           servicio `move_timed` del driver a 1.0 rad/s (evidencia 48), no de
           esta funcion, que publica en /cmd_vel_raw a 20 Hz. Sirven para saber
           que el deficit del lazo abierto existe y es de varios grados; no
           para predecir lo que va a imprimir la practica 4. El numero de esa
           practica lo pone el alumno con el transportador.

        🔴 Publica al MISMO ritmo que `girar()` (20 Hz: `mover()` + `sleep(0.05)`),
           a proposito y NO con `_mandar()` (10 Hz). Si este lazo publicara a un
           ritmo distinto del cerrado, la practica 4 estaria comparando DOS
           variables a la vez (el sensor Y el ritmo de publicacion) y le
           atribuiria el resultado a una sola — el mismo error de metodo que la
           regla "mide antes de atribuir" de este proyecto existe para evitar.
        """
        velocidad, aviso = limitar(velocidad, VEL_GIRO_MAX, 'giro', 'rad/s')
        if aviso:
            print(aviso)
        segundos, aviso = limitar(abs(segundos), TIEMPO_MAX, 'tiempo', 's')
        if aviso:
            print(aviso)
        limite = time.monotonic() + segundos
        while time.monotonic() < limite:
            self.mover(0.0, velocidad)
            time.sleep(0.05)
        self.parar()

    # ── Sensores ────────────────────────────────────────────────────────────
    def color(self):
        """El color que ve el robot: (rojo, verde, azul, claro).

        📝 Sale del servicio /get_rgbc_sensor_values y no del topic /color, y
           por una razon medida: el mensaje `Color` NO trae el canal `claro`, y
           `claro` es el que discrimina de verdad — 12.6x entre blanco y negro,
           contra un RGB que apenas se mueve. El servicio cuesta 20.6-20.8 ms
           (medido, n=200), asi que cabe de sobra en un lazo de control a
           10 Hz, que tiene 100 ms por vuelta.

        Normaliza por VERDE, que es el canal mas sensible: rojo sube R/G de 0.48
        a 2.74, azul sube B/G a 0.86.

        🔴 Lanza ErrorAtriz si el driver contesta `success=False` — el RVR no
           siempre responde (puerto serie o RVR dormido), y el driver deja los
           cuatro canales en 0 en ese caso. Sin esta comprobacion, (0,0,0,0) es
           INDISTINGUIBLE de una lectura real sobre negro: es el mismo fallo
           que costo meses de /color publicando ceros sin que nadie lo notara.
        """
        if not self.hay_color:
            print('AVISO: el sensor de color esta apagado; los valores seran '
                  'ruido de fondo (los canales oscilan entre 0 y 1, no ceros '
                  'fijos), no una lectura de verdad.')
        r = self._llamar(self._cli_color, GetRGBCSensorValues.Request(),
                         timeout=5.0, que='/get_rgbc_sensor_values')
        if not r.success:
            raise ErrorAtriz(
                f'/get_rgbc_sensor_values fallo de verdad, no es una lectura '
                f'sobre negro: {r.message or "el driver no dio mas detalle"}.')
        return (r.red_channel_value, r.green_channel_value,
                r.blue_channel_value, r.clear_channel_value)

    def distancia_frontal(self):
        """Metros hasta lo mas cercano que hay DELANTE, en un cono de +-10 grados.

        ⚠️ Un solo barrido no ve un objeto fino: a 0.68 m el X2 tira un rayo
           cada 1.7 cm, asi que algo de 5 cm da 2-3 puntos y puede desaparecer.
           Para geometria fina hay que acumular varios barridos.

        🔴 Y «delante» aqui significa «el angulo 0 de /scan», que NO SE HA
           CONTRASTADO NUNCA CON CINTA. El URDF no mete offset angular y el
           `inverted` del X2 se verifico en SENTIDO DE GIRO, nunca en OFFSET DE
           MONTAJE. Si el tambor esta montado girado unos grados, este cono
           mira unos grados a un lado y nada lo delataria: seguiria devolviendo
           distancias perfectamente plausibles. Con un cono de +-10 grados hace
           falta un error grande para que importe, pero el dato no esta.
        """
        barrido = self._ultimo('_scan', timeout=5.0, que='/scan')
        cono = math.radians(10.0)
        cerca = math.inf
        for i, distancia in enumerate(barrido.ranges):
            angulo = barrido.angle_min + i * barrido.angle_increment
            if abs(normalizar(angulo)) > cono:
                continue
            if barrido.range_min < distancia < barrido.range_max:
                cerca = min(cerca, distancia)
        if not math.isfinite(cerca):
            raise ErrorAtriz('no hay ningun punto valido delante del robot.')
        return cerca

    def bateria(self):
        """Voltios de la bateria del RVR.

        🔴 Voltios y no porcentaje: el porcentaje dijo 100 % con la bateria a
           8.29 V, a 1.29 V del umbral de «baja» del propio firmware (7.0 V;
           critica 6.5). Es una estimacion gruesa.
        """
        return float(self._ultimo('_bateria', timeout=35.0,
                                  que='/battery_state').voltage)

    # ── Luces y parada ──────────────────────────────────────────────────────
    def luces(self, rojo, verde, azul):
        """Pone TODOS los faros del robot a un color (0-255 cada canal).

        🔴 ES LA UNICA LLAMADA DE ESTA API SIN FORMA DE DETECTAR EL FALLO.
           `SetLeds.srv` tiene la respuesta VACIA (comprobado: el fichero .srv
           es `int32[] rgb_color` y nada debajo del `---`), asi que el driver no
           tiene por donde decir que no. Lo que si se valida es lo que se puede
           validar aqui: que los tres canales sean enteros de 0 a 255, ANTES de
           convertir — eso lanza `ErrorAtriz`. Pero si la peticion sale bien y
           los faros no se encienden, `luces()` vuelve sin decir nada.

        ⚠️ Y en este firmware eso no es teorico: hay comandos de LED que el RVR
           ACEPTA EN SILENCIO sin hacer nada (`set_all_leds` con una mascara mal
           formada, `undercarriage_white`). La unica comprobacion que vale para
           esta llamada es MIRAR EL ROBOT.
        """
        for nombre, valor in (('rojo', rojo), ('verde', verde), ('azul', azul)):
            validar_canal_led(valor, nombre)
        peticion = SetLeds.Request()
        peticion.rgb_color = [rojo, verde, azul]
        self._llamar(self._cli_luces, peticion, timeout=5.0, que='/set_leds')

    def parada_emergencia(self):
        """Parada de emergencia: el driver descarta TODO comando hasta liberarla.

        🔴 NO se libera sola, ni aqui ni al cerrar. Liberar es un acto explicito
           del profesor:  ros2 service call /release_emergency_stop std_srvs/srv/Empty
           Ese fue el cuarto fallo de este boton: al SOLTARLA, no al pulsarla,
           el robot arrancaba solo.
        """
        for _ in range(3):
            self._pub_parada.publish(Empty())
            time.sleep(0.05)
        print('PARADA DE EMERGENCIA enviada. El robot no obedecera hasta que\n'
              'alguien la libere:  ros2 service call /release_emergency_stop '
              'std_srvs/srv/Empty')
