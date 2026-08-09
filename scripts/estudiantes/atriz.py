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
from std_srvs.srv import Empty as EmptySrv, SetBool

# ═══════════════════════════════════════════════════════════════════════════
# CONSTANTES — cada una tiene una medida detrás. No se cambian sin otra.
# ═══════════════════════════════════════════════════════════════════════════

# 🔴 EL TOPIC. `/cmd_vel` es la SALIDA del collision_monitor: publicar ahí
#    FUNCIONA y salta la capa de seguridad entera, sin un solo aviso. Es el
#    agujero más silencioso del sistema, y los diez scripts de ROS 1 lo hacían.
#
# 🔴🔴 Y LA CARA B, MEDIDA EL 2026-08-09: pasar por el monitor significa que
#    **con algo a menos de 18 cm el robot NO SE MUEVE. Nada.** Ni avanzar
#    alejandose, ni retroceder, ni girar sobre su eje:
#
#        pared DETRAS a 16,8 cm, 188 cm libres delante
#          avanzar alejandose -> 0.0 cm   ·   girar -> 0.0 grados
#
#    `approach` escala el mando ENTERO por el tiempo hasta colision, y con un
#    punto ya DENTRO de su circulo (radius 0.18) el factor es 0, sin mirar si el
#    movimiento acerca o aleja. El robot solo sale a mano.
#
#    ⚠️ Para el alumno esto se ve como **un robot colgado**: `girar(360)` tarda
#       40 s —el plazo interno— y devuelve -0.1 grados, sin ningun mensaje.
#       SI EL ROBOT NO OBEDECE, MIRA SI TIENE ALGO A MENOS DE 20 cm antes de
#       pensar que se ha roto:  ros2 topic echo /collision_monitor_state
#
#    ✅ Y girando NO rozaria: con el monitor puenteado dio 359,6 y 358,8 de 360,
#       en 12,6 s, sin tocar la pared (con el usuario mirando). El radio
#       circunscrito del robot es ~14,2 cm contra un circulo de 18.
#       Detalle en `93_el_robot_atrapado_por_su_propia_seguridad.txt`.
TOPIC_MANDO = '/cmd_vel_raw'

VEL_MAX = 0.40        # m/s — meseta REAL medida: 0.401 comandando 0.40 (2026-07-31)
VEL_GIRO_MAX = 2.0    # rad/s — 99-102 % del comandado en las cuatro medidas
TIEMPO_MAX = 10.0     # s por llamada — decisión de diseño, no una medida
GRADOS_MAX = 720.0    # ° por llamada — ídem

# 🔴 CUANTO SIGUE GIRANDO EL ROBOT DESPUES DE MANDARLE PARAR.
#    Medido el 2026-08-03 con el robot en el suelo, n=9 a 90/180/360 grados:
#    el lazo cerrado se pasaba **+4.01 grados CONSTANTES** — 4.05 a 90, 4.04 a
#    180 y 3.96 a 360, o sea que NO depende del angulo. Un sobregiro constante
#    en grados es un tiempo constante en segundos: 0.350 s a 0.20 rad/s, que es
#    la velocidad final de la rampa. Es la INERCIA de deceleracion.
#
#    Por eso `girar()` para ANTES de llegar: compensa un retardo medido, que es
#    lo que hace un controlador de verdad con un actuador que no frena en seco.
#    No es una constante calibrada a ojo — es una medida, y se puede volver a
#    medir. Evidencia 58.
ANTICIPACION_GRADOS = 4.0

# 🔴 El watchdog del driver corta a los 0.3 s sin `cmd_vel`. Un `sleep(3)` entre
#    dos publicaciones deja al robot PARADO casi todo el tiempo, y el alumno ve
#    un robot que «no obedece». Hay que republicar más rápido que eso.
RITMO_HZ = 10.0

# 🔴 Cuánto silencio de /odom aborta un giro en lazo cerrado.
#
# El valor sale de MEDIDAS, y hay que mirar DOS regímenes — que es la corrección
# que este comentario recibió el mismo día que se escribió:
#
#   régimen permanente   σ 2.0-2.5 ms · peor hueco **78-81 ms**   (n=3, 60 s cada una)
#   recién reiniciado    σ  16-19 ms  · peor hueco **326 ms**     (20 s tras el arranque)
#
# 🔴 La primera versión de esta constante decía «1.0 s son 12 veces el peor
#    hueco». **Eso era cierto SOLO en régimen permanente.** Contra el transitorio
#    de arranque, 1.0 s dejaba 3x — exactamente el margen que este mismo bloque
#    declaraba insuficiente tres líneas más abajo. Se midió al reiniciar el
#    driver para desplegar otro cambio, y no se buscaba.
#
# ✅ **2.0 s: 6x sobre el peor transitorio medido y 25x sobre el permanente.**
#    Y el coste de subirlo es una asimetría que juega a favor: un falso aborto
#    deja al alumno con el robot a 5 grados y sin explicación; un aborto un
#    segundo más tarde sobre una odometría de verdad muerta no cuesta nada.
#
# 🔴 La versión anterior contaba VUELTAS DEL BUCLE —5, «~0.25 s a 20 Hz»— y por
#    eso abortaba giros con la odometría perfecta: 3× de margen sobre el jitter
#    y una suposición sobre el ritmo del bucle que el propio fichero admitía no
#    haber medido. Un `girar(90)` se quedaba en 5.5° **saliendo con código 0**.
#    Evidencia 85.
SILENCIO_ODOM_S = 2.0


def odom_rancia(ahora, t_ultima_muestra, umbral_s=SILENCIO_ODOM_S):
    """¿Lleva /odom demasiado tiempo sin una muestra NUEVA?

    Pura a propósito, para poder probar el criterio sin robot: el fallo que
    arregla era justo que el criterio no se podía comprobar en ningún sitio.
    Se mide en **segundos de reloj**, nunca en iteraciones del bucle que la
    llama — el bucle no sabe a qué ritmo corre.
    """
    return (ahora - t_ultima_muestra) > umbral_s


# 🔴 TODAS las señales que terminan un programa y SE PUEDEN capturar. Con solo
#    SIGINT, cerrar la terminal (SIGHUP) o perder el SSH dejaba el barrido del
#    X2 girando a 11.8 Hz para siempre: el watchdog de 0.3 s del driver para los
#    MOTORES, pero no hay nada que apague el LIDAR. Verificado provocando cada
#    señal sobre un proceso real (bloque A del informe final).
#
#    🔴 SIGQUIT (Ctrl-\) ESTA EN LA LISTA, y estuvo fuera. Se puede capturar
#       igual que las otras, y es el camino MAS probable justo despues de este
#       arreglo: ahora la biblioteca ignora los Ctrl-C repetidos y pide esperar,
#       y ese es exactamente el momento en que alguien prueba Ctrl-\. Medido sin
#       capturarla: salida 131, «core dumped», barrido ENCENDIDO.
#
#    ⚠️ LO QUE NO SE PUEDE CUBRIR, y conviene tenerlo escrito: SIGKILL
#       (`kill -9`), SIGSTOP, un corte de corriente, un `os._exit()` y una caida
#       dura del interprete (SIGSEGV/SIGABRT). En esos casos NO corre ni el
#       manejador ni `atexit`, y el barrido se queda encendido hasta que el
#       profesor lo apague con `atriz-escaneo off`. Medido, no supuesto.
SENALES_DE_CIERRE = tuple(
    senal for senal in (getattr(signal, nombre, None)
                        for nombre in ('SIGINT', 'SIGTERM', 'SIGHUP', 'SIGQUIT'))
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


def debe_apagar_barrido(lo_encendi):
    """¿Hay que apagar el barrido del LIDAR al cerrar? Solo si lo encendimos.

    🔴 Dejar las cosas como las encontramos. Si al conectar ya llegaba `/scan`,
       es que otro lo tiene encendido —la navegacion, u otro programa— y
       apagarlo al salir lo dejaria CIEGO sin avisar: sin `/scan` el
       `collision_monitor` bloquea el movimiento (medido: 0.0 cm contra 9.9 del
       control) y el robot parece averiado.

    Es un principio general, no un parche: dos consumidores del mismo recurso no
    deben pisarse, y el que llega segundo no manda sobre el primero.
    """
    return bool(lo_encendi)


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
        self._cli_luz_color = self._nodo.create_client(SetBool, '/enable_color')
        #: ¿La luz del sensor la encendio ESTE programa? Solo entonces la apaga
        #: al cerrar: si el robot arranco con `color_detection:=true`, apagarla
        #: seria romperle el montaje a quien lo dejo asi a proposito.
        self._luz_color_mia = False

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

        # 🔴 LAS CUATRO SEÑALES, no solo SIGINT. Cerrar la terminal manda SIGHUP
        #    y `systemctl stop`/`kill` mandan SIGTERM: con el manejador solo en
        #    SIGINT, los dos mataban el proceso sin pasar por `cerrar()` y el
        #    barrido del X2 se quedaba encendido. El watchdog de 0.3 s del
        #    driver para los MOTORES; del LIDAR no sabe nada.
        for senal in SENALES_DE_CIERRE:
            signal.signal(senal, self._al_senal)
        # 🔴 Y la ultima red: salida normal SIN `with`, `sys.exit()`, o una
        #    excepcion que llega arriba del todo. En esos tres casos no hay
        #    `__exit__` ni señal, y hasta ahora tampoco habia cierre.
        #    ⚠️ `atexit` NO es una garantia universal, y no hay que venderla
        #       como tal. Medido: corre con salida normal, `sys.exit()` y
        #       excepcion sin capturar; NO corre con `os._exit()`, SIGKILL,
        #       SIGSEGV, SIGABRT ni ante una señal que nadie capture. Por eso
        #       los manejadores de arriba no sobran: son los que convierten una
        #       señal en una salida por la que `atexit` SI pasa.
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
        # 🔴 EL ORDEN DE ESTAS DOS LINEAS ES LA MITAD DEL ARREGLO. `_cerrando`
        #    va PRIMERO. Al reves quedaba una ventana de dos sentencias en la
        #    que `_cerrado` ya era True pero el guardia AUN NO estaba puesto: si
        #    la señal caia justo ahi, el manejador veia `_cerrando == False`,
        #    llamaba a `cerrar()`, esta salia por `if self._cerrado: return` y
        #    el manejador hacia `sys.exit()`. Ni parar, ni /stop_scan, ni
        #    desmontar — la mecanica exacta del segundo Ctrl-C que este arreglo
        #    venia a cerrar. Reproducido entregando la señal en ese punto:
        #    rastro VACIO. Con este orden, lo peor que pasa es que el cierre
        #    corra entero desde el manejador: ['parar', '/stop_scan',
        #    'desmontar'].
        self._cerrando = True
        self._cerrado = True
        try:
            secuencia_de_cierre(
                parar=lambda: self._mandar(0.0, 0.0, repeticiones=5),
                apagar_barrido=self._apagar_barrido_y_luz,
                desmontar=self._desmontar)
        finally:
            self._cerrando = False

    def _apagar_barrido(self):
        """/stop_scan, PERO solo si lo encendimos nosotros.

        Si no se llama cuando toca, el X2 se queda girando a 11.8 Hz en vez de
        2.7, 24/7 y multiplicado por 16 robots. Y si se llama cuando NO toca,
        deja ciego a quien lo estuviera usando — la navegacion, por ejemplo.
        Lo decide `debe_apagar_barrido()`, que tiene tests.

        ⚠️ El `getattr` con defecto no es adorno: `cerrar()` puede correr desde
           `atexit` o desde el manejador de señales ANTES de que `__init__` haya
           llegado a fijar el atributo.
        """
        if not debe_apagar_barrido(getattr(self, '_barrido_era_mio', True)):
            return
        self._llamar(self._cli_parar_barrido, EmptySrv.Request(),
                     timeout=5.0, que='/stop_scan')

    def _apagar_barrido_y_luz(self):
        """El paso 2 del cierre: el barrido del LIDAR **y** la luz del color.

        🔴 El barrido va PRIMERO y en su propio `try`: apagarlo siempre es la
           promesa de esta biblioteca, y un fallo apagando el LED no puede
           llevarsela por delante. Es la misma razon por la que
           `secuencia_de_cierre` encadena `finally`.

        ⚠️ Y va aqui, no antes de `parar()`: parar el robot manda sobre apagar
           una luz. Si `/enable_color` tardara, el robot ya esta quieto.

        📝 Solo apaga lo que encendio este programa. El driver la apaga tambien
           al cerrarse (`_apagar_rvr`), pero eso no cubre al alumno que termina
           su guion y deja el driver vivo — que es el caso normal.
        """
        try:
            self._apagar_barrido()
        finally:
            if getattr(self, '_luz_color_mia', False):
                try:
                    self.sensor_color(False)
                except BaseException as e:                   # noqa: BLE001
                    print(f'AVISO: la luz del sensor de color se queda '
                          f'ENCENDIDA y gastara bateria: {e!r}')

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
        """SIGINT (Ctrl-C), SIGQUIT (Ctrl-\\), SIGTERM (`kill`) y SIGHUP (cerrar
        la terminal o perder el SSH): para el robot, apaga el barrido y sale.

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
        # ¿Lo tenia encendido otro? Se mira ANTES de encenderlo nosotros.
        # Una espera corta basta: /scan va a ~10 Hz cuando esta activo.
        self._barrido_era_mio = True
        try:
            self._ultimo('_scan', timeout=1.0, que='/scan')
            self._barrido_era_mio = False
            print('AVISO: el barrido del LIDAR ya estaba encendido (¿navegacion\n'
                  '       en marcha?). NO lo apagare al cerrar, para no dejar\n'
                  '       ciego a quien lo este usando.')
        except ErrorAtriz:
            pass                    # no llegaba nada: lo encendemos nosotros

        self._llamar(self._cli_iniciar, EmptySrv.Request(),
                     timeout=10.0, que='/start_scan')
        # Que el servicio conteste no prueba que lleguen barridos: se espera al
        # EFECTO, que es un /scan de verdad.
        self._ultimo('_scan', timeout=8.0, que='/scan')

    def sensor_color(self, encender=True):
        """Enciende o apaga LA LUZ del sensor de color.

        🔴 SIN LUZ NO HAY LECTURA **DE UNA SUPERFICIE QUE REFLEJA** -- el suelo,
           una cinta, un papel. Ahi la diferencia es de 185x y hay que
           encenderla.

        🔴 PERO SI LA SUPERFICIE EMITE LUZ PROPIA (una pantalla, una baldosa
           LED) HAY QUE DEJARLA APAGADA. Medido el 2026-08-08 sobre una pantalla
           de movil a brillo maximo, sin mover el robot (evidencia 86):

               pantalla ROJA, LED del sensor ON   ->  R/G = 0.66   <- MENOS rojo
                                                                     que verde
               pantalla ROJA, LED del sensor OFF  ->  R/G = 5.12   <- inconfundible

           Sobre vidrio el reflejo es especular y BLANCO, y aporta el 88 % de lo
           que se mide: tapa el color de debajo. Apagada, los tres primarios se
           separan por un factor 25-30.

        Es la «sesion de medicion»: se enciende, se mide lo que haga falta, y se
        apaga. `cerrar()` la apaga sola si la encendiste tu.

            with Robot() as robot:
                robot.sensor_color(True)
                r, g, b, claro = robot.color()

        ⚠️ Enciende un LED BLANCO bajo el chasis y gasta bateria mientras siga
           encendido. Apagalo cuando termines de medir.

        📝 Medido el 2026-08-06: canal claro **1 apagado contra 1320 encendido**,
           y vuelve a 1 al apagar. Hasta esa fecha esta biblioteca decia que
           habia que pedirselo al profesor y reiniciar el robot; era falso.
        """
        peticion = SetBool.Request()
        peticion.data = bool(encender)
        r = self._llamar(self._cli_luz_color, peticion, timeout=5.0,
                         que='/enable_color')
        if not r.success:
            raise ErrorAtriz(
                f'no se pudo {"encender" if encender else "apagar"} la luz del '
                f'sensor de color: {r.message}')
        self.hay_color = bool(encender)
        self._luz_color_mia = bool(encender)
        return self.hay_color

    def _comprobar_color(self):
        """¿Arranco el robot con la luz del sensor de color encendida?

        📝 Ya NO es una sentencia: desde el 2026-08-06 se enciende cuando quieras
           con `sensor_color(True)`. Esto solo dice como esta AHORA, para que
           `color()` pueda avisar en vez de devolver oscuridad que parece dato.
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
            # 🔴 «Apagada» NO es «mal». Sobre una superficie que EMITE luz
            #    --una pantalla, una baldosa LED-- apagarla es lo CORRECTO, y
            #    encenderla da lo contrario de lo que hay (evidencia 86). Por eso
            #    esto informa en vez de regañar.
            print('AVISO: la luz del sensor de color esta apagada.\n'
                  '       Para medir una superficie que REFLEJA (suelo, cinta):\n'
                  '         robot.sensor_color(True)\n'
                  '       Para una que EMITE luz (pantalla, baldosa LED), dejala\n'
                  '       APAGADA: encendida taparia el color con su reflejo.')
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

        # ── VIGILANTE DE /odom ───────────────────────────────────────────────
        # Detectar si /odom se queda congelado, POR TIEMPO DE RELOJ.
        #
        # 🔴 ANTES ESTO CONTABA VUELTAS DEL BUCLE: `MAX_SIN_CAMBIO = 5` con el
        #    comentario «~0.25 s a 20 Hz». Falla de tres maneras, y una sola ya
        #    basta (medido el 2026-08-08 sobre el robot, evidencia 85):
        #
        #    1. MIDE EN LA UNIDAD EQUIVOCADA. Cuenta iteraciones y SUPONE que el
        #       bucle va a 20 Hz. Si el proceso se queda sin CPU un cuarto de
        #       segundo -- o si el bucle gira mas rapido de lo previsto --
        #       dispara con la odometria perfecta. El propio fichero admitia
        #       «Nada de esto esta medido sobre el robot».
        #    2. EL MARGEN ERA DE 3x, NO DE 10. El peor hueco real de /odom en
        #       60 s son 81 ms (16.54 Hz, sigma 2.5 ms) y el umbral estaba en
        #       250. Cualquier hipo del planificador lo cruza.
        #    3. Y AL DISPARAR, MENTIA SOBRE LA CAUSA: decia «Odometría perdida o
        #       desconectada» con /odom a 16.54 Hz. Mandaba a buscar una averia
        #       que no existe -- misma familia que «Failed to get scan» con el
        #       barrido apagado a proposito.
        #
        #    🔴 El efecto para el alumno era el peor posible: `girar(90)` acababa
        #       en 5.5 grados, imprimia «Giro 5.5 grados de verdad» y **salia con
        #       codigo 0**. No fallaba: mentia bajito. Reproducido 1 de 4 veces.
        #
        # ✅ Ahora se mide lo que se quiere medir -- cuanto hace que llego una
        #    muestra NUEVA -- con 1.0 s de umbral: unos 16 mensajes perdidos, 12
        #    veces el peor hueco medido. Es el mismo criterio que
        #    `_vigilar_silencio` del driver, que lleva desde julio sin un falso
        #    positivo.
        #
        # ⚠️ Y sigue haciendo falta: el RVR se duerme solo a los 300.6 s, y sin
        #    este guardia un giro en lazo cerrado se quedaria esperando para
        #    siempre una muestra que no llega.
        ultimo_timestamp = None
        t_ultima_muestra = time.monotonic()

        # Se apunta ANTES del objetivo: el robot recorre los ultimos grados
        # por inercia, despues de que dejemos de mandar. Ver ANTICIPACION_GRADOS.
        objetivo_lazo = objetivo - sentido * math.radians(ANTICIPACION_GRADOS)
        if abs(objetivo_lazo) < math.radians(0.5):
            objetivo_lazo = objetivo        # giros muy cortos: sin anticipar

        while not alcanzado(acumulado, objetivo_lazo):
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
            #    🔴 Y la ventaja NO CRECE CON EL ANGULO. Barrido de 1 a 720
            #       grados: solo hay DOS valores posibles, 0 o 0.5730 — nunca
            #       otra cosa — y empata en la mitad de los angulos de todos
            #       los rangos por igual (52 % en 1-90, 48 % en 91-180, 50 % en
            #       181-360, 51 % en 361-720). Es ALIASING de reticula (un paso
            #       de 10 Hz vale exactamente dos de 20, asi que lo que decide
            #       es la paridad del ultimo tramo), no una tendencia. Sobre
            #       720 grados, 0.5730 es el 0.08 %.
            #       📝 Una version anterior de este comentario decia «en angulos
            #          grandes el sobregiro baja de verdad». Era una
            #          generalizacion a partir de cuatro muestras (90/180/360/
            #          720) — el mismo error de metodo que este fichero
            #          persigue. Los cuatro numeros eran correctos; la
            #          conclusion, no.
            #
            #    Se deja en 20 Hz igualmente, y por UNA razon que si se
            #    sostiene: `girar_por_tiempo()` publica tambien a 20 Hz, para
            #    que la practica 4 compare UNA sola variable (el sensor) y no
            #    dos. Bajar este lazo a 10 Hz obligaria a bajar el otro tambien.
            #    ⚠️ Y ojo con el limite real: /odom llega a 16.5 Hz, asi que por
            #       encima de eso quien fija el paso ya no es este bucle sino la
            #       odometria. Nada de esto esta medido sobre el robot.
            self.mover(0.0, sentido * velocidad_giro(objetivo_lazo - acumulado))
            time.sleep(0.05)

            q = self._ultimo('_odom', timeout=2.0,
                             que='/odom').pose.pose.orientation

            # Comprobar si la muestra de /odom cambio
            timestamp_actual = self._odom.header.stamp
            if timestamp_actual != ultimo_timestamp:
                ultimo_timestamp = timestamp_actual
                t_ultima_muestra = time.monotonic()
            elif odom_rancia(time.monotonic(), t_ultima_muestra):
                # 🔴 Se dice QUE SE ABORTO EL GIRO y cuanto llevaba parado el
                #    topic. No se afirma que la odometria este «perdida o
                #    desconectada»: eso es un diagnostico, y este bucle no tiene
                #    con que hacerlo. Un RVR dormido, un driver caido y una Pi
                #    saturada dan los tres el mismo silencio.
                print(f'AVISO: giro ABORTADO en {math.degrees(acumulado):.1f} de '
                      f'{grados:g} grados: /odom lleva {SILENCIO_ODOM_S:.1f} s sin '
                      f'una muestra nueva.\n'
                      f'       Mira el RITMO de /odom, no si el topic existe:\n'
                      f'         ros2 topic hz /odom      (deberia dar ~16.5)')
                self.parar()
                return math.degrees(acumulado)

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
            print('AVISO: la luz del sensor esta apagada; los valores seran '
                  'ruido de fondo (los canales oscilan entre 0 y 1, no ceros '
                  'fijos), no una lectura de verdad.\n'
                  '       Enciendela con:  robot.sensor_color(True)')
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
