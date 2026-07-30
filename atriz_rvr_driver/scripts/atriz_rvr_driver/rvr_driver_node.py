#!/usr/bin/env python3
"""Driver ROS 2 (rclpy) del Sphero RVR.

Portado de `Atriz_rvr_node.py` (ROS 1 / rospy, 1704 líneas) el 2026-07-30.

════════════════════════════════════════════════════════════════════════════════
QUÉ SE HEREDÓ TAL CUAL, PORQUE YA ESTABA BIEN
════════════════════════════════════════════════════════════════════════════════

El nodo de ROS 1 no era el desastre que el plan describía. Verificado sobre
`migracion-ros2` (24c7749) el 2026-07-30, dos cosas que el plan daba por
pendientes **ya estaban resueltas** — se añadieron en commits posteriores al
clon desactualizado sobre el que se hizo la auditoría:

  · El **watchdog de `cmd_vel`** existe (`handle_ros`, timeout 0.3 s).
  · El **event loop de asyncio ya vivía en su propio hilo** (`loop_thread` +
    `run_forever`), no en ráfagas dentro del bucle de ROS.

Ambas se conservan aquí. La segunda, además, es la arquitectura correcta y es
sobre la que se construye este fichero.

════════════════════════════════════════════════════════════════════════════════
QUÉ SE ARREGLA, Y POR QUÉ
════════════════════════════════════════════════════════════════════════════════

1. 🔴 `imu.angular_velocity` iba en **deg/s**, violando REP-103.

   El `gyroscope_handler` original hacía esto:

       robot_twist.angular.x = gyro['X']            # deg/s
       imu.angular_velocity = robot_twist.angular   # <- deg/s al topic /imu
       check_if_need_to_send_msg('gyroscope')       # <- publica
       robot_twist.angular.x = gyro['X'] * 2*pi/360 # ahora sí rad/s
       odom.twist.twist.angular = ...
       check_if_need_to_send_msg('gyroscope')       # <- publica OTRA VEZ

   Dos defectos en seis líneas: `/imu` publicaba grados por segundo, y el
   contador de componentes se incrementaba **dos veces por muestra**, así que
   `/odom` podía salir con la velocidad angular en grados. Degrada SLAM de
   forma silenciosa: nada falla, el mapa sale mal.

   Aquí se convierte **una sola vez**, al entrar, y todo lo de abajo trabaja en
   rad/s.

2. 🔴 48 llamadas a `asyncio.run()` en callbacks → `run_coroutine_threadsafe`.

   Existiendo ya un event loop propio y corriendo, `asyncio.run()` **crea y
   destruye otro loop entero** por cada `cmd_vel`. Aquí los comandos se envían
   al loop que ya existe.

   ⚠️ **Sin medir:** cuánto costaba de verdad en latencia. No se afirma que
   fuera el cuello de botella — solo que es incorrecto. Medir antes de atribuir.

3. `odom → base_link`, no `rvr_base_link`.

   El árbol TF estaba partido en dos: el driver publicaba `rvr_base_link` y el
   LIDAR colgaba de `base_link`, sin puente. Es el **bloqueante raíz** de SLAM.
   Se unifica en `base_link`, que es el nombre canónico (REP-105), y queda
   parametrizado.

4. Todo parametrizado con `declare_parameter`. El original tenía el puerto, los
   frames y el intervalo de streaming a fuego.

5. 🔴 **Keepalive y detector de silencio.** El RVR se duerme solo si nadie le
   habla, y el nodo no se enteraba: seguía vivo publicando cero, sin un error.
   Ahora se le habla cada 30 s (y de paso se publica `battery_state`, que no
   existía) y, si aun así deja de llegar telemetría, el nodo **lo dice** e
   intenta reanudarla. Bloque «SALUD DEL ENLACE».

════════════════════════════════════════════════════════════════════════════════
LO QUE ESTE FICHERO TODAVÍA NO HACE
════════════════════════════════════════════════════════════════════════════════

Se porta primero el **núcleo**: telemetría, `cmd_vel`, parada de emergencia y
watchdog. Es lo que hace falta para teleoperar y para que SLAM tenga datos.

📝 **Faltan 16 de los 20 servicios** del nodo de ROS 1 (LEDs, IR, encoders,
system info, streaming, motores crudos, `move_to_pose`...). Están listados al
final del fichero con su `.srv`. No se portan «por si acaso»: se portan cuando
se necesiten y se prueben, que es lo que este proyecto hace con todo.
"""
import asyncio
import math
import sys
import threading
import traceback

import rclpy
from geometry_msgs.msg import Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import Empty
from std_srvs.srv import Empty as EmptySrv
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from atriz_rvr_msgs.msg import Color

# El SDK de Sphero está vendorizado y se importa como paquete de nivel superior
# (ver setup.py: package_dir={'': 'scripts'}). Es la pieza validada en Python
# 3.12 el 2026-07-30 (Etapa D, GO) y no se ha tocado.
from sphero_sdk import RvrStreamingServices, SerialAsyncDal, SpheroRvrAsync

#: Componentes de telemetría que deben haber llegado antes de publicar /odom.
#: Es el mecanismo del nodo de ROS 1 y se conserva: el SDK entrega cada sensor
#: por separado, y publicar antes de tener los cinco daría mensajes a medias.
COMPONENTES_ODOM = frozenset(
    {'locator', 'quaternion', 'gyroscope', 'velocity', 'accelerometer'}
)

#: El firmware del RVR no baja de 60 ms y cuantiza a múltiplos de 20 ms.
#: Medido el 2026-07-29: 250 ms -> 3.85 Hz · 100 ms -> 9.94 Hz · 60 ms -> 16.59 Hz
#: · 50 ms -> no arranca. No lo bajes de 60 esperando más frecuencia.
INTERVALO_STREAMING_MS = 60

#: Cada cuánto se le habla al RVR para que no se duerma, en segundos.
#:
#: 🔴 EL RVR SE DUERME SOLO SI NADIE LE HABLA, y cuando lo hace este nodo no se
#: entera: deja de recibir muestras y sigue vivo publicando cero, sin un error.
#:
#: ✅ **El timeout de inactividad del RVR son 300.6 s = 5.01 min.** Medido el
#: 2026-07-31 arrancando el driver con `keepalive_period:=0.0` y vigilando el
#: ritmo de /odom durante 12 min. Se durmió DOS veces, y las dos aguantó
#: **300.6 s exactos** desde el último comando: no es una heurística difusa, es
#: un temporizador del firmware. Coincide con los 5 min documentados del RVR.
#:
#: 30 s dejan un margen de 10x sobre ese timeout. Se podría subir a 120 s sin
#: riesgo, pero no hay motivo: un comando cada 30 s son ~2 bytes/s sobre un
#: enlace de 115200 baudios que ya lleva 16.7 Hz de telemetría. El coste de
#: pasarse por abajo es despreciable; el de quedarse corto es un robot mudo en
#: mitad de una práctica.
PERIODO_KEEPALIVE_S = 30.0

#: Cuánto silencio del RVR se tolera antes de dar la alarma e intentar reanudar.
#:
#: A 60 ms de intervalo llegan ~16.7 muestras/s, así que 3 s son ~50 muestras
#: perdidas: no es un hueco puntual, es que el enlace ha dejado de entregar.
#: No se baja más porque un hueco corto bajo carga es normal y no queremos
#: reiniciar el streaming por un pico de CPU.
TIMEOUT_SILENCIO_S = 3.0


def cuaternion_desde_euler(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Convierte ángulos de Euler (rad, orden sxyz) a cuaternión.

    En ROS 1 esto lo daba `tf.transformations.quaternion_from_euler`. En ROS 2
    ese módulo no existe y `tf_transformations` es un paquete aparte que no está
    en `ros-base`, así que se hace a mano en lugar de añadir una dependencia
    para catorce líneas.
    """
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return Quaternion(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy,
    )


class RvrDriverNode(Node):
    """Nodo driver del Sphero RVR."""

    def __init__(self) -> None:
        super().__init__('rvr_driver')

        # ── Parámetros ───────────────────────────────────────────────────────
        # En el nodo de ROS 1 todo esto estaba a fuego. El puerto sobre todo:
        # `/dev/rvr` es un symlink que crea una regla udev, precisamente para
        # que el código no dependa de si el kernel lo llama ttyAMA0 o ttyS0.
        self.declare_parameter('serial_port', '/dev/rvr')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('odom_frame', 'odom')
        # 🔴 base_footprint, NO base_link. Ver el comentario largo de _quiza_publicar.
        self.declare_parameter('base_frame', 'base_footprint')
        # La IMU vive en su propio frame: los datos NO están en base_frame.
        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('streaming_interval_ms', INTERVALO_STREAMING_MS)
        self.declare_parameter('cmd_vel_timeout', 0.3)
        self.declare_parameter('publish_tf', True)
        # Keepalive y detector de silencio. Se pueden desactivar poniéndolos a 0,
        # pero solo tiene sentido para reproducir el fallo del 2026-07-30 a
        # propósito (por ejemplo, para MEDIR de una vez el timeout real del RVR).
        self.declare_parameter('keepalive_period', PERIODO_KEEPALIVE_S)
        self.declare_parameter('silence_timeout', TIMEOUT_SILENCIO_S)

        p = self.get_parameter
        self._puerto = p('serial_port').value
        self._baud = int(p('baud').value)
        self._odom_frame = p('odom_frame').value
        self._base_frame = p('base_frame').value
        self._imu_frame = p('imu_frame').value
        self._intervalo_ms = int(p('streaming_interval_ms').value)
        self._cmd_vel_timeout = float(p('cmd_vel_timeout').value)
        self._publicar_tf = bool(p('publish_tf').value)
        self._periodo_keepalive = float(p('keepalive_period').value)
        self._timeout_silencio = float(p('silence_timeout').value)

        if self._intervalo_ms < 60:
            self.get_logger().warn(
                f'streaming_interval_ms={self._intervalo_ms}: el firmware del RVR no baja '
                'de 60 ms y a 50 ms no arranca. Medido el 2026-07-29.'
            )

        # ── Estado ───────────────────────────────────────────────────────────
        self._parada_emergencia = False
        self._conduciendo = False
        self._t_ultimo_cmd_vel = 0.0
        self._recibidos: set[str] = set()
        self._lock = threading.Lock()

        # ── Estado de vigilancia del enlace con el RVR ───────────────────────
        # `_t_ultima_muestra` lo tocan los handlers del SDK (hilo del asyncio) y
        # lo lee el temporizador de vigilancia (hilo del ejecutor de rclpy), así
        # que va bajo `_lock` como el resto del estado compartido.
        self._t_ultima_muestra = 0.0
        # Hasta que el streaming arranca de verdad no hay silencio que vigilar:
        # si no, el detector saltaría durante el arranque del nodo.
        self._streaming_activo = False
        # Evita que dos recuperaciones se pisen. Una recuperación tarda cientos
        # de ms y el vigilante corre a 1 Hz: sin este guardia se encolarían
        # varias, cada una parando el streaming que la anterior acaba de armar.
        self._recuperando = False
        self._n_recuperaciones = 0
        self._bateria_pct: float | None = None

        self._odom = Odometry()
        self._odom.header.frame_id = self._odom_frame
        self._odom.child_frame_id = self._base_frame
        self._imu = Imu()
        # imu_link, no base_frame: las aceleraciones y velocidades angulares están
        # medidas en el frame del sensor. Ponerle base_frame haría que cualquier
        # consumidor (robot_localization, por ejemplo) las interpretara mal si
        # algún día el sensor deja de estar alineado con el chasis.
        self._imu.header.frame_id = self._imu_frame

        # ── Event loop de asyncio ────────────────────────────────────────────
        # Heredado del nodo de ROS 1, donde ya estaba bien resuelto. El SDK del
        # RVR es asyncio de arriba abajo y rclpy no lo es, así que el loop vive
        # en su propio hilo y los callbacks de ROS le envían trabajo con
        # run_coroutine_threadsafe. NUNCA con asyncio.run(): eso crearía un loop
        # nuevo por cada mensaje.
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)

        # 🔴 EL ORDEN DE LAS DOS LÍNEAS SIGUIENTES NO ES NEGOCIABLE.
        #
        # SpheroRvrAsync.__init__ (sphero_rvr_async.py:35) hace:
        #     asyncio.get_event_loop().run_until_complete(self._check_rvr_fw())
        #
        # Es el único `get_event_loop()` de la ruta usada, el que la auditoría
        # del 2026-07-29 señaló como el riesgo del port. Y muerde: construir el
        # objeto DESDE DENTRO de una corrutina que ya corre en ese loop falla con
        #     RuntimeError: This event loop is already running
        #
        # Verificado el 2026-07-30: el primer intento de este nodo lo hacía así
        # y el driver arrancaba con todos los topics registrados y CERO datos —
        # el fallo silencioso clásico de este proyecto.
        #
        # La solución NO es parchear el SDK: es la única pieza validada en Python
        # 3.12 (Etapa D, GO) y no se toca. Se construye ANTES de arrancar el
        # hilo, con el loop todavía parado, que es exactamente lo que hacía el
        # nodo de ROS 1 (lo creaba en la línea 78, y el run_forever estaba en la
        # 1670).
        self._rvr: SpheroRvrAsync | None = self._construir_rvr()

        self._hilo_loop = threading.Thread(
            target=self._arrancar_loop, name='asyncio-rvr', daemon=True
        )
        self._hilo_loop.start()

        # ── Publishers ───────────────────────────────────────────────────────
        # QoS: la telemetría es best-effort con historial corto. Un /odom viejo
        # no sirve de nada, y con 16 robots sobre WiFi no queremos reenvíos.
        qos_tel = QoSProfile(
            depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        self.pub_odom = self.create_publisher(Odometry, 'odom', qos_tel)
        self.pub_imu = self.create_publisher(Imu, 'imu', qos_tel)
        self.pub_color = self.create_publisher(Color, 'color', qos_tel)
        self._tf = TransformBroadcaster(self) if self._publicar_tf else None

        # La batería es un subproducto GRATIS del keepalive: para no dormirse hay
        # que hablarle al RVR, y `get_battery_percentage()` es la cosa más
        # inofensiva que se le puede pedir —una lectura— así que de paso se
        # publica. El nodo de ROS 1 no publicaba batería en ningún sitio, y en un
        # laboratorio de 16 robots que se quedan sin carga a mitad de práctica
        # eso es información que hace falta.
        #
        # QoS distinto al de la telemetría: RELIABLE + TRANSIENT_LOCAL con
        # depth=1. Llega cada 30 s, así que perder un mensaje significa 30 s a
        # ciegas; y transient_local hace que la web, al conectarse, reciba el
        # último valor conocido sin esperar medio minuto.
        self.pub_bateria = self.create_publisher(
            BatteryState, 'battery_state',
            QoSProfile(
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        # ── Subscribers ──────────────────────────────────────────────────────
        # Grupos de callbacks separados: los comandos no deben esperar a que la
        # telemetría termine de publicar, ni al contrario.
        g_cmd = MutuallyExclusiveCallbackGroup()

        self.create_subscription(
            Twist, 'cmd_vel', self._cb_cmd_vel, 1, callback_group=g_cmd
        )

        # La parada de emergencia va RELIABLE + TRANSIENT_LOCAL a propósito: es
        # el único topic del robot donde perder un mensaje es un problema de
        # seguridad, y transient_local hace que un suscriptor que llegue tarde
        # reciba el último estado.
        qos_estop = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Empty, 'emergency_stop', self._cb_parada_emergencia,
            qos_estop, callback_group=g_cmd,
        )
        # Nombre antiguo, para no romper lo que ya existe. La web publicaba en
        # /rvr/emergency_stop y el driver escuchaba is_emergency_stop: nombres
        # distintos, y por eso el botón de emergencia NO HACÍA NADA (confirmado
        # en banco el 2026-07-29). Se escuchan los dos y se unifica en la Fase 5.
        self.create_subscription(
            Empty, 'is_emergency_stop', self._cb_parada_emergencia,
            qos_estop, callback_group=g_cmd,
        )

        self.create_service(
            EmptySrv, 'release_emergency_stop', self._srv_liberar_parada,
            callback_group=g_cmd,
        )

        # ── Watchdog de cmd_vel ──────────────────────────────────────────────
        # Se comprueba a 20 Hz, cinco veces más rápido que el timeout de 0.3 s.
        # En el nodo de ROS 1 se comprobaba cada ~0.17 s, lo que dejaba el peor
        # caso en ~0.47 s de robot conduciendo solo.
        self.create_timer(0.05, self._watchdog_cmd_vel)

        # ── Keepalive y vigilancia del enlace ────────────────────────────────
        # 🔴 Los dos existen por el fallo del 2026-07-30: el RVR se durmió solo y
        # el nodo siguió vivo al 12.3 % de CPU publicando cero, sin un error.
        # Ver el comentario largo de `_conectar_rvr` y el manual, cap. 9.8.
        #
        # Van en su propio grupo de callbacks: ninguno de los dos debe esperar a
        # que la telemetría o un `cmd_vel` terminen. El vigilante en particular
        # tiene que poder correr aunque el resto esté ocupado, porque su trabajo
        # es precisamente detectar que algo dejó de pasar.
        g_salud = MutuallyExclusiveCallbackGroup()

        if self._periodo_keepalive > 0:
            self.create_timer(
                self._periodo_keepalive, self._keepalive, callback_group=g_salud
            )
        else:
            self.get_logger().warn(
                'keepalive DESACTIVADO (keepalive_period=0): el RVR se dormirá '
                'solo y dejará de publicar sin dar ningún error.'
            )

        if self._timeout_silencio > 0:
            # A 1 Hz. No hace falta más: el timeout es de segundos, no de ms, y
            # este temporizador solo compara dos números.
            self.create_timer(1.0, self._vigilar_silencio, callback_group=g_salud)
        else:
            self.get_logger().warn(
                'detector de silencio DESACTIVADO (silence_timeout=0): si el RVR '
                'deja de enviar, el nodo no lo dirá.'
            )

        self.get_logger().info(
            f'rvr_driver arrancando · puerto={self._puerto} baud={self._baud} '
            f'interval={self._intervalo_ms}ms frames={self._odom_frame}->{self._base_frame} '
            f'keepalive={self._periodo_keepalive:g}s silencio={self._timeout_silencio:g}s'
        )
        self._enviar(self._conectar_rvr(), 'conexión/streaming')

    # ─────────────────────────────────────────────────────────────────────────
    # Puente entre rclpy (sincrónico) y el SDK (asyncio)
    # ─────────────────────────────────────────────────────────────────────────
    def _arrancar_loop(self) -> None:
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    def _enviar(self, coro, etiqueta: str = 'comando'):
        """Envía una corrutina al event loop que YA existe.

        Es el sustituto de las 48 llamadas a `asyncio.run()` del nodo de ROS 1.
        No espera el resultado —un callback de ROS que se bloquea esperando al
        robot deja de atender mensajes— **pero tampoco lo tira a la basura.**

        🔴 La primera versión de este método hacía exactamente eso: encolaba y
        se olvidaba. Resultado el 2026-07-30: `cmd_vel` llegaba, el watchdog
        disparaba, y el robot NO SE MOVÍA — sin un solo mensaje de error, porque
        la excepción de `drive_rc_si_units` moría dentro del Future.

        Es el mismo fallo silencioso que este proyecto persigue en todas partes,
        cometido aquí. El `add_done_callback` es la diferencia entre «no funciona
        y no sé por qué» y un error en el log.
        """
        try:
            fut = asyncio.run_coroutine_threadsafe(coro, self._loop)
        except RuntimeError as e:
            self.get_logger().error(f'{etiqueta}: no se pudo encolar: {e}')
            return None

        def _revisar(f) -> None:
            try:
                f.result()
            except asyncio.CancelledError:
                pass
            except Exception as e:
                self.get_logger().error(f'{etiqueta} FALLÓ: {type(e).__name__}: {e}')

        fut.add_done_callback(_revisar)
        return fut

    # ─────────────────────────────────────────────────────────────────────────
    # Conexión y streaming de sensores
    # ─────────────────────────────────────────────────────────────────────────
    def _construir_rvr(self) -> SpheroRvrAsync | None:
        """Construye el objeto del SDK con el event loop TODAVÍA PARADO.

        Y mide cuánto tarda, porque ese tiempo es diagnóstico: el constructor
        hace la comprobación de firmware, y `rvr_fw_check_async.py` captura
        `except (asyncio.TimeoutError, Exception)` y **continúa en silencio**.
        Así que el nodo arranca igual de contento con el robot dormido.

        El atajo, documentado en CLAUDE.md:
            ~0 s   el RVR responde
            ~10 s  dos timeouts de 5 s -> el RVR NO responde

        Un RVR dormido no devuelve ni un byte: síntoma idéntico a un cable mal
        puesto. Por eso el tiempo se registra siempre.
        """
        import time as _time
        t0 = _time.monotonic()
        try:
            rvr = SpheroRvrAsync(
                dal=SerialAsyncDal(self._loop, port_id=self._puerto, baud=self._baud)
            )
        except Exception:
            self.get_logger().error(
                f'no se pudo construir SpheroRvrAsync:\n{traceback.format_exc()}'
            )
            return None
        dt = _time.monotonic() - t0
        if dt > 5.0:
            self.get_logger().error(
                f'la comprobación de firmware tardó {dt:.1f} s. Son timeouts: el RVR NO '
                'responde. APAGA Y ENCIENDE EL ROBOT antes de tocar la configuración — '
                'un robot dormido da el mismo síntoma que un cable roto.'
            )
        else:
            self.get_logger().info(f'RVR presente (firmware comprobado en {dt:.2f} s)')
        return rvr

    async def _conectar_rvr(self) -> None:
        if self._rvr is None:
            self.get_logger().error('sin objeto RVR: no se puede iniciar el streaming')
            return
        try:
            # Este `wake()` es el del arranque. NO basta por sí solo: el RVR se
            # duerme por inactividad, y de eso se encargan `_keepalive` y
            # `_vigilar_silencio`. Ver el bloque «SALUD DEL ENLACE» más abajo.
            await self._rvr.wake()
            await self._rvr.reset_yaw()
            await self._rvr.reset_locator_x_and_y()
            await self._registrar_sensores()
            await self._rvr.sensor_control.start(interval=self._intervalo_ms)
            # A partir de aquí SÍ hay silencio que vigilar. El reloj se pone en
            # marcha ahora, no antes: si no, el vigilante dispararía durante el
            # arranque, cuando todavía no ha llegado ninguna muestra porque no
            # tenía que haber llegado.
            with self._lock:
                self._t_ultima_muestra = self._ahora_s()
                self._streaming_activo = True
            self.get_logger().info(
                f'streaming a {self._intervalo_ms} ms '
                f'(~{1000 / self._intervalo_ms:.1f} Hz esperados)'
            )
        except Exception:
            # No se traga la excepción en silencio: eso es justo lo que hacía
            # rvr_fw_check_async.py, y por eso el nodo parecía sano sin que
            # circulara un dato.
            self.get_logger().error(f'fallo conectando con el RVR:\n{traceback.format_exc()}')

    # ─────────────────────────────────────────────────────────────────────────
    # SALUD DEL ENLACE — keepalive y detector de silencio
    #
    # 🔴 POR QUÉ EXISTE ESTE BLOQUE
    #
    # El 2026-07-30, durante la Fase 4, este nodo se quedó mudo a mitad de
    # sesión sin que nada pareciera roto: `/odom`, `/imu` y `/color` dejaron de
    # publicar A LA VEZ mientras el proceso seguía vivo al 12.3 % de CPU, con
    # 17 hilos, sus topics registrados (`Publisher count: 1`) y **ni un solo
    # mensaje de error en el log**.
    #
    # Y la pista fácil engañaba: `ros2 topic hz /tf` daba 50 Hz, así que «TF va
    # bien». Pero 50 Hz es exactamente el `transform_publish_period` de
    # slam_toolbox A SOLAS — con este driver aportando serían ~67 Hz.
    #
    # La causa era que el `wake()` del arranque se llamaba UNA sola vez y este
    # nodo no volvía a hablarle al RVR salvo cuando llegaba un `cmd_vel`. El RVR
    # se duerme por inactividad. El SDK vendorizado NO tiene
    # `set_inactivity_timeout`: solo `wake()`, `sleep()` y las de batería.
    #
    # ✅ **El timeout son 300.6 s = 5.01 min**, medido el 2026-07-31 con
    #    `keepalive_period:=0.0`: se durmió dos veces y las dos aguantó 300.6 s
    #    EXACTOS. Es un temporizador del firmware, no una heurística.
    #
    # Por qué importa fuera del banco: un robot que espere cinco minutos a que
    # el estudiante empiece su práctica LLEGARÁ MUDO a la práctica, y la web no
    # verá ningún error porque el nodo está vivo y los topics existen. Y un
    # `systemd` con `Restart=always` no lo arregla: el proceso no muere.
    #
    # Dos piezas, y hacen falta LAS DOS:
    #
    #   · `_keepalive`         PREVIENE: le habla al RVR cada 30 s para que no
    #                          se duerma, y de paso publica la batería.
    #   · `_vigilar_silencio`  DETECTA: si aun así deja de llegar telemetría, lo
    #                          DICE y trata de reanudarla.
    #
    # El keepalive solo cubre la causa conocida. El vigilante cubre el resto —un
    # cable flojo, un `sensor_control` que se cae, un firmware que se atasca— y
    # sobre todo convierte un fallo silencioso en uno ruidoso. Que es de lo que
    # va este proyecto entero.
    # ─────────────────────────────────────────────────────────────────────────
    def _keepalive(self) -> None:
        """Le habla al RVR para que no se duerma. Corre en un timer de rclpy."""
        if self._rvr is None or self._recuperando:
            # Durante una recuperación no se mete otro comando por medio: la
            # recuperación ya le está hablando al RVR, que es justo el objetivo.
            return
        self._enviar(self._leer_bateria(), 'keepalive')

    async def _leer_bateria(self) -> None:
        """Lee la batería. Es el keepalive, y de paso publica `battery_state`.

        Se usa `get_battery_percentage()` y no `wake()` a secas por dos razones:

        1. Es una LECTURA. `wake()` sobre un robot ya despierto no debería hacer
           nada, pero una lectura es inequívocamente inocua: no cambia ningún
           estado del robot, así que no puede interferir con una maniobra en
           curso ni con la parada de emergencia.
        2. Devuelve un dato que hacía falta y no se publicaba en ninguna parte,
           ni siquiera en el nodo de ROS 1. En un laboratorio de 16 robots,
           saber cuál se está quedando sin carga vale más que el comando.

        Si esta llamada FALLA, no se enmascara: es la señal más temprana de que
        el enlace con el RVR se ha caído, y llega antes que el detector de
        silencio. `_enviar` la registra en el log a través de su callback.
        """
        resp = await self._rvr.get_battery_percentage()

        # El SDK devuelve {'percentage': N}. Si algún día cambia, es preferible
        # un aviso a un KeyError que mata el keepalive para siempre.
        if not isinstance(resp, dict) or 'percentage' not in resp:
            self.get_logger().warn(
                f'keepalive: respuesta de batería inesperada ({resp!r}). El robot '
                'sigue despierto —el comando viajó—, pero no se publica batería.'
            )
            return

        pct = float(resp['percentage'])

        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._base_frame
        # El RVR da porcentaje, no voltios ni amperios. BatteryState usa NaN para
        # «no medido», y poner ceros ahí mentiría: 0.0 V es un dato, no un hueco.
        msg.voltage = float('nan')
        msg.current = float('nan')
        msg.charge = float('nan')
        msg.capacity = float('nan')
        msg.design_capacity = float('nan')
        msg.percentage = pct / 100.0          # BatteryState lo quiere en 0.0–1.0
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        msg.present = True
        self.pub_bateria.publish(msg)

        # Se avisa al bajar de umbral, UNA vez por cruce y no en cada lectura:
        # un WARN cada 30 s durante media hora es ruido que se acaba ignorando,
        # y entonces no sirve para nada.
        antes = self._bateria_pct
        self._bateria_pct = pct
        for umbral, nivel in ((10.0, 'error'), (25.0, 'warn')):
            if pct <= umbral and (antes is None or antes > umbral):
                getattr(self.get_logger(), nivel)(
                    f'batería al {pct:.0f} % — el robot se apagará solo cuando se agote'
                )
                break

    def _vigilar_silencio(self) -> None:
        """¿Ha dejado de llegar telemetría del RVR? Timer de rclpy a 1 Hz.

        Mide el SILENCIO, no el estado del proceso ni la existencia del topic.
        Es la diferencia entre detectar este fallo y no detectarlo: el proceso
        estaba perfectamente vivo y los topics registrados mientras no circulaba
        un solo dato.
        """
        if self._recuperando:
            return
        with self._lock:
            if not self._streaming_activo:
                return
            silencio = self._ahora_s() - self._t_ultima_muestra
        if silencio < self._timeout_silencio:
            return

        self._n_recuperaciones += 1
        self.get_logger().warn(
            f'el RVR lleva {silencio:.1f} s sin enviar telemetría '
            f'(se esperan ~{1000 / self._intervalo_ms:.1f} muestras/s). '
            'Lo más probable es que se haya dormido. Intentando reanudar '
            f'(intento nº {self._n_recuperaciones})…'
        )
        self._recuperando = True
        self._enviar(self._recuperar_streaming(), 'recuperación del streaming')

    async def _recuperar_streaming(self) -> None:
        """Despierta el RVR y rearma el streaming.

        Los handlers registrados con `add_sensor_data_handler` sobreviven a un
        `stop()`/`start()`, así que NO se vuelven a registrar: hacerlo los
        duplicaría y cada muestra llegaría dos veces.
        """
        try:
            if self._rvr is None:
                return
            await self._rvr.wake()
            # El `stop()` puede fallar si el RVR estaba dormido y nunca se enteró
            # de que estaba emitiendo. No es un error: es el estado esperado en
            # el caso que estamos arreglando. Se registra y se sigue, porque lo
            # que importa es el `start()` de después.
            try:
                await self._rvr.sensor_control.stop()
            except Exception as e:
                self.get_logger().debug(f'recuperación: stop() falló ({e}), se continúa')
            await self._rvr.sensor_control.start(interval=self._intervalo_ms)

            # El reloj se reinicia AQUÍ, después del start(). Si se reiniciara
            # antes, el vigilante volvería a disparar mientras la recuperación
            # sigue en curso; y si no se reiniciara, dispararía otra vez de
            # inmediato aunque la recuperación haya funcionado, porque la
            # primera muestra nueva todavía no habrá llegado.
            with self._lock:
                self._t_ultima_muestra = self._ahora_s()
            self.get_logger().info(
                'streaming reanudado. Si esto se repite cada pocos minutos, el '
                'keepalive no está llegando: revisa keepalive_period y el enlace.'
            )
        except Exception:
            # Aquí NO se calla: que la recuperación falle es exactamente lo que
            # el operador necesita saber, y es lo único que distingue «se
            # arregló solo» de «este robot necesita atención».
            self.get_logger().error(
                f'la recuperación del streaming FALLÓ:\n{traceback.format_exc()}\n'
                'Si se repite, apaga y enciende el robot: un RVR que no responde '
                'da el mismo síntoma que un cable roto.'
            )
        finally:
            # Pase lo que pase, se libera el guardia. Sin este `finally`, una
            # excepción dejaría `_recuperando=True` para siempre y el vigilante
            # no volvería a intentarlo nunca: el fallo silencioso, otra vez, y
            # esta vez dentro del código escrito para evitarlo.
            self._recuperando = False

    async def _registrar_sensores(self) -> None:
        sc = self._rvr.sensor_control
        await sc.add_sensor_data_handler(RvrStreamingServices.locator, self._h_locator)
        await sc.add_sensor_data_handler(RvrStreamingServices.quaternion, self._h_quaternion)
        await sc.add_sensor_data_handler(RvrStreamingServices.gyroscope, self._h_gyroscope)
        await sc.add_sensor_data_handler(RvrStreamingServices.velocity, self._h_velocity)
        await sc.add_sensor_data_handler(RvrStreamingServices.accelerometer, self._h_accel)
        await sc.add_sensor_data_handler(RvrStreamingServices.color_detection, self._h_color)

    # ─────────────────────────────────────────────────────────────────────────
    # Handlers del SDK. Corren en el hilo del event loop, así que todo lo que
    # toque estado compartido va bajo el lock.
    # ─────────────────────────────────────────────────────────────────────────
    async def _h_locator(self, datos) -> None:
        with self._lock:
            self._odom.pose.pose.position.x = float(datos['Locator']['X'])
            self._odom.pose.pose.position.y = float(datos['Locator']['Y'])
            self._odom.pose.pose.position.z = 0.0
        self._quiza_publicar('locator')

    async def _h_quaternion(self, datos) -> None:
        q = datos['Quaternion']
        with self._lock:
            orient = Quaternion(
                x=float(q['X']), y=float(q['Y']), z=float(q['Z']), w=float(q['W'])
            )
            self._odom.pose.pose.orientation = orient
            self._imu.orientation = orient
        self._quiza_publicar('quaternion')

    async def _h_gyroscope(self, datos) -> None:
        """Velocidad angular. **Se convierte a rad/s UNA sola vez.**

        Aquí estaba el bug de REP-103: el original asignaba deg/s a
        `imu.angular_velocity`, publicaba, y solo después convertía — contando
        la muestra dos veces. Ver la cabecera del fichero.
        """
        g = datos['Gyroscope']
        rad = math.pi / 180.0
        with self._lock:
            angular = Vector3(
                x=float(g['X']) * rad,
                y=float(g['Y']) * rad,
                z=float(g['Z']) * rad,
            )
            self._odom.twist.twist.angular = angular
            self._imu.angular_velocity = angular
        self._quiza_publicar('gyroscope')

    async def _h_velocity(self, datos) -> None:
        v = datos['Velocity']
        with self._lock:
            self._odom.twist.twist.linear.x = float(v['X'])
            self._odom.twist.twist.linear.y = float(v['Y'])
        self._quiza_publicar('velocity')

    async def _h_accel(self, datos) -> None:
        a = datos['Accelerometer']
        with self._lock:
            self._imu.linear_acceleration = Vector3(
                x=float(a['X']), y=float(a['Y']), z=float(a['Z'])
            )
        self._quiza_publicar('accelerometer')

    async def _h_color(self, datos) -> None:
        # El color no pasa por `_quiza_publicar` (no forma parte de /odom), así
        # que marca el latido por su cuenta. Si no lo hiciera, un robot que solo
        # enviara color parecería mudo.
        with self._lock:
            self._t_ultima_muestra = self._ahora_s()
        c = datos['ColorDetection']
        msg = Color()
        msg.rgb_color = [int(c['R']), int(c['G']), int(c['B'])]
        msg.confidence = float(c['Confidence'])
        self.pub_color.publish(msg)

    def _quiza_publicar(self, componente: str) -> None:
        """Publica /odom e /imu cuando han llegado los cinco componentes."""
        with self._lock:
            # Latido del enlace. Va ANTES del return de abajo, a propósito: lo
            # que se vigila es que el RVR siga ENVIANDO, no que se complete un
            # /odom. Si llegaran cuatro de los cinco componentes, /odom dejaría
            # de publicarse pero el enlace estaría vivo, y reiniciar el streaming
            # no arreglaría nada. Son dos fallos distintos y no hay que
            # confundirlos.
            self._t_ultima_muestra = self._ahora_s()
            self._recibidos.add(componente)
            if not COMPONENTES_ODOM <= self._recibidos:
                return
            self._recibidos.clear()
            ahora = self.get_clock().now().to_msg()
            self._odom.header.stamp = ahora
            self._imu.header.stamp = ahora
            odom = self._odom
            imu = self._imu
            pos = odom.pose.pose.position
            ori = odom.pose.pose.orientation

        try:
            self.pub_odom.publish(odom)
            self.pub_imu.publish(imu)
            if self._tf is not None:
                t = TransformStamped()
                t.header.stamp = ahora
                t.header.frame_id = self._odom_frame
                # 🔴 base_footprint, y el POR QUÉ importa (2026-07-30).
                #
                # Un frame solo puede tener UN PADRE en TF. El URDF publica
                # `base_footprint -> base_link` (estático), así que si el driver
                # publicara `odom -> base_link`, base_link tendría DOS padres y
                # tf2 respondería:
                #     Could not find a connection between 'odom' and
                #     'base_footprint' ... Tf has two or more unconnected trees
                #
                # Pasó exactamente eso: la primera versión de este nodo publicaba
                # odom->base_link y slam_toolbox repetía «Failed to compute odom
                # pose» indefinidamente.
                #
                # Y lo peor: `tf2_echo odom laser` SÍ resolvía, por el camino
                # odom->base_link->laser, ignorando base_footprint. La verificación
                # de la Fase 3 pasaba y ocultaba el problema. La prueba correcta es
                # `tf2_echo odom base_footprint`, que es lo que SLAM necesita.
                #
                # La cadena correcta, con un solo padre por frame:
                #     odom -> base_footprint -> base_link -> laser
                #
                # Semánticamente también es lo bueno: el locator del RVR da la
                # posición del robot sobre el plano del suelo, que es justo lo que
                # base_footprint representa.
                t.child_frame_id = self._base_frame
                t.transform.translation.x = pos.x
                t.transform.translation.y = pos.y
                t.transform.translation.z = pos.z
                t.transform.rotation = ori
                self._tf.sendTransform(t)
        except Exception:
            self.get_logger().error(f'fallo publicando:\n{traceback.format_exc()}')

    # ─────────────────────────────────────────────────────────────────────────
    # Comandos
    # ─────────────────────────────────────────────────────────────────────────
    def _cb_cmd_vel(self, msg: Twist) -> None:
        if self._parada_emergencia:
            return
        if self._rvr is None:
            self.get_logger().warn('cmd_vel recibido pero el RVR no está conectado')
            return

        # El SDK espera grados/segundo; ROS manda rad/s (REP-103).
        v_lineal = float(msg.linear.x)
        v_angular_deg = float(msg.angular.z) * 180.0 / math.pi

        self._enviar(
            self._rvr.drive_rc_si_units(
                linear_velocity=v_lineal,
                yaw_angular_velocity=v_angular_deg,
                flags=0,
            ),
            f'cmd_vel(v={v_lineal:.2f} w={v_angular_deg:.1f}deg/s)',
        )
        self._conduciendo = True
        self._t_ultimo_cmd_vel = self._ahora_s()

    def _watchdog_cmd_vel(self) -> None:
        """Para los motores si `cmd_vel` deja de llegar.

        Heredado del nodo de ROS 1, donde ya existía — el plan decía que no y
        estaba equivocado. Lo que cambia aquí es la frecuencia de comprobación:
        20 Hz en vez de ~6 Hz, así el peor caso baja de ~0.47 s a ~0.35 s.
        """
        if not self._conduciendo or self._rvr is None:
            return
        if self._ahora_s() - self._t_ultimo_cmd_vel <= self._cmd_vel_timeout:
            return
        self.get_logger().warn(
            f'watchdog: {self._cmd_vel_timeout} s sin cmd_vel, parando motores'
        )
        self._enviar(self._rvr.drive_stop(), 'watchdog drive_stop')
        self._conduciendo = False

    def _cb_parada_emergencia(self, _msg: Empty) -> None:
        self.get_logger().error('PARADA DE EMERGENCIA')
        self._parada_emergencia = True
        self._conduciendo = False
        if self._rvr is not None:
            self._enviar(self._rvr.drive_stop(), 'parada de emergencia')

    def _srv_liberar_parada(self, _req, resp):
        self.get_logger().warn('parada de emergencia liberada')
        self._parada_emergencia = False
        return resp

    def _ahora_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    # ─────────────────────────────────────────────────────────────────────────
    def cerrar(self) -> None:
        """Deja el robot parado y suelta el puerto serie."""
        self.get_logger().info('cerrando: parando motores y liberando el puerto')
        # Se apaga la vigilancia ANTES de cerrar nada. Si no, el apagado limpio
        # —que para el streaming a propósito— dispararía el detector de silencio
        # y el nodo intentaría «recuperarse» mientras se está muriendo, dejando
        # un WARN alarmante en el log de cada parada normal.
        with self._lock:
            self._streaming_activo = False
        self._recuperando = True
        if self._rvr is not None:
            try:
                fut = self._enviar(self._apagar_rvr(), 'apagado')
                if fut is not None:
                    fut.result(timeout=3.0)
            except Exception:
                self.get_logger().warn('el apagado limpio del RVR no terminó en 3 s')
        self._loop.call_soon_threadsafe(self._loop.stop)
        self._hilo_loop.join(timeout=2.0)

    async def _apagar_rvr(self) -> None:
        await self._rvr.drive_stop()
        await self._rvr.sensor_control.clear()
        await self._rvr.close()


def main(args=None) -> int:
    rclpy.init(args=args)
    nodo = RvrDriverNode()
    # MultiThreadedExecutor para que los grupos de callbacks separados sirvan de
    # algo: con el executor por defecto, un comando lento bloquearía el watchdog.
    ejecutor = MultiThreadedExecutor()
    ejecutor.add_node(nodo)
    try:
        ejecutor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        nodo.cerrar()
        ejecutor.shutdown()
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())


# ══════════════════════════════════════════════════════════════════════════════
# 📝 LO QUE FALTA POR EXPONER EN ROS 2 — Y LAS DECISIONES YA TOMADAS
# ══════════════════════════════════════════════════════════════════════════════
#
# DECISIÓN (2026-07-30): esto se porta CUANDO EL RESTO ESTÉ TERMINADO, no antes.
# Decisión del usuario, y tiene sentido: SLAM y la navegación definen qué hace
# falta de verdad de esta interfaz, así que portar 20 piezas ahora sería adivinar
# cuáles importan.
#
# ⚠️ CONSECUENCIA QUE HAY QUE TENER PRESENTE: hasta que esto se porte, **NO SE
#    PUEDEN HACER MÁQUINAS DE ESTADO** que usen LEDs, batería, encoders o luz
#    ambiente. Un programa que quisiera hacerlo tendría que hablar con el SDK por
#    su cuenta, y entonces DOS PROCESOS SE PELEARÍAN POR /dev/rvr. El puerto serie
#    no se comparte.
#
# LO BUENO: el hardware detrás de todo esto YA ESTÁ VERIFICADO (2026-07-30, ver el
# capítulo 8bis del manual y evidencia_24_04/10_leds_sensores). Los 11 grupos de
# LED confirmados a la vista, 10 de 11 sensores con datos reales. Así que portar
# esto es trabajo de rclpy, NO de averiguar si el sensor responde: si un servicio
# portado no funciona, el fallo está en el port y en ningún otro sitio.
#
# ── SERVICIOS (16) ────────────────────────────────────────────────────────────
#   LEDs        set_led_rgb (SetLEDRGB) · set_multiple_leds (SetMultipleLEDs)
#   Infrarrojos ir_mode (SetIRMode) · send_infrared_message (SendInfraredMessage)
#               set_ir_evading (SetIREvading)
#   Sensores    get_encoders (GetEncoders) · get_rgbc_sensor_values (GetRGBCSensorValues)
#               battery_state (BatteryState) · enable_color (std_srvs/SetBool)
#   Movimiento  raw_motors (RawMotors) · move_timed (MoveTimed)
#               move_to_pose (MoveToPose) · move_to_pos_and_yaw (MoveToPosAndYaw)
#   Estado      get_system_info (GetSystemInfo) · get_control_state (GetControlState)
#   Config      set_drive_parameters (SetDriveParameters)
#               configure_streaming (ConfigureStreaming) · start_streaming (StartStreaming)
#   Odometría   reset_odom (std_srvs/Empty)
#
# ── TOPICS (4) ────────────────────────────────────────────────────────────────
#   encoders (Encoder) · ambient_light (Float32)
#   infrared_messages (InfraredMessage) · cmd_degrees (DegreesTwist)  ← entrada
#
# ══════════════════════════════════════════════════════════════════════════════
# LAS DOS DECISIONES DE DISEÑO, TOMADAS EL 2026-07-30
# ══════════════════════════════════════════════════════════════════════════════
#
# 1. `infrared_messages` (InfraredMessage) SÍ · `ir_messages` (String) NO.
#
#    El driver de ROS 1 publicaba LOS DOS, con los mismos datos: uno tipado y otro
#    como cadena de texto. Se queda el tipado, y no es una preferencia estética:
#
#      · Un topic `String` con datos estructurados no se puede introspeccionar
#        (`ros2 interface show` no dice nada útil), obliga a cada consumidor a
#        parsear texto, y cualquier cambio de formato rompe en silencio.
#      · Con rosbridge —que es como habla la web (ARQUITECTURA.md, D2)— un mensaje
#        tipado llega al navegador como un objeto JSON con sus campos. Una cadena
#        llega como una cadena que hay que volver a parsear en JavaScript.
#      · Mantener dos topics con lo mismo garantiza que se desincronicen.
#
#    `ir_messages` NO se porta.
#
# 2. SIN namespace por defecto, pero el launch lo soporta.
#
#    ARQUITECTURA.md, Decisión 1: el aislamiento entre los 16 robots lo da UN
#    ROS_DOMAIN_ID POR ROBOT, que los separa por completo a nivel de DDS. Con eso,
#    un namespace `/rvr_01` NO añade aislamiento: solo alarga cada nombre de topic
#    y cada comando de diagnóstico.
#
#    Y la web tampoco lo necesita para distinguirlos: habla por rosbridge, un
#    WebSocket POR ROBOT, así que la desambiguación ya la da la conexión.
#
#    Se conserva el argumento `namespace` en los launch para el caso de meter dos
#    robots en el mismo dominio a propósito (por ejemplo, para depurar una
#    interacción entre dos), pero el valor por defecto es vacío.
#
#    ⚠️ Si algún día se cambia esta decisión, hay que cambiarla en TRES sitios a la
#       vez: el `base_frame` del driver, el link del URDF y el `frame_id` del
#       LIDAR. Desalinear uno parte el árbol TF EN SILENCIO.
#
# 3. El driver publica `odom → base_footprint`, NO `odom → base_link`.
#    (Añadida el 2026-07-30, arreglando el bloqueante de la Fase 4.)
#
#    EN TF UN FRAME SOLO PUEDE TENER UN PADRE. El driver publicaba
#    `odom → base_link` mientras el URDF publicaba `base_footprint → base_link`,
#    así que `base_link` tenía DOS padres y el árbol se partía en dos:
#
#        Could not find a connection between 'odom' and 'base_footprint' ...
#        Tf has two or more unconnected trees.
#
#    slam_toolbox repetía `Failed to compute odom pose` y no mapeaba nada.
#
#    `base_footprint` es además lo correcto por REP-105 —el frame proyectado al
#    suelo es el que se localiza— y es lo que slam_toolbox pide en su
#    `base_frame`. La IMU pasó a tener su propio `imu_frame` (`imu_link`): sus
#    datos NO están en `base_frame`, y decir lo contrario era otra imprecisión.
#
#    ⚠️ Y LA LECCIÓN DE MÉTODO, que vale más que el arreglo: la verificación de la
#       Fase 3 era `tf2_echo odom laser` y PASABA, resolviendo por el camino
#       equivocado (`odom → base_link → laser`) mientras `base_footprint` colgaba
#       de otro árbol que nadie miraba.
#
#       COMPRUEBA EL TRANSFORM QUE PIDE EL CONSUMIDOR, con sus frames exactos:
#
#           ros2 run tf2_ros tf2_echo odom base_footprint     # ← ESTA
#
#       Un `tf2_echo` que resuelve prueba que hay UN camino, no que el árbol
#       esté bien.
