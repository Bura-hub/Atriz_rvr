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
from sensor_msgs.msg import Imu
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
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('streaming_interval_ms', INTERVALO_STREAMING_MS)
        self.declare_parameter('cmd_vel_timeout', 0.3)
        self.declare_parameter('publish_tf', True)

        p = self.get_parameter
        self._puerto = p('serial_port').value
        self._baud = int(p('baud').value)
        self._odom_frame = p('odom_frame').value
        self._base_frame = p('base_frame').value
        self._intervalo_ms = int(p('streaming_interval_ms').value)
        self._cmd_vel_timeout = float(p('cmd_vel_timeout').value)
        self._publicar_tf = bool(p('publish_tf').value)

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

        self._odom = Odometry()
        self._odom.header.frame_id = self._odom_frame
        self._odom.child_frame_id = self._base_frame
        self._imu = Imu()
        self._imu.header.frame_id = self._base_frame

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

        self.get_logger().info(
            f'rvr_driver arrancando · puerto={self._puerto} baud={self._baud} '
            f'interval={self._intervalo_ms}ms frames={self._odom_frame}->{self._base_frame}'
        )
        self._enviar(self._conectar_rvr())

    # ─────────────────────────────────────────────────────────────────────────
    # Puente entre rclpy (sincrónico) y el SDK (asyncio)
    # ─────────────────────────────────────────────────────────────────────────
    def _arrancar_loop(self) -> None:
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    def _enviar(self, coro):
        """Envía una corrutina al event loop que YA existe.

        Es el sustituto de las 48 llamadas a `asyncio.run()` del nodo de ROS 1.
        No espera el resultado a propósito: un callback de ROS que se bloquea
        esperando al robot deja de atender mensajes.
        """
        try:
            return asyncio.run_coroutine_threadsafe(coro, self._loop)
        except RuntimeError as e:
            self.get_logger().error(f'no se pudo encolar la corrutina: {e}')
            return None

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
            await self._rvr.wake()
            await self._rvr.reset_yaw()
            await self._rvr.reset_locator_x_and_y()
            await self._registrar_sensores()
            await self._rvr.sensor_control.start(interval=self._intervalo_ms)
            self.get_logger().info(
                f'streaming a {self._intervalo_ms} ms '
                f'(~{1000 / self._intervalo_ms:.1f} Hz esperados)'
            )
        except Exception:
            # No se traga la excepción en silencio: eso es justo lo que hacía
            # rvr_fw_check_async.py, y por eso el nodo parecía sano sin que
            # circulara un dato.
            self.get_logger().error(f'fallo conectando con el RVR:\n{traceback.format_exc()}')

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
        c = datos['ColorDetection']
        msg = Color()
        msg.rgb_color = [int(c['R']), int(c['G']), int(c['B'])]
        msg.confidence = float(c['Confidence'])
        self.pub_color.publish(msg)

    def _quiza_publicar(self, componente: str) -> None:
        """Publica /odom e /imu cuando han llegado los cinco componentes."""
        with self._lock:
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
                # base_link, NO rvr_base_link: el árbol TF estaba partido en dos
                # y era el bloqueante raíz de SLAM.
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
            )
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
        self._enviar(self._rvr.drive_stop())
        self._conduciendo = False

    def _cb_parada_emergencia(self, _msg: Empty) -> None:
        self.get_logger().error('PARADA DE EMERGENCIA')
        self._parada_emergencia = True
        self._conduciendo = False
        if self._rvr is not None:
            self._enviar(self._rvr.drive_stop())

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
        if self._rvr is not None:
            try:
                fut = self._enviar(self._apagar_rvr())
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
# 📝 SERVICIOS DEL NODO DE ROS 1 QUE FALTAN POR PORTAR
# ══════════════════════════════════════════════════════════════════════════════
#
# Se portan cuando se necesiten Y se prueben en banco, no antes. Portar 16
# servicios a ciegas produce 16 cosas sin verificar, que es exactamente lo que
# este proyecto no hace.
#
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
# Y los topics que faltan: encoders (Encoder), ambient_light (Float32),
# infrared_messages (InfraredMessage), cmd_degrees (DegreesTwist).
#
# ⚠️ Antes de portar los dos publishers de IR, DECIDIR: el nodo de ROS 1 tenía
#    `ir_messages` (String) e `infrared_messages` (InfraredMessage), dos topics
#    para lo mismo. Se queda uno.
