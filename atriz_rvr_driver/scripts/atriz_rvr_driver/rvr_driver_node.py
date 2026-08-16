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
import concurrent.futures
import math
import sys
import threading
# 🔴 Reloj MONÓTONO, no el de ROS (`_ahora_s`, que es el del sistema). Los sellos
#    del apagado de la luz del color deciden una acción física, y un salto de NTP
#    no debe decidirla. El resto del nodo usa `_ahora_s` y se queda como está.
import time as _time
import traceback

import rclpy
from geometry_msgs.msg import Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import BatteryState, Illuminance, Imu
from std_msgs.msg import Empty
from std_srvs.srv import Empty as EmptySrv, SetBool
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from atriz_rvr_msgs.msg import (Color, ControlState, Encoder, EstadoIR,
                                EstadoRobot, InfraredMessage, MotorStatus,
                                SystemInfo)
from atriz_rvr_msgs.srv import (
    GetControlState, GetEncoders, GetRGBCSensorValues, GetSystemInfo,
    SendInfraredMessage, SetDriveParameters, SetIREvading, SetIRMode,
    MoveTimed, MoveToPosAndYaw, MoveToPose, RawMotors,
    SetLEDRGB, SetLeds, SetMultipleLEDs, SetPosAndYaw, TriggerLedEvent,
)

# El SDK de Sphero está vendorizado y se importa como paquete de nivel superior
# (ver setup.py: package_dir={'': 'scripts'}). Es la pieza validada en Python
# 3.12 el 2026-07-30 (Etapa D, GO) y no se ha tocado.
from sphero_sdk import RvrLedGroups, RvrStreamingServices, SerialAsyncDal, SpheroRvrAsync
from sphero_sdk.common.enums.power_enums import BatteryVoltageReadingTypesEnum

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

#: Gravedad estándar. El RVR reporta la aceleración en **g** y
#: `sensor_msgs/Imu` la exige en **m/s²**. Medido en reposo el 2026-07-31: el
#: módulo del vector era 0.973, no 9.81.
G_MS2 = 9.80665

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


def euler_desde_cuaternion(q: Quaternion) -> tuple[float, float, float]:
    """(roll, pitch, yaw) en radianes desde un cuaternión.

    Hace falta para poder RESTAR el yaw del arranque sin perder el roll y el
    pitch, que en este robot no son cero: la IMU reporta ~8° de inclinación.

    🔴 Eso NO significa que el robot esté inclinado. Se decía «confirmado por
    tres vías independientes» y no lo eran —TF, cuaternión y acelerómetro salen
    todos de la misma IMU—, y el 2026-07-31 se midió con una regla que el disco
    del LIDAR está a la misma altura en los cuatro puntos: el robot está
    horizontal. Ver `_h_quaternion` y el manual, cap. 13.
    """
    x, y, z, w = q.x, q.y, q.z, q.w
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sp = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sp)))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


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
        # El CHASIS. Es distinto de `base_frame` (que es `base_footprint`, la
        # proyección en el suelo) porque los sensores están en el cuerpo, no en
        # el suelo. Lo usan `/ambient_light` y `/motor_status`.
        #
        # 🔴 ESTABA ESCRITO A FUEGO en esos dos publicadores hasta el 2026-08-01.
        #    Con `namespace` sin usar hoy daba igual, pero dejaba ROTO el camino
        #    de escape: si algún día se mete a los 16 robots en un mismo dominio
        #    DDS, un namespace renombra los TOPICS pero **no los `frame_id`**, y
        #    estos dos se habrían quedado sin prefijo partiendo el árbol TF — el
        #    mismo fallo que costó la Fase 3. Ahora es un parámetro.
        self.declare_parameter('body_frame', 'base_link')
        self.declare_parameter('streaming_interval_ms', INTERVALO_STREAMING_MS)
        self.declare_parameter('cmd_vel_timeout', 0.3)
        self.declare_parameter('publish_tf', True)
        # Keepalive y detector de silencio. Se pueden desactivar poniéndolos a 0,
        # pero solo tiene sentido para reproducir el fallo del 2026-07-30 a
        # propósito (por ejemplo, para MEDIR de una vez el timeout real del RVR).
        self.declare_parameter('keepalive_period', PERIODO_KEEPALIVE_S)
        self.declare_parameter('silence_timeout', TIMEOUT_SILENCIO_S)
        # 🆕 2026-08-11 · Ritmo del sondeo de los sensores IR, en Hz. **0.0 lo apaga.**
        #
        # 🔴 POR QUÉ ES UN PARÁMETRO Y NO UNA CONSTANTE: es UN COMANDO MÁS POR
        #    SEGUNDO por el mismo puerto serie que lleva la telemetría a 16,7 Hz,
        #    y en este robot el margen del enlace está MEDIDO y es estrecho: a 50
        #    ms de intervalo el streaming ni siquiera arranca (`_registrar_sensores`).
        #    Así que su coste se mide con esto en 1.0 y en 0.0, comparando el
        #    ritmo de `/odom`, en vez de suponer que «uno por segundo no se nota».
        #
        # 📝 1 Hz no es un número redondo elegido al azar: la lectura IR del
        #    firmware SE BORRA AL SEGUNDO (referencia_sdk/sensor.md:54). Sondear
        #    más despacio garantiza perder detecciones.
        self.declare_parameter('ir_sondeo_hz', 1.0)
        # ✅ Por defecto **False** desde el 2026-07-31: se publica la
        # orientación PLANA, que es la físicamente correcta. La inclinación que
        # reporta el RVR es un artefacto de su acelerómetro, no del robot.
        # Ver `_h_quaternion` y el manual, cap. 13.
        self.declare_parameter('publicar_inclinacion', False)
        # 🔴 Enciende el LED blanco del sensor de color. Ver `_registrar_sensores`.
        self.declare_parameter('color_detection', False)

        # ── Apagado automático de la luz del sensor de color ─────────────────
        # 🔴 POR QUÉ EXISTE, y no es comodidad. La luz se puede encender desde el
        #    navegador (`enable_color`), y **el navegador no puede prometer que
        #    la apagará**: una pestaña cerrada de golpe, un portátil que se
        #    cierra o un corte de WiFi no ejecutan ninguna limpieza, y el cliente
        #    reconecta sin memoria de haber encendido nada. La garantía tiene que
        #    vivir donde algo sobrevive a todo eso: aquí.
        #
        # 🔴 «SIN ACTIVIDAD» CUENTA LAS DOS VÍAS, y esto NO es un detalle:
        #    `atriz.py:903` lee el color POR SERVICIO, no por el topic. Contando
        #    solo suscriptores de `/color`, un alumno haciendo la práctica 5 vería
        #    apagarse la luz a mitad — el apagado de seguridad rompiendo el caso
        #    legítimo. Cuenta suscriptores **o** llamadas a `get_rgbc_sensor_values`.
        #
        # ⚠️ Y hace falta ADEMÁS un tope duro: rosbridge abre **una sola
        #    suscripción ROS** para todos sus clientes WebSocket, así que el
        #    contador no distingue una pestaña de ocho y **una pestaña olvidada
        #    mantiene la luz encendida indefinidamente** — justo el caso que esto
        #    viene a cubrir. Lo que ocurra antes manda.
        #
        # 📌 Los dos son parámetros y no constantes: si en el aula resultan
        #    cortos, cambiarlos no debe exigir recompilar. 0 = desactivado.
        self.declare_parameter('color_apagado_inactividad_s', 120.0)
        self.declare_parameter('color_apagado_max_s', 900.0)

        p = self.get_parameter
        self._puerto = p('serial_port').value
        self._baud = int(p('baud').value)
        self._odom_frame = p('odom_frame').value
        self._base_frame = p('base_frame').value
        self._imu_frame = p('imu_frame').value
        self._body_frame = p('body_frame').value
        self._intervalo_ms = int(p('streaming_interval_ms').value)
        self._cmd_vel_timeout = float(p('cmd_vel_timeout').value)
        self._publicar_tf = bool(p('publish_tf').value)
        self._periodo_keepalive = float(p('keepalive_period').value)
        self._timeout_silencio = float(p('silence_timeout').value)
        self._publicar_inclinacion = bool(p('publicar_inclinacion').value)
        self._color_detection = bool(p('color_detection').value)
        self._color_inactividad = float(p('color_apagado_inactividad_s').value)
        self._color_max = float(p('color_apagado_max_s').value)

        #: ¿La luz la encendió el SERVICIO, o venía del parámetro de arranque?
        #: 🔴 Solo se apaga sola la que encendió el servicio. Quien arranca con
        #:    `color_detection:=true` lo ha pedido a propósito y no se le toca el
        #:    montaje — misma regla que `_luz_color_mia` en `atriz.py`.
        self._color_por_servicio = False
        self._color_t_enable = 0.0
        self._color_t_actividad = 0.0

        if self._intervalo_ms < 60:
            self.get_logger().warn(
                f'streaming_interval_ms={self._intervalo_ms}: el firmware del RVR no baja '
                'de 60 ms y a 50 ms no arranca. Medido el 2026-07-29.'
            )

        # ── Estado ───────────────────────────────────────────────────────────
        self._parada_emergencia = False
        self._conduciendo = False
        self._t_ultimo_cmd_vel = 0.0
        # Batería: último estado del firmware visto, y si las lecturas van bien.
        # Aquí y no con `getattr`, que es donde nadie los busca.
        self._estado_bateria = None
        #: Ultimo estado sano/fallo de cada sondeo periodico, para `_avisar_transicion`.
        self._estado_sondeo: dict = {}
        self._bat_v_ok = True
        self._bat_s_ok = True
        self._recibidos: set[str] = set()
        # 🔴 CONJUNTO APARTE, no `_recibidos`. `_quiza_publicar` VACÍA `_recibidos`
        #    en cada ciclo de /odom —es el juego de componentes recibidos— así que
        #    un «avisa una vez» apoyado en él avisaba 13 VECES POR SEGUNDO, desde
        #    el hilo de asyncio y contra el journal. Medido el 2026-08-01: bastó
        #    para tumbar la telemetría entera y hacer creer que la culpa era de
        #    un comando de LED.
        self._avisos_dados: set[str] = set()
        self._lock = threading.Lock()

        # ── Salud de los motores ─────────────────────────────────────────────
        # El RVR notifica por EVENTOS, no por sondeo: si nunca ha pasado nada, no
        # llega nada. Por eso el estado se guarda aquí y los tiempos arrancan a
        # None — «nunca ha llegado una notificación» y «todo va bien» son cosas
        # distintas, y confundirlas es el fallo silencioso favorito de este
        # proyecto.
        self._motor_atasco = [False, False]     # [izquierdo, derecho]
        self._motor_fallo = False
        self._motor_temp = [0.0, 0.0]
        self._motor_estado_term = [0, 0]
        self._t_ultimo_atasco: float | None = None      # solo la notificación
        self._t_ultimo_fallo: float | None = None       # el sondeo
        self._t_ultimo_termico: float | None = None     # el sondeo

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

        # ── Estado de `/estado_robot` (AÑADIDO 2026-08-04) ────────────────────
        # ⚠️ NO VERIFICADO: escrito sin robot delante. Ver NOTAS_ESTADO_ROBOT.md.
        #
        # 🔴 `_t_muestra_real` y `_n_muestras` son un ESPEJO de `_t_ultima_muestra`
        #    que SOLO tocan los handlers del SDK, y esa distinción es el corazón
        #    de `reanudaciones_fallidas`.
        #
        #    `_t_ultima_muestra` lo reinician también `_conectar_rvr` y
        #    `_recuperar_streaming` —a propósito, para que el vigilante no vuelva
        #    a disparar mientras la recuperación está en curso—, así que ese campo
        #    dice «hace poco que pasó algo», no «hace poco que llegó un dato». Si
        #    el contador de reanudaciones se apoyara en él, una reanudación con el
        #    RVR APAGADO parecería un éxito: es exactamente el fallo medido el
        #    2026-08-02 (8 «streaming reanudado» en 30 s con /odom a cero), y
        #    reproducirlo dentro del campo escrito para detectarlo sería el chiste
        #    completo.
        #
        #    Estos dos solo avanzan cuando el RVR entrega una muestra de verdad.
        self._t_muestra_real: float | None = None
        self._n_muestras = 0
        #: Foto de `_n_muestras` en el instante del último intento de reanudar.
        #: Si `_n_muestras` la supera, llegó un dato DESPUÉS del intento.
        self._muestras_al_reanudar = 0
        #: Intentos de reanudar SEGUIDOS que no devolvieron ni una muestra.
        self._reanudaciones_fallidas = 0
        #: Espera creciente entre intentos (2026-08-14, evidencia 116). Sin
        #: ella, un RVR APAGADO —cargando, lo cotidiano— provocaba un intento
        #: cada ~6 s para siempre: ~46.000 líneas/día por robot sobre una
        #: microSD, ahogando los errores de verdad (evidencia 52). No antes de
        #: este instante monotónico no se lanza otro intento.
        self._no_reanudar_antes_s = 0.0
        #: Contador monótono de publicaciones de `/estado_robot`. Es la señal de
        #: vida del NODO: un topic que existe no prueba que haya nadie detrás.
        self._latido = 0

        # 🔴 CUÁNDO SE COMPLETÓ EL ÚLTIMO `/odom`, Y POR QUÉ HACE FALTA UN TERCER
        #    RELOJ. Encontrado el 2026-08-04 desde el robot, revisando esta misma
        #    rama antes de fusionarla.
        #
        #    `_t_muestra_real` avanza en `_quiza_publicar` ANTES del `return` que
        #    se toma cuando aún no han llegado los cinco componentes — y eso es
        #    correcto: lo que vigila es que el RVR siga ENVIANDO. Pero deja un
        #    TERCER estado que no detecta nadie, ni el vigilante de silencio ni
        #    `rvr_responde`:
        #
        #      llegan 4 de los 5 componentes  ->  el latido avanza,
        #                                         `rvr_responde` dice true,
        #                                         y `/odom` está a 0 Hz
        #
        #    Desde el muro del profesor ese robot se pinta VERDE con la odometría
        #    muerta. Y no es hipotético: es el estado que no se pudo descartar el
        #    2026-08-04 —los cinco topics del stream a cero, el RVR contestando a
        #    consultas y el vigilante callado—, y se cerró apagando el robot, así
        #    que nunca se supo si faltaban todos los componentes o solo uno.
        #
        #    ⚠️ NO vale mirar `_recibidos` a 1 Hz: `_quiza_publicar` lo VACÍA en
        #    cada ciclo y los componentes llegan asíncronos a 16,5 Hz, así que en
        #    un instante cualquiera está medio lleno **con el robot sano**. Un
        #    `odom_completo` muestreado así diría «incompleto» casi siempre. Lo
        #    que hay que medir es CUÁNTO HACE que se completó uno, no si lo está
        #    ahora mismo.
        #
        #    El discriminante que esto le da a la web es limpio:
        #      antigüedad_muestra ~0  ·  antigüedad_odom CRECE  -> faltan componentes
        #      las dos crecen                                   -> el RVR calló
        self._t_odom_completo: float | None = None

        # ── Origen del yaw ───────────────────────────────────────────────────
        # 🔴 `reset_yaw()` NO PONE A CERO EL YAW. Medido el 2026-07-31: el driver
        # lo llama al arrancar y el cuaternión seguía dando -74.6° en reposo.
        # El yaw solo se pone a cero al ENCENDER el RVR, así que arrastra su
        # origen desde entonces: con un encendido limpio da +0.5°, pero si el
        # robot se ha movido o se ha manipulado, cualquier cosa (+64.9° medido).
        #
        # Consecuencia: la orientación de /odom no concordaba con su propia
        # posición. El desfase NO era constante — paso de ~-15° a -154.9° solo
        # con apagar y encender el robot.
        #
        # Se corrige aquí: se guarda el yaw de la primera muestra tras conectar
        # y se resta de todas. Así `/odom` arranca mirando a 0, que es lo que
        # ROS espera del frame `odom`.
        self._yaw_offset: float | None = None
        #: Último yaw corregido. Lo necesita `_h_velocity` para proyectar la
        #: velocidad del marco del mundo al del robot.
        self._yaw_actual: float = 0.0

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
        # Los dos que faltaban del driver de ROS 1. Van con el MISMO QoS que el
        # resto de la telemetría: son flujos continuos, no estados.
        #
        # 📝 `sensor_msgs/Illuminance` y no `std_msgs/Float32` como en ROS 1:
        #    lleva `header`, así que el dato viene fechado. ⚠️ El campo se llama
        #    `illuminance` y su unidad canónica es el LUX — pero **este valor NO
        #    está calibrado contra ningún patrón**: es lo que reporta el RVR.
        #    Por eso `variance` va a 0.0, que en ROS significa «desconocida», y
        #    no a un número inventado.
        self.pub_luz = self.create_publisher(Illuminance, 'ambient_light', qos_tel)
        # Los encoders son la ÚNICA fuente que no depende del marco de
        # referencia (7792 ticks/m, calibrados contra cinta métrica), así que un
        # flujo continuo vale para depurar la odometría, no solo el servicio.
        self.pub_encoders = self.create_publisher(Encoder, 'encoders', qos_tel)
        # 🆕 2026-08-11 · La otra mitad del IR robot-a-robot. Se podía ENVIAR
        #    (`send_infrared_message`) y no RECIBIR: el tipo estaba definido en
        #    atriz_rvr_msgs desde el principio y no lo publicaba nadie — figuraba
        #    como hueco en la lista del final de este fichero.
        #    ⚠️ NO ES UN STREAM: llega cuando otro robot emite, así que puede
        #       estar en silencio indefinidamente. Un `/infrared_messages` mudo
        #       NO significa que esté roto; significa que nadie ha emitido.
        self.pub_ir = self.create_publisher(InfraredMessage, 'infrared_messages',
                                            qos_tel)
        # 🆕 2026-08-11 · El ESTADO del IR, que es otra cosa que el evento.
        #    Ver EstadoIR.msg y el diseño en
        #    docs/superpowers/specs/2026-08-11-sistema-ir-robot-a-robot-design.md
        self.pub_estado_ir = self.create_publisher(EstadoIR, 'estado_ir', qos_tel)
        self._n_ir_recibidos = 0
        # Estado del IR. Bajo `self._lock`, como el resto.
        self._ir_modo = 'off'
        self._ir_far = 0
        self._ir_near = 0
        self._ir_ultimo_codigo = 0
        self._ir_t_ultimo_mensaje = None      # None = no ha llegado ninguno
        self._ir_crudo = 0
        self._ir_t_lectura = None
        self._ir_lecturas_validas = False
        self._ir_conduciendo = False
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

        # La salud de los motores, con el MISMO QoS que la batería y por la misma
        # razón: llega por eventos —puede pasar media hora sin un solo mensaje—
        # así que un suscriptor que se conecte después tiene que recibir el
        # último estado conocido, no esperar a que se rompa algo.
        #
        # 🔴 TRANSIENT_LOCAL EN EL PUBLICADOR sí añade garantía (guarda el último
        #    mensaje para quien llegue tarde). Es en el SUSCRIPTOR donde no añade
        #    nada y solo restringe — la confusión que costó la parada de
        #    emergencia (manual, cap. 15.1). Un suscriptor VOLATILE, como el de
        #    rosbridge, empareja con este publicador sin problema.
        self.pub_motores = self.create_publisher(
            MotorStatus, 'motor_status',
            QoSProfile(
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        # ── Estado del nodo y del enlace (AÑADIDO 2026-08-04) ────────────────
        # ⚠️ NO VERIFICADO: no ha corrido nunca contra un robot.
        #
        # La señal de vida barata para el muro del profesor: 16 robots vigilados
        # con ~0,5 kB/s en total, contra los 1,7 Mbit/s que cuesta mirar `/odom`.
        # Ver `EstadoRobot.msg` para el porqué de cada campo.
        #
        # QoS: EL MISMO que `/battery_state` y `/motor_status` —RELIABLE +
        # TRANSIENT_LOCAL, depth 1—, y no se replantea aquí: ya se eligió a
        # conciencia y por la misma razón que vale para este topic. Es un ESTADO,
        # no un flujo: al conectarse, la web tiene que recibir el último valor sin
        # esperar un segundo, y perder un mensaje de estado no es como perder un
        # `/odom` (ese se sustituye solo 60 ms después; este no).
        #
        # 📝 TRANSIENT_LOCAL en el PUBLICADOR sí añade garantía. Es en el
        #    SUSCRIPTOR donde no añade nada y solo restringe — la confusión que
        #    costó la parada de emergencia (manual, cap. 15.1). El suscriptor
        #    VOLATILE de rosbridge, que es por donde va a leerlo la web, empareja
        #    con este publicador sin problema.
        self.pub_estado = self.create_publisher(
            EstadoRobot, 'estado_robot',
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

        # ── QoS de la parada de emergencia ───────────────────────────────────
        # RELIABLE: es el único topic del robot donde perder un mensaje es un
        # problema de seguridad.
        #
        # 🔴 Y VOLATILE, NO TRANSIENT_LOCAL. Aquí ponía TRANSIENT_LOCAL «para que
        #    un suscriptor que llegue tarde reciba el último estado» — y ese
        #    razonamiento es del PUBLICADOR, no del suscriptor.
        #
        #    En el suscriptor, TRANSIENT_LOCAL solo RESTRINGE: exige que el
        #    publicador también lo sea. Y ninguno lo es por defecto — ni
        #    `ros2 topic pub`, ni rosbridge, que es por donde hablará la web.
        #
        #    Medido en banco el 2026-07-31, publicando en el topic correcto:
        #      New publisher discovered on topic '/rvr/emergency_stop', offering
        #      incompatible QoS. No messages will be received from it.
        #      Last incompatible policy: DURABILITY
        #
        #    VOLATILE en el suscriptor empareja con TODO: publicadores VOLATILE y
        #    también TRANSIENT_LOCAL. Es estrictamente más compatible.
        #
        # ⚠️ ES LA TERCERA VEZ QUE ESTE BOTÓN FALLA EN SILENCIO, y por tres causas
        #    distintas:
        #      1. ROS 1  — nombre de topic distinto (auditoría, 2026-07-29)
        #      2. ROS 2  — faltaba el namespace `/rvr/` (2026-07-31)
        #      3. ROS 2  — QoS incompatible (2026-07-31, esta)
        #    Las tres daban 200 OK en la web y cero efecto en el robot.
        qos_estop = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            Empty, 'emergency_stop', self._cb_parada_emergencia,
            qos_estop, callback_group=g_cmd,
        )
        # Nombre antiguo, para no romper lo que ya existe.
        self.create_subscription(
            Empty, 'is_emergency_stop', self._cb_parada_emergencia,
            qos_estop, callback_group=g_cmd,
        )

        # 🔴 Y EL QUE LA WEB USA DE VERDAD, con su `/rvr/` y en ABSOLUTO.
        #
        # Los dos de arriba son nombres RELATIVOS: con el namespace vacío (que es
        # el valor por defecto, ver ARQUITECTURA.md D1) resuelven a
        # `/emergency_stop` y `/is_emergency_stop`. La web publica en
        # **`/rvr/emergency_stop`**, así que NINGUNO DE LOS DOS LA OÍA.
        #
        # Comprobado en banco el 2026-07-31, con el driver corriendo:
        #     ros2 topic info /rvr/emergency_stop
        #     Unknown topic '/rvr/emergency_stop'
        #
        # Es la SEGUNDA vez que este botón falla por un nombre. La primera fue en
        # ROS 1: la web publicaba en `/rvr/emergency_stop` y el driver escuchaba
        # `is_emergency_stop` (auditoría, 2026-07-29). Se arregló el nombre y se
        # coló el NAMESPACE.
        #
        # ⚠️ Tres suscripciones para una función es feo, y a propósito: en un
        #    botón de emergencia el modo de fallo es «no llega el mensaje», y ha
        #    fallado dos veces por eso. La Fase 5 reescribe la web sobre
        #    rosbridge y ahí se unifica a UNO — pero no antes de que el nuevo
        #    esté probado de extremo a extremo.
        self.create_subscription(
            Empty, '/rvr/emergency_stop', self._cb_parada_emergencia,
            qos_estop, callback_group=g_cmd,
        )

        self.create_service(
            EmptySrv, 'release_emergency_stop', self._srv_liberar_parada,
            callback_group=g_cmd,
        )

        # ── Servicios que hablan con el RVR ──────────────────────────────────
        # 🔴 GRUPO PROPIO, y no es un detalle: `_pedir()` BLOQUEA esperando la
        # respuesta del robot. Con el MultiThreadedExecutor de este nodo,
        # bloquear en `g_srv` no detiene a los demás grupos — la telemetría y
        # `cmd_vel` siguen atendiéndose. Compartiendo grupo con la telemetría,
        # una llamada lenta al RVR congelaría `/odom`.
        g_srv = MutuallyExclusiveCallbackGroup()
        for tipo, nombre, cb in (
            (SetLEDRGB, 'set_led_rgb', self._srv_led_rgb),
            (SetMultipleLEDs, 'set_multiple_leds', self._srv_leds_multiples),
            (SetLeds, 'set_leds', self._srv_leds_todos),
            (GetRGBCSensorValues, 'get_rgbc_sensor_values', self._srv_rgbc),
            (SetBool, 'enable_color', self._srv_enable_color),
            (GetEncoders, 'get_encoders', self._srv_encoders),
            (GetSystemInfo, 'get_system_info', self._srv_system_info),
            (GetControlState, 'get_control_state', self._srv_control_state),
            (TriggerLedEvent, 'trigger_led_event', self._srv_led_event),
            (SendInfraredMessage, 'send_infrared_message', self._srv_ir_mensaje),
            (SetIRMode, 'set_ir_mode', self._srv_ir_modo),
            (SetIREvading, 'set_ir_evading', self._srv_ir_evasion),
            (SetDriveParameters, 'set_drive_parameters', self._srv_drive_params),
            (SetPosAndYaw, 'set_pos_and_yaw', self._srv_set_pos_yaw),
            # 🔴 Estos CUATRO MUEVEN EL ROBOT. Ver el bloque de abajo.
            (MoveTimed, 'move_timed', self._srv_move_timed),
            (RawMotors, 'raw_motors', self._srv_raw_motors),
            (MoveToPose, 'move_to_pose', self._srv_move_to_pose),
            (MoveToPosAndYaw, 'move_to_pos_and_yaw', self._srv_move_to_pos_yaw),
        ):
            self.create_service(tipo, nombre, cb, callback_group=g_srv)

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

        # ── El grupo de los LATIDOS (AÑADIDO 2026-08-15, evidencia 126) ──────
        # 🔴 Los publicadores puros NO pueden compartir grupo con los sondeos
        #    que BLOQUEAN. Estuvieron todos en `g_salud` (mutuamente
        #    excluyente), y el comentario del latido prometía «tiene que poder
        #    correr AUNQUE la telemetría esté ocupada» — pero con el enlace
        #    caído `_sondear_ir` bloqueaba hasta 6 s por tic dentro del MISMO
        #    grupo, y el latido quedó MEDIDO a ~0,2 Hz (7 mensajes en 35 s
        #    donde promete 1 Hz): el muro pintaba «la Pi calla» sobre una Pi
        #    viva. La intención estaba escrita; el grupo la contradecía.
        #    Aquí van solo callbacks que leen memoria y publican (o encolan
        #    sin esperar): nada de este grupo habla con el puerto serie.
        g_latido = MutuallyExclusiveCallbackGroup()

        # 🔴 REPUBLICAR LA SALUD DE MOTORES A 1 Hz, aunque no haya cambiado.
        #
        #    El publicador es TRANSIENT_LOCAL, así que en teoría «un suscriptor
        #    que llegue tarde recibe el último estado». **Eso resultó NO SER
        #    FIABLE**: medido el 2026-08-01, un suscriptor nuevo se quedaba sin
        #    recibir nada en 10 s **2 de cada 3 veces**, con el topic publicando
        #    perfectamente y en su propio proceso.
        #
        #    Con el sondeo cada 30 s, eso dejaba a la web hasta medio minuto a
        #    ciegas sobre si un motor está en fallo — que es justo el dato que
        #    decide si alguien tiene que ir al edificio.
        #
        #    Republicar es GRATIS: no toca el puerto serie, solo reenvía lo que ya
        #    está en memoria. El SONDEO sigue a 30 s, que es lo que sí cuesta.
        self.create_timer(1.0, self._publicar_motores, callback_group=g_latido)

        # ── El latido de `/estado_robot` (AÑADIDO 2026-08-04) ────────────────
        # ⚠️ NO VERIFICADO contra un robot.
        #
        # Grupo `g_latido` (2026-08-15, antes `g_salud`): este temporizador tiene
        # que poder correr AUNQUE todo lo demás esté ocupado, porque su trabajo es
        # justamente decir que el nodo sigue vivo — y en `g_salud` esa promesa se
        # incumplía: `_sondear_ir` bloqueando en el mismo grupo lo dejó MEDIDO a
        # ~0,2 Hz durante los reintentos (evidencia 126, apartado 4).
        #
        # 🔴 NO SE CONDICIONA A `silence_timeout > 0` ni a `keepalive_period > 0`,
        #    al contrario que los dos de arriba. Desactivar el detector de
        #    silencio es lo que se hace para REPRODUCIR el fallo a propósito, y es
        #    precisamente entonces cuando más falta hace poder ver qué pasa desde
        #    fuera. El latido no le habla al RVR: no puede molestar a nada.
        self.create_timer(1.0, self._publicar_estado, callback_group=g_latido)

        # ── El sondeo del IR (AÑADIDO 2026-08-11) ────────────────────────────
        # Mismo grupo `g_salud` que los otros dos, y por la misma razón: consulta
        # al RVR y bloquea mientras espera, así que no puede compartir grupo con
        # la telemetría.
        _ir_hz = float(self.get_parameter('ir_sondeo_hz').value)
        if _ir_hz > 0.0:
            self.create_timer(1.0 / _ir_hz, self._sondear_ir,
                              callback_group=g_salud)
            self.get_logger().info(
                f'sondeo IR a {_ir_hz:g} Hz -> /estado_ir')
        else:
            # Se publica igual, con `lecturas_validas=False`: así el topic existe
            # y dice explícitamente que no hay lecturas, en vez de desaparecer y
            # dejar al consumidor sin saber si está apagado o roto.
            self.create_timer(1.0, self._publicar_estado_ir,
                              callback_group=g_latido)
            self.get_logger().warn(
                'sondeo IR DESACTIVADO (ir_sondeo_hz=0): /estado_ir publicará '
                'lecturas_validas=False. Los mensajes IR siguen llegando.')

        # El apagado automático de la luz del color. Grupo del LATIDO y no de
        # los sondeos: solo compara números y, cuando toca, ENCOLA un comando
        # sin esperarlo — nunca bloquea, así que no puede molestar al latido.
        if self._color_inactividad > 0 or self._color_max > 0:
            self.create_timer(1.0, self._vigilar_luz_color, callback_group=g_latido)
        else:
            self.get_logger().warn(
                'apagado automático de la luz del color DESACTIVADO: si un '
                'cliente la enciende y desaparece, se queda encendida hasta que '
                'alguien la apague o muera el driver.')

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

    #: Los LEDs del RVR, por índice. El orden es el de `RvrLedGroups`, y el que
    #: usan los servicios `SetLEDRGB` y `SetMultipleLEDs` en su `led_id`.
    #:
    #: 📝 `all_lights` va el ÚLTIMO a propósito: así el índice 0 es una luz
    #:    concreta y no «todas», que sería una sorpresa desagradable para quien
    #:    pruebe con led_id=0.
    LEDS = [
        'headlight_left', 'headlight_right',
        'brakelight_left', 'brakelight_right',
        'status_indication_left', 'status_indication_right',
        'battery_door_front', 'battery_door_rear',
        'power_button_front', 'power_button_rear',
        'undercarriage_white', 'all_lights',
    ]

    def _pedir(self, coro, etiqueta: str, timeout: float = 5.0):
        """Envía una corrutina al loop **y ESPERA su resultado**.

        Es el hermano de `_enviar()`, y existe porque los servicios `Get*` tienen
        que devolver un dato: encolar y olvidarse no vale.

        🔴 ESTO BLOQUEA EL CALLBACK, y por eso los servicios van en su PROPIO
           grupo de callbacks (`g_srv`). Con el `MultiThreadedExecutor` que usa
           este nodo, bloquear en un grupo no detiene a los demás: la telemetría
           y `cmd_vel` siguen atendiéndose. Si los servicios compartieran grupo
           con la telemetría, una llamada lenta al RVR congelaría `/odom`.

        Devuelve `(ok, resultado, mensaje)`. Nunca lanza: un servicio que
        revienta deja al cliente esperando, y eso es peor que un `success=False`.
        """
        if self._rvr is None:
            return False, None, 'el RVR no está conectado'
        try:
            fut = asyncio.run_coroutine_threadsafe(coro, self._loop)
        except RuntimeError as e:
            self.get_logger().error(f'{etiqueta}: no se pudo encolar: {e}')
            return False, None, f'no se pudo encolar: {e}'
        try:
            return True, fut.result(timeout), ''
        except concurrent.futures.TimeoutError:
            fut.cancel()
            self.get_logger().error(f'{etiqueta}: el RVR no contestó en {timeout:.0f} s')
            return False, None, f'el RVR no contestó en {timeout:.0f} s'
        except Exception as e:
            self.get_logger().error(f'{etiqueta} FALLÓ: {type(e).__name__}: {e}')
            return False, None, f'{type(e).__name__}: {e}'

    @staticmethod
    def _rgb_valido(*valores) -> bool:
        """Los canales van de 0 a 255. El SDK no lo comprueba y el RVR tampoco."""
        return all(isinstance(v, int) and 0 <= v <= 255 for v in valores)

    @staticmethod
    def _codigo_ir_valido(v) -> bool:
        """Los códigos IR van de 0 a 7 (`InfraredCodes`, ocho valores).

        🔴 Lo comprueba ESTE driver porque no lo comprueba nadie más: ni el
           cliente del SDK, ni la capa de protocolo, ni el firmware devuelve
           error. Medido leyendo el SDK entero el 2026-08-11.
        📝 El registro de LECTURA admite 0-15 (referencia_sdk/sensor.md:54). Esto
           valida lo que se EMITE, que es lo que la documentación acota a 0-7.
        """
        return isinstance(v, int) and 0 <= v <= 7

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

            # Los umbrales de batería del FIRMWARE, una vez y al log. La web los
            # necesita para interpretar el `voltage` de `/battery_state`, y así
            # no se codifican a mano en dos sitios: se leen de donde son verdad.
            # Medidos en rvr-01: low 7.0 V · critical 6.5 V · histéresis 0.2 V.
            # 🔴🔴 `timeout` EXPLÍCITO, Y NO ES OPCIONAL. El SDK usa `timeout=None`
            #    por defecto, que en `asyncio.wait_for` significa **esperar para
            #    siempre**. Y esta es la PRIMERA llamada del arranque que espera
            #    respuesta: `wake`, `reset_yaw` y `reset_locator_x_and_y` no la
            #    piden (sus comandos no declaran `outputs`).
            #
            #    Sin tope, con un RVR que no contesta, `_conectar_rvr` se queda
            #    parado aquí: nunca llega a `sensor_control.start()`,
            #    `_streaming_activo` sigue en False, **el detector de silencio se
            #    autodesactiva**, y el `except` de abajo no salta porque un
            #    cuelgue no es una excepción. Resultado: nodo vivo, topics
            #    registrados, CERO datos y CERO errores — el peor modo de fallo
            #    de este proyecto, reintroducido por código escrito para
            #    observabilidad. Encontrado en auditoría el 2026-08-01.
            try:
                u = await self._rvr.get_battery_voltage_state_thresholds(timeout=5.0)
                self.get_logger().info(
                    f"umbrales de batería (firmware): baja {u['low_threshold']:.2f} V · "
                    f"crítica {u['critical_threshold']:.2f} V · "
                    f"histéresis {u['hysteresis']:.2f} V")
            except Exception as e:                              # noqa: BLE001
                self.get_logger().warn(f'sin umbrales de batería: {e}')

            await self._registrar_sensores()
            await self._registrar_notificaciones()
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
        # Y de paso se SONDEA la salud de los motores: en este firmware las
        # notificaciones no llegan (ver `_sondear_motores`), así que preguntar es
        # la única vía. Publica al terminar.
        self._enviar(self._sondear_motores(), 'sondeo de motores')

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
        # 🔴 TIMEOUT AQUI TAMBIEN, y esta es la linea que importa: es la PRIMERA
        #    del keepalive. `get_battery_percentage` declara `outputs`, asi que
        #    espera respuesta — y sin tope esperaba PARA SIEMPRE.
        #
        #    Con el RVR mudo se colgaba aqui y **las lecturas de voltaje y estado
        #    de abajo no llegaban a ejecutarse nunca**: sus `timeout=3.0` eran
        #    irrelevantes. `/battery_state` dejaba de publicarse entero y, al ser
        #    TRANSIENT_LOCAL, un suscriptor tardio recibia el ultimo voltaje bueno
        #    *latched* — exactamente el daño que el comentario de abajo dice
        #    evitar. Y sin una linea en el log: `_enviar` registra por
        #    `add_done_callback`, y ese future no termina nunca.
        #
        #    Preexistente (2026-07-30), pero encontrado el 2026-08-01 en una
        #    auditoria adversarial de los arreglos de ESE MISMO DIA. 📝 Arreglar
        #    dos de tres llamadas deja el fallo intacto: **la mas temprana manda**.
        resp = await self._rvr.get_battery_percentage(timeout=3.0)

        # ── Voltaje y estado, del propio firmware ────────────────────────
        # 🔴 POR QUÉ NO BASTA EL PORCENTAJE: medido el 2026-08-01, decía **100 %**
        #    con la batería a **8.29 V**, a 1.29 V del umbral de «low». El
        #    porcentaje es una estimación gruesa; el voltaje y el estado son lo
        #    que el firmware usa para decidir, con su propia histéresis.
        #    Con 16 robots, «¿cuál se está quedando sin carga?» se responde con
        #    esto, no con el porcentaje. Evidencia 43.
        #
        # 📝 Son dos lecturas más en la MISMA pasada del keepalive, así que no
        #    cuestan un viaje extra al puerto serie ni despiertan nada.
        voltios = estado = None
        try:
            # 🔴 Con `timeout=None` un RVR mudo dejaría esta corrutina esperando
            #    para siempre, y **no se publicaría `/battery_state` en absoluto**
            #    — ni siquiera el porcentaje, que ya se leyó bien más arriba.
            #    Y el topic es TRANSIENT_LOCAL: un suscriptor tardío recibiría el
            #    último voltaje bueno *latched*, y la web tiene instrucción de
            #    mirar `voltage`.
            #    3 s: de sobra para una lectura, y muy por debajo de los 30 s del
            #    keepalive, así que dos pasadas no se solapan.
            r = await self._rvr.get_battery_voltage_in_volts(
                reading_type=BatteryVoltageReadingTypesEnum.calibrated_and_filtered,
                timeout=3.0)
            voltios = float(r['voltage'])
        except Exception as e:                                  # noqa: BLE001
            # 🔴 NO se usa `_avisar_una_vez`: su set de claves no se vacía nunca,
            #    así que un fallo transitorio al arrancar quemaría la clave y un
            #    fallo PERMANENTE después sería invisible — `voltage` volvería a
            #    NaN cada 30 s en silencio. Se avisa en cada TRANSICIÓN.
            if self._bat_v_ok:
                self.get_logger().warn(f'sin voltaje de batería: {e}')
            self._bat_v_ok = False
        else:
            if not self._bat_v_ok:
                self.get_logger().info('voltaje de batería recuperado')
            self._bat_v_ok = True
        try:
            r = await self._rvr.get_battery_voltage_state(timeout=3.0)
            estado = int(r['state'])
        except Exception as e:                                  # noqa: BLE001
            if self._bat_s_ok:
                self.get_logger().warn(f'sin estado de batería: {e}')
            self._bat_s_ok = False
        else:
            if not self._bat_s_ok:
                self.get_logger().info('estado de batería recuperado')
            self._bat_s_ok = True

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
        # BatteryState usa NaN para «no medido», y poner ceros ahí mentiría:
        # 0.0 V es un dato, no un hueco. El RVR no da amperios
        # (`get_current_sense_amplifier_current` responde `bad_cid`, evidencia 41).
        msg.voltage = voltios if voltios is not None else float('nan')
        msg.current = float('nan')
        # Coherencia con el criterio de arriba: el RVR no da temperatura de
        # BATERIA (la de `/motor_status` es de los motores), así que NaN, no 0.0.
        msg.temperature = float('nan')
        msg.charge = float('nan')
        msg.capacity = float('nan')
        msg.design_capacity = float('nan')
        msg.percentage = pct / 100.0          # BatteryState lo quiere en 0.0–1.0
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        # Estados del firmware: 0 unknown · 1 ok · 2 low · 3 critical.
        #
        # ⚠️ `power_supply_health` NO PUEDE EXPRESAR «low», y no se fuerza: una
        #    batería con poca carga está SANA, no averiada. Marcarla como DEAD
        #    engañaría a cualquier consumidor.
        # 🔴 **La señal autoritativa para la web es `voltage`**, comparada con los
        #    umbrales del firmware (low 7.0 V, critical 6.5 V), que se registran
        #    en el log al arrancar. Aquí solo se refleja lo que el campo admite.
        msg.power_supply_health = {
            None: BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN,
            0: BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN,
            1: BatteryState.POWER_SUPPLY_HEALTH_GOOD,
            2: BatteryState.POWER_SUPPLY_HEALTH_GOOD,   # «low» ≠ enferma
            3: BatteryState.POWER_SUPPLY_HEALTH_DEAD,   # crítica: va a apagarse
        }.get(estado, BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN)
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        msg.present = True
        self.pub_bateria.publish(msg)

        # 🔴 El aviso se da al CAMBIAR de estado, no en cada lectura: un WARN cada
        #    30 s durante media hora es ruido que se acaba ignorando. Y la
        #    histéresis (0.2 V) la aplica el FIRMWARE, así que no rebota sola.
        if estado is not None and estado != self._estado_bateria:
            nombre = {0: 'desconocido', 1: 'ok', 2: 'BAJA', 3: 'CRÍTICA'}.get(estado, '?')
            v = f'{voltios:.2f} V' if voltios is not None else 'sin voltaje'
            if estado == 3:
                self.get_logger().error(f'BATERÍA {nombre} ({v}) — el RVR se apagará')
            elif estado == 2:
                self.get_logger().warn(f'batería {nombre} ({v}) — toca cargar')
            else:
                self.get_logger().info(f'estado de batería: {nombre} ({v})')
            self._estado_bateria = estado

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
            # ── AÑADIDO 2026-08-04 · el reseteo de `reanudaciones_fallidas` ───
            # ⚠️ NO VERIFICADO contra un robot.
            #
            # 🔴 EL ÉXITO ES QUE LLEGUE UN DATO, NO QUE LA LLAMADA VUELVA. Por eso
            #    se compara `_n_muestras` —que solo tocan los handlers— contra la
            #    foto que se hizo al lanzar el último intento, y NO se mira si
            #    `_recuperar_streaming` terminó sin excepción: con el RVR apagado,
            #    `wake`+`stop`+`start` **no lanzan**, y por eso el log escribió
            #    «streaming reanudado» 8 veces en 30 s con /odom a cero
            #    (2026-08-02, evidencia 52). Ese es el fallo que este contador
            #    existe para hacer visible; darlo por bueno aquí sería
            #    reproducirlo dentro de su propio detector.
            #
            # Va ANTES del `return` de abajo a propósito: el camino normal —el
            # robot va bien y hay silencio de sobra— es justamente el que tiene
            # que poner el contador a cero.
            if self._n_muestras > self._muestras_al_reanudar:
                # ── EL «REANUDADO» HONESTO VIVE AQUÍ (2026-08-14, ev. 116) ───
                # Antes lo imprimía `_recuperar_streaming` cuando wake+stop+
                # start no lanzaban excepción — que con el RVR APAGADO es
                # siempre: 8 «streaming reanudado» en 30 s con /odom a cero
                # (evidencia 52). El único sitio que puede decir «volvió» es
                # este: donde se comprueba que LLEGÓ UNA MUESTRA.
                if self._reanudaciones_fallidas > 0:
                    self.get_logger().info(
                        f'el RVR VOLVIÓ: primera muestra tras '
                        f'{self._reanudaciones_fallidas} intento(s) de '
                        f'reanudación sin respuesta.'
                    )
                self._reanudaciones_fallidas = 0
                self._no_reanudar_antes_s = 0.0
        if silencio < self._timeout_silencio:
            return
        # ── ESPERA CRECIENTE (2026-08-14, evidencia 116) ─────────────────────
        # timeout × 2^fallidas, con tope de 60 s: 3, 6, 12, 24, 48, 60, 60…
        # Un RVR dormido de verdad se recupera al PRIMER intento (medido), así
        # que la espera solo crece en el caso que no tiene arreglo automático:
        # el RVR apagado. De ~14.000 intentos/día a ~1.440 líneas en el peor
        # caso, y el journal deja de ahogar los errores de verdad.
        if self._ahora_s() < self._no_reanudar_antes_s:
            return

        # AÑADIDO 2026-08-04 · se cuenta el intento y se fotografía el número de
        # muestras. Si dentro de un rato `_n_muestras` no ha subido de aquí, este
        # intento no sirvió de nada y el contador lo dirá.
        with self._lock:
            self._reanudaciones_fallidas += 1
            self._muestras_al_reanudar = self._n_muestras
            fallidas = self._reanudaciones_fallidas
            espera = min(60.0, self._timeout_silencio * (2.0 ** fallidas))
            self._no_reanudar_antes_s = self._ahora_s() + espera

        self._n_recuperaciones += 1
        if fallidas <= 1:
            self.get_logger().warn(
                f'el RVR lleva {silencio:.1f} s sin enviar telemetría '
                f'(se esperan ~{1000 / self._intervalo_ms:.1f} muestras/s). '
                'Lo más probable es que se haya dormido. Intentando reanudar '
                f'(intento nº {self._n_recuperaciones})…'
            )
        else:
            # A partir del segundo intento sin respuesta, el diagnóstico
            # honesto ya no es «se durmió»: un RVR dormido vuelve al primer
            # intento (medido, 3 de 3). Esto es un RVR apagado, cargando o
            # con el cable fuera — y se dice, con la próxima cita anotada.
            self.get_logger().warn(
                f'sigue SIN llegar ni una muestra del RVR tras {fallidas} '
                f'intentos: apagado, cargando o el cable fuera. Próximo '
                f'intento en {espera:.0f} s.'
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
            # 🔴 AQUÍ DECÍA «streaming reanudado» — Y ERA MENTIRA (ev. 52/116):
            #    con el RVR apagado, wake+stop+start NO lanzan, así que esto se
            #    imprimía cada ~6 s para siempre sobre un robot muerto. El
            #    «volvió» de verdad lo imprime `_vigilar_silencio` cuando LLEGA
            #    una muestra — el único testigo que no puede mentir.
            self.get_logger().debug(
                'reanudación enviada sin excepción — eso NO prueba nada: el '
                'éxito es que llegue una muestra.'
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
        # ── EL SENSOR DE COLOR ───────────────────────────────────────────────
        # El parámetro solo decide el estado INICIAL. Encender y apagar en
        # caliente funciona: lo hace el servicio `enable_color`.
        #
        # 🔴 CORREGIDO EL 2026-08-06. Aquí decía, desde el 2026-07-31, que «con
        #    el streaming ya configurado, `enable_color_detection` NO HACE NADA
        #    — 481 mensajes de /color, todos ceros». **Es falso, y la medida que
        #    lo sostenía no probaba eso**: el servicio bajo prueba se apagaba a
        #    sí mismo dentro de la misma llamada, así que casi todos aquellos
        #    mensajes eran posteriores al `enable(False)`.
        #    Remedido con el streaming corriendo a 250 ms: no-cero 0/24 → 24/24,
        #    canal claro 1 → 1321, y vuelta a 1 al apagar. Ver
        #    `mediciones_banco/probar_color_stream_caliente.py`.
        #
        # 📝 Lo que sí sigue siendo cierto: el sensor SIN su luz no da nada, y
        #    `/color` publica [0,0,0] mientras esté apagado — no calla. Un topic
        #    que habla no significa un sensor que mide.
        #
        # ⚠️ Por defecto APAGADO, y no por cautela: encenderlo deja un LED blanco
        #    bajo el chasis mientras siga encendido, y eso gasta batería en un
        #    laboratorio de 16 robots. Quien quiera el color lo pide.
        if self._color_detection:
            await self._rvr.enable_color_detection(is_enabled=True)
            self.get_logger().info(
                'sensor de color ENCENDIDO: /color dará valores reales. '
                'Deja un LED blanco encendido bajo el chasis mientras el driver viva.')
        else:
            self.get_logger().warn(
                '/color publicará [0, 0, 0]: el sensor de color está APAGADO. '
                'Actívalo EN CALIENTE con el servicio enable_color(true), o al '
                'arrancar con color_detection:=true — enciende un LED blanco bajo '
                'el chasis y gasta batería. (/ambient_light NO depende de esto: '
                'es otro sensor, en otro sitio, y funciona igual.)')

        sc = self._rvr.sensor_control
        await sc.add_sensor_data_handler(RvrStreamingServices.locator, self._h_locator)
        await sc.add_sensor_data_handler(RvrStreamingServices.quaternion, self._h_quaternion)
        await sc.add_sensor_data_handler(RvrStreamingServices.gyroscope, self._h_gyroscope)
        await sc.add_sensor_data_handler(RvrStreamingServices.velocity, self._h_velocity)
        await sc.add_sensor_data_handler(RvrStreamingServices.accelerometer, self._h_accel)
        await sc.add_sensor_data_handler(RvrStreamingServices.color_detection, self._h_color)
        await sc.add_sensor_data_handler(RvrStreamingServices.ambient_light, self._h_luz)
        await sc.add_sensor_data_handler(RvrStreamingServices.encoders, self._h_encoders)

    async def _registrar_notificaciones(self) -> None:
        """Notificaciones por EVENTO del RVR: atasco, fallo y térmica de motores.

        🔴 POR QUÉ ESTO IMPORTA MÁS QUE PARECE
           Hasta el 2026-08-01 un robot con una oruga trabada se veía IGUAL que
           uno que navegaba mal: `/odom` avanzaba poco, Nav2 abortaba por «no
           progresar», y nada distinguía un fallo de hardware de uno de
           algoritmo. Con 16 robots en otro edificio, esa diferencia decide si
           alguien tiene que ir hasta allí.

        ⚠️ NO ES UN STREAM. No van por `sensor_control` ni tienen intervalo: el
           RVR envía un mensaje **cuando pasa algo**. Puede estar media hora en
           silencio con todo bien. Por eso el estado se guarda y se republica, y
           por eso el mensaje lleva `antiguedad_*_s`.

        📝 Fallar aquí NO debe tumbar el driver. Si el firmware no soporta alguna
           de estas notificaciones, se avisa y se sigue: la telemetría y la
           conducción no dependen de esto.
        """
        try:
            # 🔴 `on_*_notify` NO espera la notificación: devuelve una **Task** que
            #    se queda escuchando. Si esa task revienta, la excepción se queda
            #    dentro del objeto Task y NO SE IMPRIME EN NINGÚN SITIO — el
            #    driver seguiría diciendo «registradas» sobre algo muerto.
            #    Por eso se guardan y se inspeccionan.
            tareas = {
                'atasco': await self._rvr.on_motor_stall_notify(
                    handler=self._h_motor_atasco),
                'fallo': await self._rvr.on_motor_fault_notify(
                    handler=self._h_motor_fallo),
                'térmica': await self._rvr.on_motor_thermal_protection_status_notify(
                    handler=self._h_motor_termico),
            }
            # 🔴 SE REGISTRA LA RESPUESTA DE CADA `enable_*`, no solo que no
            #    lance excepción. El SDK devuelve un dict de error en vez de
            #    lanzar cuando el firmware rechaza un comando, así que un
            #    `try/except` a secas daría «activas» sobre algo que el RVR ha
            #    dicho que no. Es el mismo patrón que ya escondió tres fallos de
            #    la parada de emergencia: `200 OK` y nada funcionando.
            for nombre, coro in (
                ('atasco',  self._rvr.enable_motor_stall_notify(is_enabled=True)),
                ('fallo',   self._rvr.enable_motor_fault_notify(is_enabled=True)),
                ('térmica', self._rvr.enable_motor_thermal_protection_status_notify(
                    is_enabled=True)),
            ):
                r = await coro
                self.get_logger().info(f'  enable {nombre}: respuesta = {r!r}')
            # 📝 Estas tasks TERMINAN enseguida, y es lo normal: `on_command` del
            #    SDK solo mete el manejador en la tabla de despacho
            #    (`add_command_worker`) y vuelve. Se comprobó el 2026-08-01 —una
            #    versión de este código avisaba de «la task ya terminó» como si
            #    fuera un fallo, y era un FALSO POSITIVO.
            self._tareas_notify = tareas
            # Se guardan igualmente: si alguna guardara una excepción, se pierde
            # en silencio, y esto deja dónde mirarlo.
            for nombre, tarea in tareas.items():
                if tarea is not None and tarea.done() and tarea.exception():
                    self.get_logger().error(
                        f'  🔴 {nombre}: excepción al registrar: {tarea.exception()!r}')
            self.get_logger().info(
                'notificaciones de motor registradas -> /motor_status')
        except Exception:                                   # noqa: BLE001
            self.get_logger().warn(
                'no se pudieron registrar las notificaciones de motor; se seguirá '
                f'sondeando igualmente:\n{traceback.format_exc()}')

        # ── 🆕 2026-08-11 · Mensajes IR de otro robot -> /infrared_messages ───
        #
        # Va en su PROPIO try/except y no en el bloque de arriba a propósito: si
        # el firmware rechaza esto, las notificaciones de motor no tienen por qué
        # caer con él. Son cosas distintas que solo comparten el ser por evento.
        #
        # 🔴 Se registra la RESPUESTA del `enable_*`, igual que arriba y por el
        #    mismo motivo: el SDK devuelve un dict de error en vez de lanzar
        #    cuando el firmware rechaza un comando, así que un try/except a secas
        #    diría «activada» sobre algo que el RVR ha dicho que no.
        #
        # ⚠️ Que esto registre bien NO significa que vayan a llegar mensajes. Las
        #    de motor se registran igual de bien y NO llegan en este firmware.
        #    Quien quiera saberlo, que mire el contador de `_h_ir_mensaje`: solo
        #    un mensaje recibido de verdad lo demuestra.
        try:
            r = await self._rvr.enable_robot_infrared_message_notify(is_enabled=True)
            self.get_logger().info(f'  enable IR recibido: respuesta = {r!r}')
            self._tarea_notify_ir = \
                await self._rvr.on_robot_to_robot_infrared_message_received_notify(
                    handler=self._h_ir_mensaje)
            self.get_logger().info(
                'notificaciones IR robot-a-robot registradas -> /infrared_messages '
                '(en silencio mientras nadie emita: NO es un fallo)')
        except Exception:                                   # noqa: BLE001
            self._tarea_notify_ir = None
            self.get_logger().warn(
                'no se pudieron registrar las notificaciones IR; se seguirá '
                f'pudiendo ENVIAR IR, pero no recibir:\n{traceback.format_exc()}')

        # 🔴 LAS NOTIFICACIONES NO LLEGAN EN ESTE FIRMWARE — medido, ver
        #    `_sondear_motores`. Por eso el estado real se consigue SONDEANDO,
        #    no esperando, y este primer sondeo llena `/motor_status` con datos
        #    de verdad desde el arranque en vez de con el valor inicial.
        await self._sondear_motores()


    async def _sondear_motores(self) -> None:
        """Pregunta al RVR por el fallo y la térmica de los motores.

        🔴 POR QUÉ SE SONDEA EN VEZ DE ESCUCHAR — medido el 2026-08-01

           El SDK ofrece `enable_motor_*_notify` + `on_motor_*_notify`, y este
           driver las registra correctamente. **No llega ni una.** Se comprobó:

             · registro correcto: `on_command` mete el handler en la tabla de
               despacho del SDK y vuelve (la task terminando es NORMAL).
             · `enable_*` no da error — devuelven `None`, que en este SDK
               significa «comando sin respuesta pedida», no «fallo».
             · motores forzados a 220/255 con el robot sujeto: nada.
             · 100 s esperando la térmica, que debería llegar sola: nada.
               4 mensajes, los 4 del keepalive, todos con antigüedad -1.

           Es el mismo caso que `core_time`: está en el SDK y **el firmware no
           lo transmite**. Precedente documentado en CLAUDE.md.

           Las consultas directas, en cambio, **SÍ responden**:
               get_motor_fault_state                 -> {'is_fault': False}
               get_motor_thermal_protection_status   -> 28.0 °C / 27.5 °C

        ⚠️ EL ATASCO SE QUEDA FUERA. El SDK **no tiene** un `get_motor_stall_state`:
           solo existe por notificación, y la notificación no llega. Así que
           `atascado_*` seguirá en `false` con `antiguedad_atasco_s = -1` mientras
           este firmware no cambie — y ese `-1` es justo lo que impide leerlo como
           «no hay atasco».

        📝 Cuesta dos lecturas por el puerto serie cada 30 s, junto al keepalive.
           Son lecturas: no cambian nada del robot.
        """
        if self._rvr is None:
            return
        try:
            f = await asyncio.wait_for(self._rvr.get_motor_fault_state(), timeout=5.0)
            if isinstance(f, dict):
                act = bool(f.get('is_fault', False))
                with self._lock:
                    cambio = self._motor_fallo != act
                    self._motor_fallo = act
                    self._t_ultimo_fallo = self._ahora_s()
                if cambio and act:
                    self.get_logger().error(
                        '🔴 FALLO ELÉCTRICO DE MOTOR. Más grave que un atasco: '
                        'NO se resuelve levantando el robot.')
                elif cambio:
                    self.get_logger().warn('fallo de motor: resuelto')
        except Exception as e:                              # noqa: BLE001
            self._avisar_una_vez('sondeo_fallo', f'get_motor_fault_state falló: {e!r}')

        try:
            d = await asyncio.wait_for(
                self._rvr.get_motor_thermal_protection_status(), timeout=5.0)
            if isinstance(d, dict):
                with self._lock:
                    self._motor_temp = [
                        float(d.get('left_motor_temperature', 0.0)),
                        float(d.get('right_motor_temperature', 0.0)),
                    ]
                    prev = list(self._motor_estado_term)
                    self._motor_estado_term = [
                        int(d.get('left_motor_status', 0)),
                        int(d.get('right_motor_status', 0)),
                    ]
                    nuevo = list(self._motor_estado_term)
                    temps = list(self._motor_temp)
                    self._t_ultimo_termico = self._ahora_s()
                if nuevo != prev and any(nuevo):
                    self.get_logger().warn(
                        f'⚠️ protección térmica activa: estado izq={nuevo[0]} '
                        f'der={nuevo[1]} · {temps[0]:.1f} °C / {temps[1]:.1f} °C. '
                        'El firmware limita la potencia: el robot navegará mal.')
        except Exception as e:                              # noqa: BLE001
            self._avisar_una_vez('sondeo_term',
                                 f'get_motor_thermal_protection_status falló: {e!r}')
        self._publicar_motores()

    # ─────────────────────────────────────────────────────────────────────────
    # Handlers del SDK. Corren en el hilo del event loop, así que todo lo que
    # toque estado compartido va bajo el lock.
    #
    # 🔴🔴 EL RVR NO USA UNA SOLA CONVENCIÓN DE EJES. HAY QUE MIRAR CADA SENSOR.
    #
    #   RVR en FRD:  x adelante · y a la DERECHA · z ABAJO
    #   ROS en FLU:  x adelante · y a la IZQUIERDA · z ARRIBA    <- REP-103
    #
    # Pasar de uno a otro es un giro de 180° alrededor del eje x: sobre un
    # vector (x, y, z) -> (x, -y, -z), y sobre un cuaternión de orientación
    # (x, y, z, w) -> (x, -y, -z, w).
    #
    # ⚠️ PERO SOLO DOS DE LOS CUATRO SENSORES LO NECESITAN. Medido uno a uno el
    #    2026-07-31, después de que aplicarlo "por analogía" a los cuatro
    #    rompiera los otros dos:
    #
    #      cuaternión     FRD -> hay que invertir (x, -y, -z, w)
    #      locator        FRD -> hay que invertir la Y
    #      giroscopio     YA EN FLU -> solo deg/s a rad/s
    #      acelerómetro   YA EN FLU -> solo g a m/s²
    #
    #    Comprobarlo cuesta un giro y una lectura en reposo. Suponerlo costó
    #    publicar la gravedad apuntando al techo y un giroscopio que contradecía
    #    a la orientación de su propio mensaje /imu.
    #
    # Hasta el 2026-07-31 el driver los copiaba CRUDOS, y eso ponía el yaw de
    # `/odom` con el signo al revés. Consecuencia: `/scan` y `/odom` decían que
    # el robot giraba en sentidos CONTRARIOS, y SLAM tiraba de los dos a la vez.
    # El mapa salía espejado o emborronado — y coherente consigo mismo, así que
    # MIRARLO no lo detectaba.
    #
    # CÓMO SE MIDIÓ, porque adivinarlo no vale (2026-07-31):
    #
    #   1. `verificar_inverted_lidar.py` giró el robot y correlacionó el barrido
    #      de antes con el de después: el patrón se desplazó -47° mientras la
    #      odometría decía -47°. La física exige signos OPUESTOS, así que uno de
    #      los dos estaba mal — pero eso solo no dice CUÁL.
    #   2. El SDK documenta `yaw_angular_velocity` con la regla de la mano
    #      derecha (positivo = antihorario) y el driver pasa `angular.z` sin
    #      tocarlo.
    #   3. Se mandó un giro positivo y SE MIRÓ EL ROBOT: giró a la IZQUIERDA.
    #      -> el SDK cumple, el giro real fue +47°, y el barrido (-47°) era el
    #         correcto. El equivocado era el yaw de `/odom`.
    #   4. Confirmado por segunda vía: al curvar a la izquierda, la `y` del
    #      locator salía NEGATIVA. Los dos signos invertidos, una sola causa.
    #
    # ⚠️ EL COMANDO `cmd_vel` NO SE TOCA. `drive_rc_si_units` ya es de mano
    #    derecha (comprobado mirando el robot), así que `angular.z` va tal cual.
    #    La conversión FRD->FLU es solo de SENSORES a ROS.
    #
    # ⚠️ Y `inverted: true` del YDLIDAR ES CORRECTO. No lo toques: el LIDAR
    #    nunca fue el problema, aunque lo pareciera.
    # ─────────────────────────────────────────────────────────────────────────
    async def _h_locator(self, datos) -> None:
        """Posición. 🔴 El marco del locator está 90° GIRADO respecto al robot.

        Medido el 2026-07-31 con cinco pruebas (`15_velocidad_odom.txt`):

          · El marco del locator es FIJO —no gira con el robot— y se REALINEA
            en cada `reset_locator_x_and_y()`, o sea al arrancar el driver.
          · Su eje X queda **90° girado** respecto al «adelante» del robot: por
            eso avanzar en línea recta daba SIEMPRE -90°, con giros y apagados
            de por medio.

        Y el marco crudo del locator **ya es dextrógiro y con ángulos positivos
        en sentido antihorario**, coherente con el yaw. Se comprobó girando el
        robot con `cmd_vel`: la dirección de avance cambió +88.8° en el marco
        crudo mientras el robot giraba +90° en antihorario.

        ⚠️ El `-Y` que había aquí SOBRABA, y hacía que la posición y la
           orientación de `/odom` tuvieran MANOS CONTRARIAS: girando el robot,
           el yaw publicado cambiaba +89.4° y el desplazamiento -88.8°.

           Vino de una inferencia inválida: se dedujo midiendo que «al curvar a
           la izquierda, `dy` salía negativo», dando por hecho que el eje X del
           locator apuntaba hacia ADELANTE. Está 90° girado, así que ese
           razonamiento no valía. Ahora hay medida directa.

        LA TRANSFORMACIÓN: rotar -90° para alinear X con el «adelante» inicial,
        que es lo que ROS espera del frame `odom`.

            R(-90°)·(x, y) = (y, -x)
        """
        loc = datos['Locator']
        x_crudo, y_crudo = float(loc['X']), float(loc['Y'])
        with self._lock:
            self._odom.pose.pose.position.x = y_crudo
            self._odom.pose.pose.position.y = -x_crudo
            self._odom.pose.pose.position.z = 0.0
        self._quiza_publicar('locator')

    async def _h_quaternion(self, datos) -> None:
        q = datos['Quaternion']
        with self._lock:
            # 🔴 (x, -y, -z, w): FRD -> FLU. Sin esto el yaw sale al revés y
            # SLAM pelea contra su propio emparejado de barridos.
            crudo = Quaternion(
                x=float(q['X']), y=-float(q['Y']), z=-float(q['Z']), w=float(q['W'])
            )
            roll, pitch, yaw = euler_desde_cuaternion(crudo)

            # 🔴 Restar el yaw del arranque. Ver `self._yaw_offset`.
            #
            # Se toma de la PRIMERA muestra que llega tras conectar, no de una
            # constante: el origen del yaw depende de cuánto se haya movido el
            # robot desde que se encendió, y eso no se puede saber de antemano.
            #
            # 🔴 SE CONSERVAN roll y pitch, PERO LA PREMISA CAMBIÓ (2026-07-31).
            #
            # Aquí ponía «el robot está inclinado ~8° y esa inclinación es real».
            # Se apoyaba en tres confirmaciones supuestamente independientes:
            # el árbol TF, el Roll de la IMU y el acelerómetro.
            #
            # NO SON INDEPENDIENTES. El TF toma su rotación de esta misma línea,
            # esta línea sale del cuaternión del RVR, el cuaternión lo calcula la
            # IMU, y el acelerómetro es el MISMO chip. Es UNA sola fuente contada
            # tres veces.
            #
            # Y el 2026-07-31 el usuario midió con una regla que el disco del
            # LIDAR está a la MISMA altura en los cuatro puntos —delante, detrás,
            # izquierda, derecha— respecto al suelo. Es decir: **el robot está
            # físicamente horizontal**, y los ~8° son un desvío de la IMU.
            #
            # ⏳ CONSECUENCIA, SIN APLICAR TODAVÍA: esto publica un roll falso de
            #    ~8° en `/odom` y en TF. Un roll en `odom -> base_footprint`
            #    inclina el plano del láser, y comprime los alcances por cos(8°) =
            #    0.990: un 1 %, ~1 cm por metro. Cabe dentro de la deriva medida
            #    de SLAM (1-3 cm), así que **podría ser parte de ella**.
            #
            #    La corrección sería `roll = pitch = 0.0`. NO se aplica sin
            #    medirla: hay que repetir la caracterización de deriva con y sin,
            #    y comparar. Manual, cap. 13.
            if self._yaw_offset is None:
                self._yaw_offset = yaw
                self.get_logger().info(
                    f'origen del yaw fijado en {math.degrees(yaw):+.1f}° '
                    '(reset_yaw() del RVR no lo pone a cero; se resta aquí)'
                )
            yaw -= self._yaw_offset
            yaw = math.atan2(math.sin(yaw), math.cos(yaw))   # normaliza a ±pi

            self._yaw_actual = yaw
            if not self._publicar_inclinacion:
                # 🔬 `publicar_inclinacion:=false` — para MEDIR, no para usar a
                # ciegas. Publica la orientación plana que REP-105 espera de
                # `odom -> base_footprint`, en vez del roll de ~8° de la IMU que
                # no corresponde a ninguna inclinación real del robot.
                roll = 0.0
                pitch = 0.0
            orient = cuaternion_desde_euler(roll, pitch, yaw)
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
            # deg/s -> rad/s (REP-103). Y **NADA MÁS**: el giroscopio SÍ viene
            # ya en FLU, al contrario que el cuaternión y el locator.
            #
            # Medido el 2026-07-31 girando en sentido antihorario:
            #     gyro.z crudo = +0.433 rad/s   -> antihorario positivo = FLU ✅
            # Aplicarle el (x,-y,-z) del cuaternión lo dejaba en -0.433, o sea
            # contradiciendo a la orientación del propio mensaje /imu.
            angular = Vector3(
                x=float(g['X']) * rad,
                y=float(g['Y']) * rad,
                z=float(g['Z']) * rad,
            )
            self._odom.twist.twist.angular = angular
            self._imu.angular_velocity = angular
        self._quiza_publicar('gyroscope')

    async def _h_velocity(self, datos) -> None:
        """Velocidad, pasada del marco del MUNDO al del ROBOT.

        🔴 `odom.twist` va expresado en `child_frame_id`, o sea en el marco del
        ROBOT: avanzar da `linear.x` positivo SIEMPRE, mire donde mire.

        El RVR la reporta en el marco del LOCATOR, que es el del mundo. Y el
        stream es **EXACTO** — medido el 2026-07-31 con el robot avanzando recto:

            dirección del desplazamiento del locator:  +90.2°
            dirección del vector Velocity:             +90.1°   <- 0.1°
            módulo real 0.199 m/s · Velocity 0.200              <- 0 % de error

        ⚠️ Durante un día este proyecto creyó que el stream era «basura» porque
           reportaba 0.001 m/s con el robot a 0.147 real. La observación era
           cierta; la conclusión, falsa: se leía solo la componente X con el
           robot encarado a ~90° del eje X del locator, donde X vale ~0 aunque
           el robot cruce la habitación. El sensor nunca fue el problema.

        DOS PASOS, y hacen falta los dos:

          1. La misma rotación de -90° que `_h_locator`, para llevar el vector
             al marco `odom` (X = «adelante» del robot al arrancar el driver).
          2. Proyectar sobre el rumbo actual, para pasarlo al marco del robot.
        """
        v = datos['Velocity']
        vx_crudo, vy_crudo = float(v['X']), float(v['Y'])
        with self._lock:
            # 1) marco del locator -> marco `odom`:  R(-90°)·(x, y) = (y, -x)
            vx_odom, vy_odom = vy_crudo, -vx_crudo
            # 2) marco `odom` -> marco del robot: proyección sobre el rumbo.
            c, s = math.cos(self._yaw_actual), math.sin(self._yaw_actual)
            self._odom.twist.twist.linear.x = vx_odom * c + vy_odom * s
            self._odom.twist.twist.linear.y = -vx_odom * s + vy_odom * c
        self._quiza_publicar('velocity')

    async def _h_accel(self, datos) -> None:
        a = datos['Accelerometer']
        with self._lock:
            # 🔴 EL RVR DA LA ACELERACIÓN EN **g**, Y `sensor_msgs/Imu` LA QUIERE
            # EN m/s². Medido en reposo el 2026-07-31: el módulo del vector era
            # **0.973**, no 9.81. El driver de ROS 1 tampoco convertía, así que
            # `/imu` llevaba desde siempre valores 9.8 veces pequeños.
            #
            # Y **no** lleva la conversión FRD->FLU: el acelerómetro ya viene con
            # z ARRIBA. En reposo el valor crudo de z era **+0.967 g**; forzarle
            # el (x,-y,-z) del cuaternión lo dejaba en -0.967, es decir, con la
            # gravedad apuntando al techo.
            #
            # ⚠️ La lección: el RVR NO usa una sola convención para todo. El
            #    cuaternión y el locator vienen en FRD; el giroscopio y el
            #    acelerómetro, en FLU. Hay que comprobar CADA sensor por
            #    separado — aplicar la corrección "por analogía" rompió estos
            #    dos el 2026-07-31.
            self._imu.linear_acceleration = Vector3(
                x=float(a['X']) * G_MS2,
                y=float(a['Y']) * G_MS2,
                z=float(a['Z']) * G_MS2,
            )
        self._quiza_publicar('accelerometer')

    async def _h_color(self, datos) -> None:
        # El color no pasa por `_quiza_publicar` (no forma parte de /odom), así
        # que marca el latido por su cuenta. Si no lo hiciera, un robot que solo
        # enviara color parecería mudo.
        with self._lock:
            self._t_ultima_muestra = self._ahora_s()
            # AÑADIDO 2026-08-04 · espejo para `/estado_robot`. Ver `__init__`.
            self._t_muestra_real = self._t_ultima_muestra
            self._n_muestras += 1
        c = datos['ColorDetection']
        msg = Color()
        msg.rgb_color = [int(c['R']), int(c['G']), int(c['B'])]
        msg.confidence = float(c['Confidence'])
        self.pub_color.publish(msg)

    async def _h_luz(self, datos) -> None:
        """Luz ambiente. Clave del stream: `AmbientLight` -> `Light`.

        ✅ FUNCIONA SIEMPRE, y NO depende de `color_detection`. Este comentario
           llegó a decir lo contrario y era **falso** — ver el manual, cap. 18.4.

        🔴 ES OTRO SENSOR, EN OTRO SITIO, que el de color. Medido el 2026-08-01, y
           es la medida que lo separa:

             · encender los 10 grupos de LED del chasis lo sube de **1.76 a
               23.55** (13.3x). O sea que VE LOS PROPIOS LEDS DEL ROBOT.
             · el sensor RGBC, en cambio, da valores **idénticos** con los LEDs en
               rojo, verde o azul: está apantallado y solo ve lo que tiene debajo.

           ✅ Y EL PORQUÉ ES FÍSICO, no del software: el sensor mira **hacia
              arriba**, y encima del Sphero está el **piso que sostiene el LIDAR**
              —4.6 cm, ver `03_operacion/MEDIDAS_ROBOT.md`— que es **BLANCO**. Ese
              piso le devuelve la luz de los propios LEDs del robot.

           🔴 DECISIÓN (usuario, 2026-08-01): **ESTE TOPIC NO SE USA.** En este
              montaje un valor alto significa «el robot tiene LEDs encendidos», no
              «hay luz». Se probó solo para saber si el sensor responde, y
              responde.

              Se sigue publicando porque es gratis —va en el mismo stream que el
              resto— pero **ningún consumidor debe apoyarse en él** mientras el
              piso del LIDAR siga ahí, que será siempre. No se arregla con
              software: haría falta pintar de negro la cara inferior del piso o
              mover el sensor, y nada del laboratorio necesita luz ambiente.

        ⚠️ NO va por `_quiza_publicar`: no forma parte de `/odom`. Pero SÍ marca
           el latido, igual que el color — si no, un robot que solo enviara luz
           parecería mudo al detector de silencio.
        """
        with self._lock:
            self._t_ultima_muestra = self._ahora_s()
            # AÑADIDO 2026-08-04 · espejo para `/estado_robot`. Ver `__init__`.
            self._t_muestra_real = self._t_ultima_muestra
            self._n_muestras += 1
        m = Illuminance()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self._body_frame
        m.illuminance = float(datos['AmbientLight']['Light'])
        m.variance = 0.0            # 0.0 = desconocida (REP de sensor_msgs)
        self.pub_luz.publish(m)

    def _sondear_ir(self) -> None:
        """Consulta los sensores IR y el sistema de control, y publica `/estado_ir`.

        🔴 NO PUEDE LANZAR NUNCA — misma regla que `_publicar_estado`: el
           2026-07-31 una excepción dentro de un manejador dejó `/odom` e `/imu` a
           cero sin una línea en el log. Un topic informativo no puede llevarse
           por delante la telemetría.

        📝 DOS consultas, no una, y la segunda es la que da valor al mensaje:
           `get_active_control_system_id()` devuelve 8 mientras el firmware
           conduce el robot por IR. Sin eso, `following` mueve el robot y NADA en
           ROS se entera — no pasa por `cmd_vel`, así que ni el watchdog ni el
           `collision_monitor` lo ven.
        """
        # 🔴 CON EL ENLACE CAÍDO, NO SE PREGUNTA (2026-08-15, evidencia 126).
        #    Cada `_pedir` de abajo bloquea hasta 3 s; con el puerto muerto eso
        #    eran hasta 6 s por tic A 1 Hz — una tormenta que saturaba su grupo
        #    de callbacks y llenaba el journal a 2 líneas/3 s (la familia del
        #    `Failed to get scan`). La recuperación es trabajo del vigilante y
        #    del puerto que se reabre solo; este sondeo espera a que vuelvan.
        with self._lock:
            enlace_caido = self._recuperando or self._reanudaciones_fallidas > 0
        if enlace_caido:
            if not getattr(self, '_ir_sondeo_pausado', False):
                self._ir_sondeo_pausado = True
                self.get_logger().warn(
                    'sondeo IR EN PAUSA: el enlace con el RVR está caído; '
                    '/estado_ir dirá lecturas_validas=False hasta que vuelva.')
            with self._lock:
                self._ir_lecturas_validas = False
            self._publicar_estado_ir()
            return
        if getattr(self, '_ir_sondeo_pausado', False):
            self._ir_sondeo_pausado = False
            self.get_logger().info('sondeo IR reanudado: el enlace volvió.')
        try:
            ok_l, lect, _ = self._pedir(
                self._rvr.get_bot_to_bot_infrared_readings(),
                'sondeo IR: lecturas', timeout=3.0)
            crudo = None
            if ok_l and isinstance(lect, dict):
                crudo = lect.get('sensor_data')

            ok_c, ctrl, _ = self._pedir(
                self._rvr.get_active_control_system_id(),
                'sondeo IR: sistema de control', timeout=3.0)
            conduciendo = False
            if ok_c and isinstance(ctrl, dict):
                # 8 = ControlSystemIdsEnum.infrared_follow_and_evade
                conduciendo = any(v == 8 for v in ctrl.values())

            with self._lock:
                if crudo is not None:
                    self._ir_crudo = int(crudo)
                    self._ir_t_lectura = self._ahora_s()
                    self._ir_lecturas_validas = True
                else:
                    # 🔴 NO se publican ceros como si fueran lecturas. Si la
                    #    consulta falló, se dice que falló.
                    self._ir_lecturas_validas = False
                self._ir_conduciendo = conduciendo
        except Exception:                                   # noqa: BLE001
            self.get_logger().error(
                f'sondeo IR: excepción no esperada:\n{traceback.format_exc()}')
            with self._lock:
                self._ir_lecturas_validas = False
        self._publicar_estado_ir()

    def _publicar_estado_ir(self) -> None:
        """Publica `/estado_ir`. Separado del sondeo para que también se publique
        con el sondeo apagado (`ir_sondeo_hz:=0`), diciendo que no hay lecturas.
        """
        try:
            ahora = self._ahora_s()
            with self._lock:
                crudo = self._ir_crudo
                validas = self._ir_lecturas_validas
                t_lect = self._ir_t_lectura
                cod = self._ir_ultimo_codigo
                t_msg = self._ir_t_ultimo_mensaje
                modo, far, near = self._ir_modo, self._ir_far, self._ir_near
                conduciendo = self._ir_conduciendo

            m = EstadoIR()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = self._base_frame
            m.crudo = int(crudo) & 0xFFFFFFFF
            # Del byte MENOS significativo al más. Ese orden es una DECISIÓN, no
            # un hecho: por eso va también `crudo`. Ver EstadoIR.msg.
            m.sensor_0 = (m.crudo >> 0) & 0xFF
            m.sensor_1 = (m.crudo >> 8) & 0xFF
            m.sensor_2 = (m.crudo >> 16) & 0xFF
            m.sensor_3 = (m.crudo >> 24) & 0xFF
            m.lecturas_validas = bool(validas)
            # -1.0 = nunca se ha consultado. Distinto de «hace mucho».
            m.antiguedad_lectura_s = (-1.0 if t_lect is None
                                      else float(ahora - t_lect))
            m.ultimo_codigo = int(cod)
            m.hay_mensaje = t_msg is not None
            m.antiguedad_mensaje_s = (-1.0 if t_msg is None
                                      else float(ahora - t_msg))
            m.modo = modo
            m.far_code = int(far)
            m.near_code = int(near)
            m.conduciendo_por_ir = bool(conduciendo)
            self.pub_estado_ir.publish(m)
        except Exception:                                   # noqa: BLE001
            self.get_logger().error(
                f'/estado_ir: no se pudo publicar:\n{traceback.format_exc()}')

    async def _h_ir_mensaje(self, datos) -> None:
        """Mensaje IR recibido de OTRO robot -> `/infrared_messages`.

        ✅ LA CLAVE ES `infrared_code`, Y ESTÁ MEDIDA. Volcado del primer mensaje
           real, en rvr-01 el 2026-08-11 con rvr-02 emitiendo el código 3:

               PRIMER mensaje IR recibido. Payload CRUDO: {'infrared_code': 3}

           UNA sola clave, plana, en snake_case. La notificación (CID 0x2C)
           declara un único campo de salida.

        🔴 Y ESO DESMONTÓ DOS COSAS:
           1. Las cuatro intensidades del mensaje eran FICCIÓN: son parámetros
              del ENVÍO. `InfraredMessage.msg` se rehizo por eso.
           2. El nodo de ROS 1 leía `datos['InfraredMessage']['Code']` — contra
              este payload, `KeyError` en la primera línea. **El IR de ROS 1
              nunca recibió un solo mensaje**, y nadie lo notó porque hacía falta
              un segundo robot para verlo.

        ✅ Y el firmware SÍ entrega esta notificación, al revés que las de motor
           (que NO llegan, medido en `_sondear_motores`). Se conserva el volcado
           del primero y el contador: si algún día cambia el firmware, se verá
           aquí y con el dato delante.
        """
        self._n_ir_recibidos += 1
        if self._n_ir_recibidos == 1:
            self.get_logger().info(
                f'PRIMER mensaje IR recibido. Payload CRUDO: {datos!r}')

        if not isinstance(datos, dict) or 'infrared_code' not in datos:
            # 🔴 Se avisa en vez de publicar un 0. Un código 0 es VÁLIDO, así que
            #    publicarlo ante un payload que no entiendo sería inventarme un
            #    mensaje. Y el volcado dice exactamente qué llegó.
            self.get_logger().error(
                f'mensaje IR con un payload que no reconozco: {datos!r}. '
                "Esperaba la clave 'infrared_code'. NO se publica nada.")
            return

        try:
            codigo = int(datos['infrared_code'])
        except (TypeError, ValueError):
            self.get_logger().error(
                f"mensaje IR con 'infrared_code' no numérico: {datos!r}")
            return

        m = InfraredMessage()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self._base_frame
        m.code = codigo & 0xFF
        self.pub_ir.publish(m)

        # Y el estado, para que `/estado_ir` sepa cuánto hace del último.
        with self._lock:
            self._ir_ultimo_codigo = m.code
            self._ir_t_ultimo_mensaje = self._ahora_s()

    async def _h_encoders(self, datos) -> None:
        """Cuentas de encoder. Clave del stream: `Encoders` -> `Left`, `Right`.

        📝 7792 ticks/m, calibrados contra cinta métrica (CLAUDE.md). Es la única
           fuente del robot que NO depende del marco de referencia, así que sirve
           para contrastar `/odom` cuando se sospecha de los marcos.
        """
        with self._lock:
            self._t_ultima_muestra = self._ahora_s()
            # AÑADIDO 2026-08-04 · espejo para `/estado_robot`. Ver `__init__`.
            self._t_muestra_real = self._t_ultima_muestra
            self._n_muestras += 1
        e = datos['Encoders']
        m = Encoder()
        # 🔴 LAS CLAVES SON `LeftTicks`/`RightTicks`, NO `Left`/`Right`. La tabla
        #    de documentación del propio SDK
        #    (`sensor_streaming_control.py`, «| Encoders | Left, Right |») dice
        #    otra cosa que el payload real. Con las claves malas el handler
        #    lanzaba `KeyError`, el topic quedaba registrado y **cero mensajes**:
        #    el síntoma exacto de un RVR dormido. Medido el 2026-08-01 mirando el
        #    dict crudo, que es lo único que lo desempata.
        #
        # 🔴 Y VIENEN SIN SIGNO, en 32 bits: un retroceso llega como 4294965940,
        #    que son -1356. `Encoder.msg` es int32, así que sin convertir se
        #    publicaría un número absurdo — y creciente, que es peor: parecería
        #    un encoder que funciona.
        m.left_wheel_count = self._u32_a_signed(e['LeftTicks'])
        m.right_wheel_count = self._u32_a_signed(e['RightTicks'])
        self.pub_encoders.publish(m)

    @staticmethod
    def _u32_a_signed(v) -> int:
        """Entero de 32 bits sin signo -> con signo. 4294965940 -> -1356."""
        v = int(v)
        return v - (1 << 32) if v >= (1 << 31) else v

    # ── Salud de los motores (notificaciones por evento) ─────────────────────
    #
    # 🔴 ESTOS TRES CORREN EN EL HILO DE ASYNCIO, igual que los handlers de
    #    telemetría, y una excepción aquí NO da error visible: el 2026-07-31 una
    #    en `_h_quaternion` dejó `/odom` e `/imu` mudos sin una sola línea en el
    #    log. Por eso cada uno va envuelto: si el firmware manda un campo con
    #    otro nombre, se avisa UNA vez y el driver sigue funcionando.

    async def _h_motor_atasco(self, datos) -> None:
        try:
            # `motor_index`: 0 = izquierdo, 1 = derecho (`MotorIndexesEnum`).
            idx = int(datos.get('motor_index', -1))
            act = bool(datos.get('is_triggered', False))
            if idx not in (0, 1):
                self._avisar_una_vez('motor_atasco_idx',
                                     f'motor_index inesperado: {datos!r}')
                return
            with self._lock:
                cambio = self._motor_atasco[idx] != act
                self._motor_atasco[idx] = act
                self._t_ultimo_atasco = self._ahora_s()
            lado = 'izquierdo' if idx == 0 else 'derecho'
            if cambio:
                # ERROR, no warn: en un laboratorio remoto esto decide si alguien
                # tiene que ir al edificio.
                if act:
                    self.get_logger().error(
                        f'🔴 MOTOR {lado.upper()} ATASCADO. No es que el robot esté '
                        'parado: el firmware ve corriente y no ve giro. '
                        'Comprueba la oruga.')
                else:
                    self.get_logger().warn(f'motor {lado}: atasco resuelto')
            self._publicar_motores()
        except Exception:                                   # noqa: BLE001
            self._avisar_una_vez('motor_atasco_exc',
                                 f'fallo en _h_motor_atasco:\n{traceback.format_exc()}')

    async def _h_motor_fallo(self, datos) -> None:
        try:
            act = bool(datos.get('is_fault', False))
            with self._lock:
                cambio = self._motor_fallo != act
                self._motor_fallo = act
                self._t_ultimo_fallo = self._ahora_s()
            if cambio:
                if act:
                    self.get_logger().error(
                        '🔴 FALLO ELÉCTRICO DE MOTOR. Más grave que un atasco: '
                        'NO se resuelve levantando el robot.')
                else:
                    self.get_logger().warn('fallo de motor: resuelto')
            self._publicar_motores()
        except Exception:                                   # noqa: BLE001
            self._avisar_una_vez('motor_fallo_exc',
                                 f'fallo en _h_motor_fallo:\n{traceback.format_exc()}')

    async def _h_motor_termico(self, datos) -> None:
        try:
            with self._lock:
                self._motor_temp = [
                    float(datos.get('left_motor_temperature', 0.0)),
                    float(datos.get('right_motor_temperature', 0.0)),
                ]
                prev = list(self._motor_estado_term)
                self._motor_estado_term = [
                    int(datos.get('left_motor_status', 0)),
                    int(datos.get('right_motor_status', 0)),
                ]
                nuevo = list(self._motor_estado_term)
                temps = list(self._motor_temp)
                self._t_ultimo_termico = self._ahora_s()
            # 📝 Solo se avisa cuando el estado CAMBIA y no es 0. Los valores
            #    distintos de 0 los define el firmware y este proyecto no los ha
            #    caracterizado, así que se dicen en crudo en vez de traducirlos.
            if nuevo != prev and any(nuevo):
                self.get_logger().warn(
                    f'⚠️ protección térmica de motores activa: estado '
                    f'izq={nuevo[0]} der={nuevo[1]} · '
                    f'{temps[0]:.1f} °C / {temps[1]:.1f} °C. '
                    'El firmware limita la potencia: el robot navegará mal.')
            self._publicar_motores()
        except Exception:                                   # noqa: BLE001
            self._avisar_una_vez('motor_term_exc',
                                 f'fallo en _h_motor_termico:\n{traceback.format_exc()}')

    def _avisar_transicion(self, clave: str, ok: bool, mensaje: str) -> None:
        """Avisa en cada CAMBIO sano<->fallo, no una sola vez en la vida.

        🔴 POR QUE NO VALE `_avisar_una_vez` PARA UN SONDEO PERIODICO: su conjunto
           de claves **no se vacia nunca**. Un fallo transitorio al arrancar quema
           la clave, y si despues empieza a fallar de verdad —cada 30 s, para
           siempre— **no se dice ni una palabra**. El dato se queda congelado y
           parece sano.

           Encontrado el 2026-08-01: se habia arreglado en la bateria y **se dejo
           en pie en el sondeo de motores**, que tiene el mismo patron.
        """
        with self._lock:
            antes = self._estado_sondeo.get(clave, True)
            self._estado_sondeo[clave] = ok
        if ok and not antes:
            self.get_logger().info(f'{clave}: recuperado')
        elif not ok and antes:
            self.get_logger().warn(mensaje)

    def _avisar_una_vez(self, clave: str, mensaje: str) -> None:
        """Un aviso repetido a 16 Hz llena el journal y esconde lo demás."""
        with self._lock:
            visto = clave in self._avisos_dados
            self._avisos_dados.add(clave)
        if not visto:
            self.get_logger().warn(mensaje)

    def _publicar_motores(self) -> None:
        """Publica el estado actual. Se llama desde los handlers y del keepalive.

        🔴 `antiguedad_*_s = -1.0` significa **nunca ha llegado una notificación**,
           no «hace mucho». Es la diferencia entre «todo va bien» y «no sé nada»,
           y publicarlas iguales sería exactamente el tipo de fallo silencioso que
           este driver lleva media docena de veces pagando.
        """
        ahora = self._ahora_s()
        with self._lock:
            m = MotorStatus()
            m.atascado_izquierdo = self._motor_atasco[0]
            m.atascado_derecho = self._motor_atasco[1]
            m.fallo = self._motor_fallo
            m.temperatura_izquierdo = float(self._motor_temp[0])
            m.temperatura_derecho = float(self._motor_temp[1])
            m.estado_termico_izquierdo = int(self._motor_estado_term[0])
            m.estado_termico_derecho = int(self._motor_estado_term[1])
            m.antiguedad_atasco_s = (
                -1.0 if self._t_ultimo_atasco is None
                else float(ahora - self._t_ultimo_atasco))
            m.antiguedad_fallo_s = (
                -1.0 if self._t_ultimo_fallo is None
                else float(ahora - self._t_ultimo_fallo))
            m.antiguedad_termico_s = (
                -1.0 if self._t_ultimo_termico is None
                else float(ahora - self._t_ultimo_termico))
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self._body_frame
        self.pub_motores.publish(m)

    def _vigilar_luz_color(self) -> None:
        """Apaga la luz del sensor si nadie la está usando. 1 Hz.

        Solo toca la luz que encendió **el servicio**: la del parámetro de
        arranque la puso alguien a propósito y no se le apaga por debajo.

        Dos condiciones, y manda **la que ocurra antes**:

          inactividad  nadie suscrito a `/color` **y** ninguna llamada a
                       `get_rgbc_sensor_values` en `color_apagado_inactividad_s`
          tope duro    `color_apagado_max_s` desde que se encendió, pase lo que
                       pase — porque rosbridge comparte UNA suscripción entre
                       todos sus clientes y una pestaña olvidada mantendría el
                       contador a 1 para siempre

        🔴 Se apaga con `_enviar`, no con `_pedir`: esto corre en un temporizador
           y bloquear aquí esperando al RVR congelaría el grupo `g_salud`, que es
           el que lleva el latido y el detector de silencio. `_enviar` no espera
           pero **sí registra el fallo**, que es lo que lo distingue de tirar el
           resultado a la basura.

        📝 Y el efecto es comprobable desde fuera sin creerse nada: `/color`
           vuelve a `[0,0,0]` y `color_activo` de `/estado_robot` pasa a `false`.
        """
        if not self._color_por_servicio or self._rvr is None:
            return

        ahora = _time.monotonic()
        # Alguien mirando el topic cuenta como actividad, igual que una consulta.
        if self.pub_color is not None and self.pub_color.get_subscription_count() > 0:
            self._color_t_actividad = ahora

        motivo = None
        if self._color_inactividad > 0 and \
                ahora - self._color_t_actividad >= self._color_inactividad:
            motivo = (f'nadie la usa desde hace '
                      f'{ahora - self._color_t_actividad:.0f} s')
        elif self._color_max > 0 and ahora - self._color_t_enable >= self._color_max:
            motivo = (f'lleva {ahora - self._color_t_enable:.0f} s encendida '
                      f'(tope {self._color_max:g} s)')
        if motivo is None:
            return

        # Se marca ANTES de encolar. Si se marcara después, el siguiente tic
        # (1 s) volvería a entrar y encolaría un segundo apagado sobre el mismo
        # puerto serie: el temporizador dispararía en bucle hasta que el primero
        # llegara. Marcar primero cierra la ventana.
        self._color_por_servicio = False
        self._color_detection = False
        self._enviar(self._rvr.enable_color_detection(is_enabled=False),
                     'apagado automático de la luz del color')
        self.get_logger().info(
            f'luz del sensor de color APAGADA sola: {motivo}. '
            f'Vuelve a encenderla con enable_color(true).')

    def _publicar_estado(self) -> None:
        """Publica `/estado_robot`. Timer de rclpy a 1 Hz, grupo `g_salud`.

        ⚠️ **NO VERIFICADO**: escrito el 2026-08-04 sin robot delante. Compila y
           nada más. El procedimiento para comprobarlo está en
           `NOTAS_ESTADO_ROBOT.md`, en la raíz del repositorio.

        🔴 ESTE MÉTODO NO PUEDE LANZAR NUNCA, y el `try` de abajo no es celo: el
           2026-07-31 una `AttributeError` dentro de un manejador de telemetría
           dejó `/odom` e `/imu` a CERO **sin una sola línea en el log**, con los
           topics existiendo y `/scan` funcionando. Que una señal de vida se
           lleve por delante la telemetría sería el peor intercambio posible, así
           que todo va envuelto y cualquier fallo sale por `error()`.

        📝 El latido avanza **al final** y solo si la publicación salió bien: si
           avanzara antes, un fallo permanente al publicar daría un contador que
           sube sin que nadie reciba nada. Aquí eso da igual —quien no recibe no
           ve el número— pero la regla del proyecto es contar EFECTOS, no
           intentos, y no se hace la excepción.
        """
        try:
            ahora = self._ahora_s()
            with self._lock:
                t_real = self._t_muestra_real
                t_odom = self._t_odom_completo
                fallidas = self._reanudaciones_fallidas
                activo = self._streaming_activo
                # 🆕 2026-08-11 · Se lee AQUI, dentro del cerrojo que ya esta
                #    tomado, y no mas abajo:  es un Lock normal, NO
                #    reentrante, asi que una segunda adquisicion en el mismo
                #    metodo seria un bloqueo esperando a que alguien lo cambie.
                ir_cond = self._ir_conduciendo

            m = EstadoRobot()
            m.header.stamp = self.get_clock().now().to_msg()
            # `body_frame` por coherencia con `/motor_status`, que es el mensaje
            # hermano. 📝 De este mensaje **nada es espacial**: el header está por
            # la MARCA DE TIEMPO, que es lo que permite a la web saber si lo que
            # tiene en pantalla es de hace un segundo o de hace un minuto.
            m.header.frame_id = self._body_frame

            # -1.0 = «no se sabe», nunca «cero». Es la convención de este
            # proyecto y la misma que usan las `antiguedad_*_s` de MotorStatus.
            m.antiguedad_muestra_s = (
                -1.0 if t_real is None else float(ahora - t_real))
            m.antiguedad_odom_s = (
                -1.0 if t_odom is None else float(ahora - t_odom))

            # Umbral: el mismo `silence_timeout` que usa el vigilante, para que
            # las dos señales no puedan contradecirse.
            #
            # ⚠️ Con el vigilante DESACTIVADO (`silence_timeout:=0`, que es como
            #    se reproduce el fallo del RVR dormido a propósito) se usa el
            #    valor por defecto en vez de cero: un umbral de 0 s daría
            #    `rvr_responde=false` permanentemente sobre un robot perfecto.
            umbral = (self._timeout_silencio if self._timeout_silencio > 0
                      else TIMEOUT_SILENCIO_S)
            # `activo` importa: antes de que el streaming arranque no ha llegado
            # nada todavía, y decir que el RVR responde sería inventárselo.
            m.rvr_responde = bool(
                activo and t_real is not None and (ahora - t_real) < umbral)

            # Se lee sin el lock, igual que hacen `_cb_cmd_vel` y
            # `_srv_control_state`: es un bool y en CPython leerlo es atómico.
            # Coger el lock aquí no añadiría nada y sí acoplaría este callback al
            # hilo de asyncio.
            m.parada_emergencia = bool(self._parada_emergencia)

            # 🔴 LA LUZ DEL SENSOR DE COLOR, y este campo existe por una razón
            #    concreta: **se apaga sola** (`_vigilar_luz_color`). Un cliente
            #    que recuerde «yo la encendí» pintaría el botón encendido sobre
            #    un sensor a oscuras. Con el apagado automático, el estado hay que
            #    LEERLO, no recordarlo — y por eso no vale un flag en el navegador.
            m.color_activo = bool(self._color_detection)
            # 🆕 El mismo dato que /estado_ir, en el canal barato. Ver el msg.
            m.conduciendo_por_ir = bool(ir_cond)

            m.reanudaciones_fallidas = int(fallidas)
            m.latido = int(self._latido)
            self.pub_estado.publish(m)
            self._latido += 1
        except Exception:                                       # noqa: BLE001
            # Nunca se propaga: un latido roto no puede tumbar el nodo, y
            # callarlo sería el fallo silencioso de siempre.
            self.get_logger().error(
                f'fallo publicando /estado_robot:\n{traceback.format_exc()}')

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
            # AÑADIDO 2026-08-04 · el espejo que SOLO tocan los handlers. No
            # cuesta un reloj más: reusa el valor de la línea de arriba. Ver el
            # bloque `_t_muestra_real` de `__init__`.
            self._t_muestra_real = self._t_ultima_muestra
            self._n_muestras += 1
            self._recibidos.add(componente)
            if not COMPONENTES_ODOM <= self._recibidos:
                return
            self._recibidos.clear()
            # AÑADIDO 2026-08-04 · AQUÍ, después del return, y esa es toda la
            # gracia: solo avanza cuando un /odom se completa de verdad. Ver el
            # bloque `_t_odom_completo` de `__init__`.
            self._t_odom_completo = self._t_ultima_muestra
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
            # 🔴 `drive_stop()` NO BASTA con los modos IR activos: `evading` y
            #    `following` son modos del FIRMWARE, así que el RVR volvería a
            #    conducir en la siguiente detección — la parada frenaría un
            #    instante y el robot arrancaría solo. Es el mismo patrón que ya
            #    mordió al liberar la parada con Nav2 (manual, cap. 15.4).
            #    Se mandan siempre, cuesten o no: la parada no pregunta.
            self._enviar(self._rvr.stop_robot_to_robot_infrared_evading(),
                         'parada: evasión IR')
            self._enviar(self._rvr.stop_robot_to_robot_infrared_following(),
                         'parada: seguimiento IR')
            # Y el estado, o `/estado_ir` seguiría diciendo `modo: following`
            # sobre un robot que la parada acaba de detener. Se anota aquí y no
            # al confirmar cada `stop_*` porque la parada NO PREGUNTA: manda los
            # comandos cuesten o no, y el estado tiene que reflejar la intención
            # de la parada, no el acuse del RVR.
            with self._lock:
                self._ir_modo = 'off'
                self._ir_conduciendo = False

    # ─────────────────────────────────────────────────────────────────────────
    # Servicios: LEDs
    # ─────────────────────────────────────────────────────────────────────────
    # ⚠️ NINGUNO DE ESTOS MUEVE EL ROBOT. Se portaron primero justamente por eso:
    #    se pueden probar en banco sin espacio y sin riesgo.
    #
    # 📝 El LED del sensor de color NO se controla desde aquí: no es un grupo de
    #    `RvrLedGroups`, sino `enable_color_detection`. Si no se desactiva, se
    #    queda encendido gastando batería (CLAUDE.md).

    @staticmethod
    def _brillos_para(grupo: int, r: int, g: int, b: int) -> list[int]:
        """Cuántos valores de brillo espera este grupo. NO son siempre tres.

        🔴 ESTE ERA UN BUG REAL, y del peor tipo: `success=True` con el LED
           apagado. Encontrado el 2026-08-01 mirando el robot mientras el
           servicio decía que sí — el usuario vio encenderse diez de los doce
           grupos, y los dos que faltaban son justo los que NO tienen 3 canales.

        `led_group` es una MÁSCARA DE BITS, un bit por canal, y `set_all_leds`
        espera **un valor de brillo por bit encendido**:

            los 10 grupos normales   3 bits  -> [r, g, b]
            all_lights              30 bits  -> [r, g, b] x 10
            undercarriage_white      1 bit   -> un solo valor (es blanco, no RGB)

        Mandar tres valores a un grupo de 1 o de 30 bits no da error: el RVR lo
        acepta y no hace nada. Por eso hay que contar los bits en vez de suponer.
        """
        bits = bin(grupo).count('1')
        if bits % 3 == 0:
            return [r, g, b] * (bits // 3)
        # Un LED de canal único (el blanco de los bajos): no tiene color, solo
        # brillo. Se usa el máximo de los tres para que «ponme rojo» encienda
        # algo en vez de quedarse a oscuras por tener el verde y el azul a cero.
        return [max(r, g, b)] * bits

    def _srv_led_rgb(self, req, resp):
        """Un LED, un color."""
        if not 0 <= req.led_id < len(self.LEDS):
            resp.success, resp.message = False, (
                f'led_id {req.led_id} fuera de rango (0..{len(self.LEDS)-1}); '
                f'ver la tabla LEDS')
            return resp
        if not self._rgb_valido(req.red, req.green, req.blue):
            resp.success, resp.message = False, 'los canales RGB van de 0 a 255'
            return resp
        grupo = getattr(RvrLedGroups, self.LEDS[req.led_id]).value
        ok, _, msg = self._pedir(
            self._rvr.set_all_leds(
                led_group=grupo,
                led_brightness_values=self._brillos_para(
                    grupo, req.red, req.green, req.blue)),
            f'set_led_rgb({self.LEDS[req.led_id]})')
        resp.success = ok
        resp.message = msg or f'{self.LEDS[req.led_id]} = ({req.red}, {req.green}, {req.blue})'
        return resp

    def _srv_leds_multiples(self, req, resp):
        """Varios LEDs de una vez. Las cuatro listas tienen que medir lo mismo."""
        n = len(req.led_ids)
        if not n or any(len(x) != n for x in (req.red_values, req.green_values, req.blue_values)):
            resp.success, resp.message = False, (
                f'las cuatro listas deben tener la misma longitud y no estar vacías '
                f'(ids {n}, r {len(req.red_values)}, g {len(req.green_values)}, '
                f'b {len(req.blue_values)})')
            return resp
        # Se valida TODO antes de mandar nada: dejar la mitad de los LEDs
        # cambiados y la otra mitad no es peor que no hacer nada.
        for i, r, g, b in zip(req.led_ids, req.red_values, req.green_values, req.blue_values):
            if not 0 <= i < len(self.LEDS):
                resp.success, resp.message = False, f'led_id {i} fuera de rango'
                return resp
            if not self._rgb_valido(r, g, b):
                resp.success, resp.message = False, f'RGB fuera de 0..255 en el led_id {i}'
                return resp
        hechos = []
        for i, r, g, b in zip(req.led_ids, req.red_values, req.green_values, req.blue_values):
            grupo_i = getattr(RvrLedGroups, self.LEDS[i]).value
            ok, _, msg = self._pedir(
                self._rvr.set_all_leds(
                    led_group=grupo_i,
                    led_brightness_values=self._brillos_para(grupo_i, r, g, b)),
                f'set_multiple_leds({self.LEDS[i]})')
            if not ok:
                resp.success, resp.message = False, f'falló en {self.LEDS[i]}: {msg}'
                return resp
            hechos.append(self.LEDS[i])
        resp.success, resp.message = True, f'{len(hechos)} LEDs: {", ".join(hechos)}'
        return resp

    def _srv_leds_todos(self, req, resp):
        """Todos los LEDs a un color. `SetLeds.srv` no tiene campos de respuesta."""
        c = list(req.rgb_color)
        if len(c) != 3 or not self._rgb_valido(*c):
            self.get_logger().warn(
                f'set_leds: rgb_color debe ser [r, g, b] con valores 0..255, llegó {c}')
            return resp
        # `c * 10` era correcto —all_lights tiene 30 bits— pero se usa el helper
        # para que exista UNA sola regla: si mañana cambia la máscara, cambia en
        # un sitio. Este servicio funcionaba; los otros dos no.
        self._pedir(
            self._rvr.set_all_leds(
                led_group=RvrLedGroups.all_lights.value,
                led_brightness_values=self._brillos_para(
                    RvrLedGroups.all_lights.value, *c)),
            'set_leds')
        return resp

    # ─────────────────────────────────────────────────────────────────────────
    # Servicios: lecturas
    # ─────────────────────────────────────────────────────────────────────────

    async def _encender_color(self, encender: bool) -> None:
        """El `enable` y su espera, juntos. La espera NO es decorativa.

        🔴 Sin el `sleep`, un cliente que lea justo después de que el servicio
           vuelva pilla la muestra ANTERIOR —oscuridad— y se lleva un
           `success=True` sobre valores falsos. Es exactamente el fallo de la
           primera versión de `get_rgbc_sensor_values` (2026-07-31), que hacía
           `enable(True) → leer → enable(False)` y devolvía negro con éxito.

        📝 Los 0,1 s salen de ROS 1 (`Atriz_rvr_node.py:341`), donde este ciclo
           funcionó durante toda la vida del laboratorio. Medido el 2026-08-06:
           con esa espera, la primera muestra del stream ya viene iluminada.
        """
        await self._rvr.enable_color_detection(is_enabled=encender)
        await asyncio.sleep(0.1)

    def _srv_enable_color(self, req, resp):
        """Enciende o apaga la luz del sensor de color, EN CALIENTE.

        Es el botón «sesión de medición» de la web: `data=true` enciende el LED
        y `/color` empieza a dar valores reales; `data=false` lo apaga.

        ✅ MEDIDO EL 2026-08-06, con el streaming ya arrancado a 250 ms
        (`mediciones_banco/probar_color_stream_caliente.py`):

            muestras no-cero de /color:  0/24 → 24/24 → 24/24 → 0/24
            canal claro por consulta:    1 → 1321 → 1322 → 1     (1321×)
            RGB reales sobre suelo claro: (255, 223, 209)

        Dos rutas independientes se mueven a la vez y vuelven a la línea base al
        apagar, y el usuario vio el LED blanco encenderse bajo el chasis.

        🔴 ESTO REFUTA lo que este fichero afirmó desde el 2026-07-31 —«con el
           streaming ya configurado, `enable_color_detection` NO HACE NADA, 481
           mensajes todos ceros»—. Aquella medida no probaba eso: el servicio
           bajo prueba **se apagaba a sí mismo** dentro de la misma llamada, y
           los 481 mensajes (~38 s a 12,7 Hz) son casi todos POSTERIORES al
           `enable(False)`. Una medida que no distingue las dos hipótesis no
           refuta ninguna. La corrección la trajo el usuario, que recordaba el
           ciclo funcionando en ROS 1 — y el código de ROS 1 lo respaldaba
           (servicio `enable_color`, `Atriz_rvr_node.py:331-344` y `:1636`).

        ⚠️ Deja un LED blanco encendido bajo el chasis y gasta batería. Quien lo
           encienda, que lo apague. `_apagar_rvr()` lo apaga siempre al cerrar,
           mire o no el parámetro, precisamente por esto.
        """
        encender = bool(req.data)
        ok, _, msg = self._pedir(self._encender_color(encender), 'enable_color')
        if ok:
            ahora = _time.monotonic()
            self._color_por_servicio = encender
            self._color_t_enable = ahora
            self._color_t_actividad = ahora
            # 🔴 Se actualiza para que `_srv_rgbc` no siga avisando de oscuridad
            #    cuando el sensor ya está encendido: un aviso que miente entrena
            #    al alumno a ignorar los avisos.
            self._color_detection = encender
            self.get_logger().info(
                f'sensor de color {"ENCENDIDO" if encender else "apagado"} '
                f'por servicio')
        resp.success = ok
        resp.message = msg or (
            'sensor de color encendido: /color ya da valores reales'
            if encender else 'sensor de color apagado')
        return resp

    def _srv_rgbc(self, _req, resp):
        """Sensor de color, en crudo.

        🔴 NO enciende ni apaga la luz, **y ahora es a propósito**: para eso
           está `enable_color` (`std_srvs/SetBool`). Encender y apagar dentro de
           esta misma llamada es lo que hacía la primera versión, y devolvía
           oscuridad con `success=True`.

        ⚠️ Si la luz está apagada —lo está al arrancar— y la superficie REFLEJA,
           esto devuelve valores de oscuridad (~1, 0, 1, 1 medidos). Para que dé
           algo, `enable_color(true)` en caliente o `color_detection:=true` al
           arrancar.

        🔴 PERO CON LA LUZ APAGADA **SÍ SE LEE EL SENSOR**, y eso es una FUNCIÓN,
           no un estado degradado. Esta llamada consulta al RVR pase lo que pase:
           lo único que cambia con la luz es `message`. Sobre una superficie que
           EMITE luz —una pantalla, una baldosa LED— la luz apagada es el modo
           CORRECTO, y el encendido da lo contrario de lo que hay. Medido el
           2026-08-08 sobre una pantalla de móvil (evidencia 86):

               pantalla ROJA, luz ON   ->  R/G = 0.66   <- menos rojo que verde
               pantalla ROJA, luz OFF  ->  R/G = 5.12   <- inconfundible

        🔴 Y ESTA ES LA ÚNICA VÍA PARA ESE MODO: el topic `/color` publica
           **ceros** con la luz apagada (medido: 39 mensajes, 0 no-cero), porque
           sale del STREAMING del RVR y el streaming se apaga con la detección.
           Este servicio CONSULTA, así que sigue dando datos. Contrato completo
           en `03_operacion/SENSOR_COLOR.md`.
        """
        # 🔴 Esto cuenta como ACTIVIDAD, y de ello depende que el apagado
        #    automático no le corte la práctica a un alumno: `atriz.py` lee el
        #    color por aquí, no por el topic `/color`.
        self._color_t_actividad = _time.monotonic()
        ok, datos, msg = self._pedir(self._rvr.get_rgbc_sensor_values(), 'get_rgbc')
        if not ok or not isinstance(datos, dict):
            resp.success, resp.message = False, msg or f'respuesta inesperada: {datos!r}'
            return resp
        resp.red_channel_value = int(datos.get('red_channel_value', 0))
        resp.green_channel_value = int(datos.get('green_channel_value', 0))
        resp.blue_channel_value = int(datos.get('blue_channel_value', 0))
        resp.clear_channel_value = int(datos.get('clear_channel_value', 0))
        resp.success = True
        # 🔴 El mensaje AFIRMABA «estos valores son oscuridad», y eso es FALSO
        #    cuando la superficie emite luz propia -- que es un modo legítimo y
        #    medido (evidencia 86). Un aviso que asegura de más sobre lo que no
        #    puede saber es peor que no avisar: aquí el driver no tiene forma de
        #    distinguir «negro» de «pantalla apagada» de «no hay nada debajo».
        resp.message = ('' if self._color_detection else
                        'la luz del sensor está APAGADA. Si la superficie '
                        'REFLEJA (suelo, cinta), esto es oscuridad: enciéndela '
                        'con enable_color(true). Si EMITE luz (pantalla, '
                        'baldosa LED), apagada es lo correcto.')
        return resp

    def _srv_encoders(self, _req, resp):
        """Cuentas de los encoders.

        📝 Son la única fuente del robot que NO depende del marco de referencia
        (CLAUDE.md), y están calibradas: 7792 ticks/m contra cinta métrica.
        """
        ok, datos, msg = self._pedir(self._rvr.get_encoder_counts(), 'get_encoders')
        if not ok or not isinstance(datos, dict):
            resp.success, resp.message = False, msg or f'respuesta inesperada: {datos!r}'
            return resp
        resp.left_wheel_count = int(datos.get('left_wheel_count', 0))
        resp.right_wheel_count = int(datos.get('right_wheel_count', 0))
        resp.success, resp.message = True, ''
        return resp

    def _srv_system_info(self, _req, resp):
        """Versiones e identificadores. Útil para la flota: saber qué hay en cada robot.

        ⚠️ `get_main_application_version` y `get_processor_name` EXIGEN `target`:
           1 es Nordic y 2 es ST. Sin el argumento dan `TypeError` (CLAUDE.md).
        """
        info = SystemInfo()
        fallos = []
        ok, v, msg = self._pedir(self._rvr.get_main_application_version(target=1),
                                 'system_info: versión')
        if ok and isinstance(v, dict):
            info.app_major = int(v.get('major', 0))
            info.app_minor = int(v.get('minor', 0))
            info.app_revision = int(v.get('revision', 0))
        else:
            fallos.append(f'versión ({msg})')
        ok, v, msg = self._pedir(self._rvr.get_bootloader_version(target=1),
                                 'system_info: bootloader')
        if ok and isinstance(v, dict):
            info.bootloader_major = int(v.get('major', 0))
            info.bootloader_minor = int(v.get('minor', 0))
            info.bootloader_revision = int(v.get('revision', 0))
        else:
            fallos.append(f'bootloader ({msg})')
        for campo, coro, clave, conv in (
            ('board_revision', self._rvr.get_board_revision(), 'revision', int),
            ('mac_address', self._rvr.get_mac_address(), 'mac_address', str),
            ('sku', self._rvr.get_sku(), 'sku', str),
        ):
            ok, d, msg = self._pedir(coro, f'system_info: {campo}')
            if ok and isinstance(d, dict) and clave in d:
                setattr(info, campo, conv(d[clave]))
            else:
                fallos.append(f'{campo} ({msg or d!r})')
        for campo, target in (('processor_1_name', 1), ('processor_2_name', 2)):
            ok, d, msg = self._pedir(self._rvr.get_processor_name(target=target),
                                     f'system_info: {campo}')
            if ok and isinstance(d, dict):
                setattr(info, campo, str(d.get('name', '')).strip('\x00'))
            else:
                fallos.append(f'{campo} ({msg})')
        # 📝 `uptime_ms` se queda a 0 SIEMPRE: el SDK vendorizado no tiene ningún
        #    método de uptime. El campo viene del .msg heredado de ROS 1, donde
        #    tampoco se rellenaba. Se dice en el mensaje en vez de dejar un cero
        #    mudo que parezca un dato.
        fallos.append('uptime_ms (el SDK no lo expone)')
        resp.system_info = info
        # Parcial cuenta como éxito, y se dice cuáles faltaron: es información de
        # diagnóstico, y devolver `False` por un campo tiraría los demás.
        resp.success = True
        resp.message = '' if not fallos else 'sin obtener: ' + '; '.join(fallos)
        return resp

    # ─────────────────────────────────────────────────────────────────────────
    # Servicios: estado, LEDs por evento, infrarrojos y configuración
    # ─────────────────────────────────────────────────────────────────────────
    # ⚠️ NINGUNO DE ESTOS MUEVE EL ROBOT tampoco. Los cuatro que sí lo mueven
    #    —MoveTimed, RawMotors, MoveToPose, MoveToPosAndYaw— van aparte, porque
    #    necesitan espacio despejado y aviso al usuario.

    #: Los eventos de `TriggerLedEvent.srv`, como patrones de color.
    #: 📝 En ROS 1 esto eran animaciones de la aplicación, no del SDK: el RVR no
    #:    tiene «eventos de LED». Aquí se traducen a colores fijos, que es lo que
    #:    de verdad hace falta en el laboratorio — que un estudiante vea de un
    #:    vistazo si su robot arrancó, se paró o falló.
    EVENTOS_LED = {
        1: ('STARTUP', (0, 0, 255)),          # azul
        2: ('EMERGENCY_STOP', (255, 0, 0)),   # rojo
        3: ('START_DRIVING', (0, 255, 0)),    # verde
        4: ('ERROR', (255, 0, 255)),          # magenta
        5: ('VISION_STARTUP', (0, 255, 255)), # cian
        10: ('DOWNLOAD_MODEL', (255, 180, 0)),# ámbar
    }

    def _srv_control_state(self, _req, resp):
        """Qué controlador manda y si hay fallo de motor."""
        estado = ControlState()
        fallos = []
        ok, d, msg = self._pedir(self._rvr.get_active_control_system_id(),
                                 'control_state: controlador')
        if ok and isinstance(d, dict):
            estado.active_controller_id = int(
                d.get('controller_id', d.get('active_control_system_id', 0)))
        else:
            fallos.append(f'controlador ({msg or d!r})')
        ok, d, msg = self._pedir(self._rvr.get_motor_fault_state(),
                                 'control_state: fallo de motor')
        if ok and isinstance(d, dict):
            estado.motor_fault = bool(d.get('is_fault', False))
        else:
            fallos.append(f'fallo de motor ({msg or d!r})')
        # Estos dos NO vienen del RVR: los sabe el driver, que es quien manda.
        # `is_stopped` incluye la parada de emergencia a propósito: un robot con
        # la parada puesta está parado aunque le lleguen `cmd_vel`.
        conduciendo = (not self._parada_emergencia
                       and (self._ahora_s() - self._t_ultimo_cmd_vel) < self._cmd_vel_timeout)
        estado.is_driving = conduciendo
        estado.is_stopped = not conduciendo
        resp.control_state = estado
        resp.success = True
        resp.message = '' if not fallos else 'sin obtener: ' + '; '.join(fallos)
        return resp

    def _srv_led_event(self, req, resp):
        """Patrón de color por evento. Ver `EVENTOS_LED`."""
        if req.stop_current_event:
            ok, _, _ = self._pedir(
                self._rvr.set_all_leds(led_group=RvrLedGroups.all_lights.value,
                                       led_brightness_values=[0, 0, 0] * 10),
                'trigger_led_event: apagar')
            resp.success = ok
            return resp
        ev = self.EVENTOS_LED.get(req.event_id)
        if ev is None:
            self.get_logger().warn(
                f'trigger_led_event: event_id {req.event_id} desconocido; '
                f'los válidos son {sorted(self.EVENTOS_LED)}')
            resp.success = False
            return resp
        nombre, color = ev
        ok, _, _ = self._pedir(
            self._rvr.set_all_leds(led_group=RvrLedGroups.all_lights.value,
                                   led_brightness_values=list(color) * 10),
            f'trigger_led_event({nombre})')
        resp.success = ok
        return resp

    def _srv_ir_mensaje(self, req, resp):
        """Emite un mensaje IR. No mueve el robot, pero SÍ emite."""
        if not 0 <= req.code <= 7:
            resp.success, resp.message = False, 'code va de 0 a 7'
            return resp
        for nombre, v in (('front', req.front_strength), ('left', req.left_strength),
                          ('right', req.right_strength), ('rear', req.rear_strength)):
            if not 0 <= v <= 64:
                resp.success, resp.message = False, f'{nombre}_strength va de 0 a 64'
                return resp
        ok, _, msg = self._pedir(
            self._rvr.send_infrared_message(
                infrared_code=req.code, front_strength=req.front_strength,
                left_strength=req.left_strength, right_strength=req.right_strength,
                rear_strength=req.rear_strength),
            'send_infrared_message')
        resp.success, resp.message = ok, msg or f'código {req.code} emitido'
        return resp

    def _srv_ir_modo(self, req, resp):
        """Modo IR robot-a-robot: broadcasting, following o off.

        📝 `evading` tiene su propio servicio (`set_ir_evading`), como en ROS 1.
        """
        modo = (req.mode or '').strip().lower()
        acciones = {
            'broadcasting': lambda: self._rvr.start_robot_to_robot_infrared_broadcasting(
                far_code=req.far_code, near_code=req.near_code),
            'following': lambda: self._rvr.start_robot_to_robot_infrared_following(
                far_code=req.far_code, near_code=req.near_code),
            # 🔴 'off' PARABA SOLO EL BROADCASTING (2026-08-01). Así que
            #    `following` —que hace conducir al robot— no se podía apagar, y
            #    `evading` no tenía forma alguna de apagarse desde ROS. Los tres
            #    métodos existen en el SDK y ahora se llaman los tres: apagar
            #    debe apagar, no apagar una de tres cosas.
            'off': lambda: self._rvr.stop_robot_to_robot_infrared_broadcasting(),
        }
        if modo not in acciones:
            resp.success, resp.message = False, (
                f"modo '{req.mode}' desconocido; usa {sorted(acciones)}")
            return resp

        # 🔴 VALIDACIÓN DE RANGO — RECUPERADA EL 2026-08-11. Es una REGRESIÓN que
        #    arrastrábamos desde la migración: el nodo de ROS 1 convertía los
        #    códigos con `InfraredCodes(req.far_code)`, que lanza con un valor
        #    inválido, y al portar a ROS 2 se pasaron crudos.
        #
        #    Y el SDK NO valida: de todo el árbol, el único sitio con el límite
        #    está en un helper (`infrared_control_async.py:99`) que este driver
        #    no usa, y solo comprueba `strength`. Un `far_code=200` viajaría al
        #    firmware sin que nadie diga nada.
        if modo != 'off' and not (self._codigo_ir_valido(req.far_code)
                                  and self._codigo_ir_valido(req.near_code)):
            resp.success, resp.message = False, (
                f'far_code={req.far_code} near_code={req.near_code}: los códigos '
                'IR van de 0 a 7')
            return resp

        # 🔴 AGUJERO DE SEGURIDAD, encontrado el 2026-08-11 con DOS robots
        #    delante. Es el MISMO que se tapó el 2026-08-01 en `set_ir_evading`,
        #    y se quedó abierto aquí:
        #
        #        $ ros2 topic pub --once /emergency_stop std_msgs/msg/Empty {}
        #        $ ros2 service call /set_ir_mode ... "{mode: 'following'}"
        #          -> success=True          <- y el RVR se pone a conducir
        #
        #    `following` hace conducir al robot igual que `evading`: es un modo
        #    del FIRMWARE, no pasa por `cmd_vel`, así que ni el watchdog ni el
        #    collision_monitor lo ven. Arrancarlo con la parada ACTIVA es
        #    exactamente lo que la parada existe para impedir.
        #
        #    Solo se protege `following`. `broadcasting` únicamente EMITE luz
        #    infrarroja —no mueve este robot— y `off` tiene que funcionar
        #    siempre, con parada o sin ella: apagar no se niega nunca.
        #
        # 📌 Y la lección: el arreglo del 2026-08-01 se dio por bueno mirando el
        #    servicio que se arregló, no buscando los demás que movían el robot.
        #    «Busca TODAS las menciones, no la primera» también vale para los
        #    controles de seguridad.
        if modo == 'following':
            permitido, motivo = self._mover_permitido()
            if not permitido:
                resp.success, resp.message = False, motivo
                return resp
            self.get_logger().warn(
                'set_ir_mode(following): el RVR conducirá SOLO al detectar IR. '
                'Ni el watchdog de cmd_vel ni el collision_monitor intervienen. '
                'Espacio despejado.')

        ok, _, msg = self._pedir(acciones[modo](), f'set_ir_mode({modo})')
        if modo == 'off' and ok:
            # Los otros dos, que son los que conducen. Sin esperar respuesta:
            # apagar no puede quedarse bloqueado.
            self._enviar(self._rvr.stop_robot_to_robot_infrared_following(),
                         'set_ir_mode(off): following')
            self._enviar(self._rvr.stop_robot_to_robot_infrared_evading(),
                         'set_ir_mode(off): evading')
        # 📝 El modo se anota SOLO si el comando salió bien, y es la misma regla
        #    que el latido de `/estado_robot`: se cuentan EFECTOS, no intentos.
        #    Un `/estado_ir` diciendo `modo: following` sobre un comando que
        #    falló sería peor que no decir nada.
        if ok:
            with self._lock:
                self._ir_modo = modo
                if modo == 'off':
                    self._ir_far = self._ir_near = 0
                    self._ir_conduciendo = False
                else:
                    self._ir_far, self._ir_near = int(req.far_code), int(req.near_code)
        resp.success, resp.message = ok, msg or f'modo IR: {modo}'
        return resp

    def _srv_ir_evasion(self, req, resp):
        """🔴 ESTE SÍ PUEDE MOVER EL ROBOT: la evasión IR conduce sola.

        No se le manda `cmd_vel`, así que el watchdog del driver NO lo para, y el
        `collision_monitor` tampoco lo ve — el RVR conduce por su cuenta. Se avisa
        en el log a nivel WARN por eso.
        """
        # 🔴 ESTO FALTABA, y era un agujero de seguridad real (2026-08-01).
        #    Este servicio hace CONDUCIR al robot, pero era el único de los que
        #    mueven que NO comprobaba la parada de emergencia: con la parada
        #    ACTIVA respondía `success=True` y el RVR se ponía a conducir solo.
        #    Y el manual afirmaba lo contrario — «se comprueba en todos».
        ok, msg = self._mover_permitido()
        if not ok:
            resp.success, resp.message = False, msg
            return resp
        # 🔴 VALIDACIÓN RECUPERADA EL 2026-08-11. El nodo de ROS 1 SÍ comprobaba
        #    los rangos aquí (`Atriz_rvr_node.py:522`) y al portar se perdió: era
        #    una regresión de seguridad, no de estilo. El SDK tampoco valida.
        if not (self._codigo_ir_valido(req.far_code)
                and self._codigo_ir_valido(req.near_code)):
            resp.success, resp.message = False, (
                f'far_code={req.far_code} near_code={req.near_code}: los códigos '
                'IR van de 0 a 7')
            return resp
        self.get_logger().warn(
            'set_ir_evading: el RVR conducirá SOLO al detectar IR. Ni el watchdog '
            'de cmd_vel ni el collision_monitor intervienen. Espacio despejado.')
        ok, _, msg = self._pedir(
            self._rvr.start_robot_to_robot_infrared_evading(
                far_code=req.far_code, near_code=req.near_code),
            'set_ir_evading')
        if ok:
            with self._lock:
                self._ir_modo = 'evading'
                self._ir_far, self._ir_near = int(req.far_code), int(req.near_code)
        resp.success, resp.message = ok, msg or 'evasión IR activada'
        return resp

    def _srv_drive_params(self, req, resp):
        """Parámetros del limitador de velocidad del RVR.

        ⚠️ Cambia CÓMO acelera el robot. No lo mueve por sí mismo, pero afecta a
           todo lo que se mueva después — incluido Nav2, cuyo `max_linear_accel`
           está ajustado a la rampa MEDIDA de ~0.5 s (manual, cap. 11).
        """
        ok, _, msg = self._pedir(
            self._rvr.set_drive_target_slew_parameters(
                a=req.a, b=req.b, c=req.c,
                linear_acceleration=req.linear_acceleration,
                linear_velocity_slew_method=req.linear_velocity_slew_method),
            'set_drive_parameters')
        if ok:
            self.get_logger().warn(
                f'set_drive_parameters: rampa cambiada (a={req.a} b={req.b} c={req.c} '
                f'accel={req.linear_acceleration}). Nav2 está ajustado a la rampa '
                'de fábrica: revísalo si vas a navegar.')
        resp.success = ok
        return resp

    def _srv_set_pos_yaw(self, req, resp):
        """Poner la odometría a cero. **Solo (0, 0, 0).**

        🔴 EL SDK NO PUEDE FIJAR UNA POSE ARBITRARIA. Solo tiene
           `reset_locator_x_and_y()`, que la pone a (0,0), y `reset_yaw()`, que
           **no hace nada** — el yaw del RVR se pone a cero al ENCENDER el robot
           (medido, manual cap. 10).

        Así que este servicio acepta (0,0,0) y **rechaza el resto en vez de
        fingir**. Para una pose arbitraria haría falta que el driver llevara sus
        propios desplazamientos de x, y y yaw sobre lo que publica en `/odom`; no
        está hecho, y tocar la ruta de la odometría —la parte más verificada del
        driver— pide su propia sesión de pruebas.
        """
        p = req.position
        if abs(p.x) > 1e-6 or abs(p.y) > 1e-6 or abs(req.yaw) > 1e-6:
            self.get_logger().warn(
                f'set_pos_and_yaw: solo se admite (0,0,0); llegó '
                f'({p.x:.3f}, {p.y:.3f}, yaw {req.yaw:.3f}). Ver el docstring.')
            resp.success = False
            return resp
        ok, _, _ = self._pedir(self._rvr.reset_locator_x_and_y(), 'set_pos_and_yaw')
        if ok:
            # El yaw publicado se pone a cero restando el actual, que es lo que
            # `reset_yaw()` NO hace.
            with self._lock:
                self._yaw_offset = None
            self.get_logger().info('odometría puesta a cero (posición y origen del yaw)')
        resp.success = ok
        return resp

    # ─────────────────────────────────────────────────────────────────────────
    # Servicios QUE MUEVEN EL ROBOT
    # ─────────────────────────────────────────────────────────────────────────
    # 🔴🔴 ESTOS CUATRO SE SALTAN EL `collision_monitor`, Y NO HAY FORMA DE
    #      EVITARLO DESDE AQUÍ.
    #
    #   La capa de seguridad filtra `/cmd_vel_raw -> /cmd_vel` (manual, cap. 12).
    #   Estos servicios NO publican en ningún topic: le hablan al RVR por el
    #   puerto serie. El monitor no puede verlos ni frenarlos.
    #
    #   Tampoco los para el watchdog de `cmd_vel`: ese vigila que sigan LLEGANDO
    #   mensajes, y aquí no hay mensajes que dejen de llegar.
    #
    #   Lo único que los detiene es la PARADA DE EMERGENCIA, que se comprueba
    #   abajo, y `drive_stop`.
    #
    # ⚠️ Con Nav2 corriendo, además, dos cosas mandarían al robot a la vez. No se
    #    prohíbe —un estudiante puede querer justo eso— pero se avisa.

    def _mover_permitido(self) -> tuple[bool, str]:
        """Comprobaciones comunes a todo servicio que mueva el robot."""
        if self._rvr is None:
            return False, 'el RVR no está conectado'
        if self._parada_emergencia:
            return False, ('parada de emergencia ACTIVA: llama primero a '
                           '/release_emergency_stop')
        return True, ''

    def _srv_move_timed(self, req, resp):
        """Velocidad constante durante N segundos, y para. BLOQUEANTE."""
        ok, msg = self._mover_permitido()
        if not ok:
            self.get_logger().warn(f'move_timed rechazado: {msg}')
            resp.success = False
            return resp
        if not 0.0 < req.duration <= 30.0:
            self.get_logger().warn(
                f'move_timed: duration {req.duration} fuera de (0, 30] s')
            resp.success = False
            return resp
        # El mismo límite que el resto del stack: 0.40 m/s y 2.0 rad/s son los
        # máximos MEDIDOS de este robot (manual, cap. 11.3).
        if abs(req.linear) > 0.40 or abs(req.angular) > 2.0:
            self.get_logger().warn(
                f'move_timed: ({req.linear}, {req.angular}) supera los máximos '
                'medidos del robot (0.40 m/s, 2.0 rad/s)')
            resp.success = False
            return resp
        self.get_logger().warn(
            f'move_timed: {req.duration:.1f} s a ({req.linear:.2f} m/s, '
            f'{req.angular:.2f} rad/s). NO pasa por el collision_monitor.')

        async def _mover():
            # `drive_rc_si_units` y no `drive_with_heading`: frena diez veces
            # mejor — 1.1 cm de deriva contra 11.3 (CLAUDE.md).
            fin = self._loop.time() + req.duration
            while self._loop.time() < fin:
                if self._parada_emergencia:
                    break
                await self._rvr.drive_rc_si_units(
                    linear_velocity=float(req.linear),
                    yaw_angular_velocity=math.degrees(float(req.angular)),
                    flags=0)
                # Se repite el comando: el RVR tiene su propio timeout de mando y
                # un solo `drive` no dura 30 s.
                await asyncio.sleep(0.1)
            await self._rvr.drive_stop()

        ok, _, msg = self._pedir(_mover(), 'move_timed',
                                 timeout=req.duration + 10.0)
        resp.success = ok
        return resp

    def _srv_raw_motors(self, req, resp):
        """Control directo de los dos motores. Sin cinemática de por medio."""
        ok, msg = self._mover_permitido()
        if not ok:
            resp.success, resp.message = False, msg
            return resp
        for nombre, modo in (('left_mode', req.left_mode), ('right_mode', req.right_mode)):
            if modo not in (0, 1, 2):
                resp.success, resp.message = False, f'{nombre} debe ser 0 (off), 1 o 2'
                return resp
        for nombre, v in (('left_speed', req.left_speed), ('right_speed', req.right_speed)):
            if not 0 <= v <= 255:
                resp.success, resp.message = False, f'{nombre} va de 0 a 255'
                return resp
        self.get_logger().warn(
            f'raw_motors: L({req.left_mode},{req.left_speed}) '
            f'R({req.right_mode},{req.right_speed}). NO pasa por el collision_monitor, '
            'y NO tiene watchdog: sigue hasta que le mandes modo 0.')
        ok, _, msg = self._pedir(
            self._rvr.raw_motors(
                left_mode=req.left_mode, left_duty_cycle=req.left_speed,
                right_mode=req.right_mode, right_duty_cycle=req.right_speed),
            'raw_motors')
        resp.success, resp.message = ok, msg
        return resp

    def _ir_a(self, x, y, yaw, speed, si, etiqueta):
        """Lo común a `move_to_pose` y `move_to_pos_and_yaw`."""
        ok, msg = self._mover_permitido()
        if not ok:
            self.get_logger().warn(f'{etiqueta} rechazado: {msg}')
            return False
        self.get_logger().warn(
            f'{etiqueta}: a ({x:.2f}, {y:.2f}) yaw {math.degrees(yaw):.0f}° '
            f'a {speed}{" m/s" if si else " (0..128)"}. NO pasa por el '
            'collision_monitor.')
        # 🔴 El yaw va en GRADOS y el marco es el del LOCATOR, que NO es el del
        #    robot: su eje X está 90° girado y se realinea en cada
        #    `reset_locator_x_and_y` (manual, cap. 10). Quien use esto tiene que
        #    saberlo — no es «x metros hacia delante».
        coro = (self._rvr.drive_to_position_si if si
                else self._rvr.drive_to_position_normalized)
        ok, _, _ = self._pedir(
            coro(yaw_angle=math.degrees(yaw), x=float(x), y=float(y),
                 linear_speed=float(speed), flags=0),
            etiqueta, timeout=30.0)
        return ok

    def _srv_move_to_pose(self, req, resp):
        """Ir a una pose del marco del LOCATOR. Ver el aviso en `_ir_a`."""
        o = req.pose.orientation
        yaw = math.atan2(2.0 * (o.w * o.z + o.x * o.y),
                         1.0 - 2.0 * (o.y * o.y + o.z * o.z))
        resp.success = self._ir_a(req.pose.position.x, req.pose.position.y, yaw,
                                  req.speed, req.speed_in_si, 'move_to_pose')
        return resp

    def _srv_move_to_pos_yaw(self, req, resp):
        """Igual, pero con el yaw suelto en vez de un cuaternión."""
        resp.success = self._ir_a(req.position.x, req.position.y, float(req.yaw),
                                  req.speed, req.speed_in_si, 'move_to_pos_and_yaw')
        return resp

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

        # 🔴 APAGAR LO QUE ESTE NODO HAYA ENCENDIDO. Cada `(True)` necesita su
        # `(False)`, también aquí — y este `finally` faltaba:
        #
        #   El 2026-07-31 se añadió `color_detection` y el driver encendía el
        #   sensor al arrancar sin apagarlo nunca. Al matar el nodo, **el LED
        #   blanco se quedaba encendido bajo el chasis**, gastando batería hasta
        #   que alguien apagara el robot. Lo vio el usuario, no el código.
        #
        # Se apaga siempre, no solo si `self._color_detection`: un servicio o una
        # ejecución anterior pueden haberlo dejado encendido, y apagar algo ya
        # apagado es inocuo.
        for coro, etq in ((self._rvr.enable_color_detection(is_enabled=False),
                           'sensor de color'),
                          (self._rvr.set_all_leds(
                              led_group=RvrLedGroups.all_lights.value,
                              led_brightness_values=[0, 0, 0] * 10), 'LEDs')):
            try:
                await coro
            except Exception as e:
                # Que falle apagar una luz no debe impedir soltar el puerto.
                self.get_logger().warn(f'no se pudo apagar {etq}: {e}')

        await self._rvr.sensor_control.clear()
        await self._rvr.close()

    # ⚠️ Esto solo corre con un cierre LIMPIO (SIGINT, que es lo que manda
    #    `ros2 launch`). Con `kill -9` no corre nada y las luces se quedan como
    #    estén: es la razón de parar el nodo con SIGINT y no a lo bruto.


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
# ── ✅ PORTADO EL 2026-07-31: 18 SERVICIOS ────────────────────────────────────
#   De los 16 que esta lista daba por pendientes, se portaron 14 y se añadieron 4
#   que no existían en ROS 1 (`set_leds`, `trigger_led_event`, `set_pos_and_yaw`,
#   y `battery_state` pasó de servicio a TOPIC, que es mejor: la web no tiene que
#   preguntar).
#
# ── 🔴 LO QUE SIGUE SIN ESTAR, Y NO ES CERO ───────────────────────────────────
#
#   SIN EQUIVALENTE — huecos de verdad:
#     · `reset_odom` (std_srvs/Empty)      el driver resetea el locator AL ARRANCAR
#                                          y no hay forma de volver a hacerlo en
#                                          caliente. Para una sesión larga de
#                                          laboratorio, esto se va a notar.
#     · ✅ topic `ambient_light`           YA EXISTE. Este renglón se quedó viejo:
#                                          `/ambient_light` publica (105 mensajes
#                                          en 8 s, medido el 2026-08-11 en rvr-02).
#                                          Lo publica `_h_luz`.
#     · ✅ topic `infrared_messages`       CERRADO el 2026-08-11. Decía: «se puede
#       (InfraredMessage)                  ENVIAR IR pero NO RECIBIR; el tipo ya
#                                          está definido y no lo publica nadie».
#                                          Ya lo publica `_h_ir_mensaje`.
#                                          ⚠️ Que se registre NO prueba que el
#                                          firmware lo entregue: eso lo dice el
#                                          contador de mensajes recibidos, no el
#                                          registro.
#     · ✅ topic `encoders` (Encoder)      YA EXISTE, y también se quedó viejo:
#                                          `/encoders` publica a ~17 Hz (137
#                                          mensajes en 8 s, rvr-02, 2026-08-11).
#                                          Lo publica `_h_encoders`.
#
#   📌 DOS DE LOS CUATRO «HUECOS» LLEVABAN CERRADOS SIN QUE NADIE LOS TACHARA, y
#      el tercero se ha cerrado hoy. Una lista de pendientes que no se poda dice
#      cosas falsas con la misma seguridad que las verdaderas.
#
#   DIFERIDOS A PROPÓSITO, con su razón:
#     · `configure_streaming` / `start_streaming`  pueden romper la telemetría del
#           propio driver: `_registrar_sensores` ya configura el streaming, y
#           reconfigurarlo en caliente deja `/odom` mudo.
#           📝 Aquí se añadía «es el mismo mecanismo por el que
#              `enable_color_detection` no funciona una vez configurado». **Eso
#              era falso** (ver abajo) y se retira: la razón de diferir estos dos
#              sigue en pie por sí sola, pero no se apoya en aquello.
#
#   🔴 Y AQUÍ ESTABA `enable_color`, EN ESTA LISTA, DESDE EL 2026-07-31:
#
#        «`enable_color` (SetBool) 🔴 MEDIDO: NO PUEDE FUNCIONAR como servicio.»
#
#      **Está implementado desde el 2026-08-06**, tres pantallas más arriba en
#      este mismo fichero, y verificado: `/color` no-cero 0 → 53 → 0 y canal claro
#      1 → 1320 → 0 (evidencias 76 y 77).
#
#      Se deja escrito el error en vez de borrarlo, porque el daño que hizo es la
#      parte útil: **ese comentario es el que hizo decir a quien construía la web
#      que el botón de color no podía existir**, y bloqueó la función seis días.
#      La afirmación nunca estuvo medida —el servicio bajo prueba se apagaba a sí
#      mismo dentro de la misma llamada— y sobrevivió a tres revisiones de código
#      porque cada lectura la confirmaba.
#
#      📌 Y sobrevivió UN DÍA MÁS a su propia refutación: se corrigieron los tres
#         sitios evidentes y este, en la lista de «lo que no se implementa», siguió
#         vivo hasta que lo encontró el Claude del PC. **Una trampa documentada
#         también caduca, y hay que ir a buscarla donde no se la espera.**
#     · `cmd_degrees` (DegreesTwist)  `/cmd_vel` en rad/s es el estándar de ROS
#           (REP-103) y es lo que habla Nav2. Un segundo topic de entrada con otras
#           unidades es una fuente de errores, no una comodidad.
#     · `ir_messages` (String)  duplicado sin tipar de `infrared_messages`. Ver
#           la decisión 1, más abajo.
#
# ── 📝 Y EL SDK OFRECE MUCHO MÁS QUE EL DRIVER DE ROS 1 ───────────────────────
#   `SpheroRvrAsync` expone **94 métodos públicos**; este driver usa **27**.
#   Entre lo que NO llega a ROS por ninguna vía hay cosas con uso real en un
#   laboratorio: magnetómetro, temperatura, tensión de batería en voltios,
#   notificaciones de **motor atascado** y **fallo de motor**, aviso de que el
#   robot **va a dormirse**, lecturas IR robot-a-robot y paletas de color.
#
#   ⚠️ Así que «está todo lo del Sphero en ROS» es FALSO hoy. Lo que sí es cierto:
#      está todo lo que el driver de ROS 1 exponía, menos los cuatro huecos de
#      arriba, y con 4 piezas nuevas.
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
