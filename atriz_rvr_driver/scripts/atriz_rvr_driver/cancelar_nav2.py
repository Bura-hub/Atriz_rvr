#!/usr/bin/env python3
"""Cancela los objetivos de Nav2 cuando se pulsa la parada de emergencia.

    ros2 run atriz_rvr_driver cancelar_nav2

Lo arranca `nav2.launch.py`. Sin Nav2 corriendo no hace nada y no molesta.

═══════════════════════════════════════════════════════════════════════════════
QUÉ AGUJERO TAPA — y no es el que parece
═══════════════════════════════════════════════════════════════════════════════
La parada de emergencia del driver **sí para el robot**: pone una bandera, llama
a `drive_stop()` y a partir de ahí `_cb_cmd_vel` descarta todo lo que llega. Con
Nav2 mandando a 10 Hz el robot se queda quieto igualmente. Eso funciona.

🔴 EL PROBLEMA ESTÁ AL LIBERAR. `/release_emergency_stop` solo baja la bandera:

    def _srv_liberar_parada(self, _req, resp):
        self._parada_emergencia = False

Si el objetivo de Nav2 sigue vivo —y lo estará, porque el `SimpleProgressChecker`
está relajado a 0.25 m en 15 s (cap. 11.13)— el controlador nunca dejó de
publicar. En el instante en que la bandera baja, **el robot arranca solo**, sin
que nadie haya mandado nada.

Que es exactamente lo contrario de lo que debe hacer una parada de emergencia:
soltarla tiene que dejar el robot QUIETO, no devolverlo a lo que estaba haciendo.

═══════════════════════════════════════════════════════════════════════════════
POR QUÉ UN NODO APARTE Y NO DENTRO DEL DRIVER
═══════════════════════════════════════════════════════════════════════════════
Porque **el driver tiene que funcionar sin Nav2**. Es la misma razón por la que
SLAM y la navegación viven en launches separados: los estudiantes teleoperan sin
Nav2, y meter un cliente de acción de Nav2 dentro del driver lo ataría a un
paquete que la mitad del tiempo no está corriendo.

Aquí la responsabilidad vive con lo que hay que cancelar: este nodo lo arranca
`nav2.launch.py`, y si Nav2 no está, este nodo tampoco.

═══════════════════════════════════════════════════════════════════════════════
CÓMO CANCELA
═══════════════════════════════════════════════════════════════════════════════
Con el servicio `_action/cancel_goal` de cada acción, y un `CancelGoal.Request`
**vacío**. En `action_msgs` eso no significa «no cancelar nada»: un `goal_id` a
cero y un `stamp` a cero es la política **CANCEL_ALL**. Se evita así tener que
seguir la pista de los handles de objetivo, que es justo lo que fallaría cuando
el objetivo lo lanzó otro proceso (la web, RViz2, un script).

📝 Se usa un cliente de servicio, no `ros2 action list`: el descubrimiento de DDS
   no es autoritativo — omitió 1 de los 18 servicios del driver (CLAUDE.md).
"""
import rclpy
from action_msgs.srv import CancelGoal
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import Empty

#: Las acciones que mueven el robot. `navigate_to_pose` es la que usa la web,
#: pero las otras dos también conducen y hay que cortarlas igual.
ACCIONES = (
    'navigate_to_pose',
    'navigate_through_poses',
    'follow_path',
)

#: 🔴 VOLATILE, no TRANSIENT_LOCAL. En un SUSCRIPTOR, `TRANSIENT_LOCAL` no añade
#: ninguna garantía: solo EXIGE que el publicador también lo sea, y ni
#: `ros2 topic pub` ni rosbridge lo son por defecto. Esa fue la tercera causa por
#: la que la parada de emergencia falló en silencio (manual, cap. 15.1).
QOS_PARADA = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class CanceladorNav2(Node):
    def __init__(self) -> None:
        super().__init__('cancelar_nav2')

        # Cancelar es una llamada bloqueante desde un callback de suscripción:
        # sin grupo reentrante el ejecutor se bloquearía a sí mismo.
        g = ReentrantCallbackGroup()
        self._clientes = {
            a: self.create_client(CancelGoal, f'/{a}/_action/cancel_goal',
                                  callback_group=g)
            for a in ACCIONES
        }

        # Los tres nombres, igual que en el driver. El del namespace absoluto es
        # el que usa la web y el que faltaba en ROS 1; los otros dos se mantienen
        # porque un botón de emergencia con un nombre de más no cuesta nada, y
        # con uno de menos el modo de fallo es «no llega el mensaje».
        for topico in ('emergency_stop', 'is_emergency_stop',
                       '/rvr/emergency_stop'):
            self.create_subscription(Empty, topico, self._cb_parada,
                                     QOS_PARADA, callback_group=g)

        self.get_logger().info(
            'cancelador de Nav2 activo — escuchando emergency_stop, '
            'is_emergency_stop y /rvr/emergency_stop')

    def _cb_parada(self, _msg: Empty) -> None:
        self.get_logger().error('PARADA DE EMERGENCIA — cancelando objetivos de Nav2')
        for accion, cli in self._clientes.items():
            # Sin esperar: si Nav2 no está corriendo esto no debe bloquear la
            # parada. El driver ya frenó los motores por su cuenta.
            if not cli.service_is_ready():
                self.get_logger().warn(f'  {accion}: no disponible (¿Nav2 apagado?)')
                continue
            # Request vacío = CANCEL_ALL (goal_id y stamp a cero).
            fut = cli.call_async(CancelGoal.Request())
            fut.add_done_callback(
                lambda f, a=accion: self._informar(a, f))

    def _informar(self, accion: str, fut) -> None:
        try:
            resp = fut.result()
        except Exception as e:                      # noqa: BLE001
            self.get_logger().error(f'  {accion}: falló la cancelación: {e}')
            return
        n = len(resp.goals_canceling)
        if n:
            self.get_logger().warn(f'  {accion}: {n} objetivo(s) cancelado(s)')
        else:
            # No es un fallo: lo normal es que no hubiera nada navegando.
            self.get_logger().info(f'  {accion}: nada que cancelar')


def main(args=None) -> None:
    rclpy.init(args=args)
    nodo = CanceladorNav2()
    ejecutor = MultiThreadedExecutor()
    ejecutor.add_node(nodo)
    try:
        ejecutor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
