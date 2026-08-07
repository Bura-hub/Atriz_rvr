#!/usr/bin/env python3
"""Supervisor de SLAM y Nav2: los arranca, los para, y dice si FUNCIONAN.

    ros2 run atriz_rvr_driver supervisor_navegacion

Lo arranca `robot.launch.py`, bajo `atriz-robot.service`.

═══════════════════════════════════════════════════════════════════════════════
POR QUÉ ES UN NODO APARTE Y NO ESTÁ EN EL DRIVER
═══════════════════════════════════════════════════════════════════════════════
Tres razones, en orden de peso:

1. 🔴 **Aísla el privilegio.** Este nodo llama a `systemctl start|stop`, que
   necesita una regla de polkit. Metido en el driver, esa capacidad se la
   quedaría el proceso que YA tiene el puerto serie, los motores, la parada de
   emergencia y 19 servicios expuestos por rosbridge. Aparte, la regla se acota
   a un ejecutable y a dos unidades, y quitársela mañana es parar una unidad.

2. **No comparte `g_srv`.** Los 19 servicios del driver van en un
   `MutuallyExclusiveCallbackGroup` donde `_pedir()` espera hasta 5,0 s al RVR.
   Un RVR dormido dejaría el botón de SLAM esperando por algo que no tiene nada
   que ver con SLAM.
   📝 Y el matiz que se corrigió el 2026-08-06: eso **no** bloquea
      `/release_emergency_stop`, que vive en `g_cmd`. El límite es real, el
      sujeto no era ese.

3. 🔴 **El estado de SLAM no puede vivir en el ciclo de vida del driver.** Si el
   driver se reinicia, `slam_toolbox` SOBREVIVE —vivo y mudo— y una señal
   alojada en el driver se pondría a cero justo en el caso que existe para
   detectar.

📝 Precedente en este mismo repositorio: `cancelar_nav2.py`, nodo aparte del
   mismo paquete, cuya cabecera lleva escrito el mismo argumento.

📝 **Si este nodo muere, SLAM y Nav2 siguen corriendo**: son unidades de
   systemd, no hijos suyos. Se pierden el botón y la señal, y la web lo ve
   porque `latido` deja de avanzar.

═══════════════════════════════════════════════════════════════════════════════
LO QUE ESTÁ MEDIDO, Y POR QUÉ EL DISEÑO ES ASÍ
═══════════════════════════════════════════════════════════════════════════════
Evidencia 79 (2026-08-07, n=2 sobre rvr-01):

    Nav2 hasta ACEPTAR OBJETIVOS      24,489 s · 24,044 s   (dispersión 0,44)
    systemctl start --no-block        0,053 · 0,054 · 0,052 · 0,050 s
    systemctl start SIN --no-block    26,1 s        🔴

🔴 **`--no-block` no es una precaución: es obligatorio, con tres órdenes de
   magnitud de margen.** Los tres plazos de la cadena web son de 5,0 s
   —`_pedir()` del driver, `default_call_service_timeout` de rosbridge y el
   `ms = 5000` del cliente—. Un servicio que esperase a `systemctl` daría
   **timeout sobre una operación que sí funcionó**.

🔴 **Un solo `start` sin mapa deja la unidad LATCHEADA.** `StartLimitBurst=3`
   cuenta *arranques*, no clics: el inicial más dos reintentos automáticos son
   ya los tres, en ~40 s. De `failed` solo se sale con `reset-failed`, o sea con
   privilegio, que nadie tiene desde el navegador.
   → **Por eso este nodo se NIEGA antes de llamar a systemctl.** Un `isfile` de
     coste cero evita el único estado del que la web no puede salir sola.
"""
import os
import subprocess
import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.action import ActionClient
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool
from nav2_msgs.action import NavigateToPose
from atriz_rvr_msgs.msg import EstadoNavegacion

#: `comm` de los procesos. **Truncado a 15 caracteres por el kernel**, que es lo
#: que devuelve `ps`: `async_slam_toolbox_node` sale como `async_slam_tool`.
#: Verificado el 2026-08-06 copiando `/bin/sleep` con ese nombre.
COMM_SLAM = 'async_slam_tool'
COMM_AMCL = 'amcl'

#: Sin barrido durante este tiempo, se considera CIEGO. Sale del ritmo de
#: `/scan` (~10 Hz medido, 11,48 libre): 20 mensajes perdidos ≈ 2 s. El umbral
#: se expresa en mensajes y se traduce, que es la regla de este proyecto —un
#: umbral en milisegundos se queda obsoleto en cuanto cambia el ritmo.
SIN_SCAN_S = 2.0

#: Tope del estado ARRANCANDO. Sale de `TimeoutStartSec=120` de la unidad, no de
#: un número inventado. Pasado esto se va a FALLO **con motivo**: nunca un
#: «arrancando» eterno, que es como se disfraza un fallo de arranque.
TOPE_ARRANQUE_S = 120.0


class Supervisor(Node):

    def __init__(self):
        super().__init__('supervisor_navegacion')

        self.declare_parameter('unidad_slam', 'atriz-slam.service')
        self.declare_parameter('unidad_nav', 'atriz-nav.service')
        # 🔴 EL MISMO SITIO QUE `atriz-nav.sh`, Y POR LA MISMA VIA.
        #    El script hace `MAPA="${ATRIZ_MAPA:-$HOME/atriz_ws/src/.../aula.yaml}"`.
        #    Si aqui se mirara otro sitio, este nodo diria «hay mapa» y la unidad
        #    fallaria por no encontrarlo — o al reves, que es peor: el boton
        #    deshabilitado sobre un mapa que si esta. Paso el 2026-08-07: el
        #    launch imponia la ruta INSTALADA y el script usa la FUENTE.
        self.declare_parameter(
            'mapa', os.environ.get('ATRIZ_MAPA') or os.path.expanduser(
                '~/atriz_ws/src/Atriz_rvr/atriz_rvr_bringup/maps/aula.yaml'))
        p = self.get_parameter
        self._u_slam = p('unidad_slam').value
        self._u_nav = p('unidad_nav').value
        self._mapa = p('mapa').value

        #: Instante en que se aceptó cada petición de arranque. None = no está
        #: arrancando. Es lo que da `*_arrancando_s` y lo que corta el tope.
        self._t_pedido = {'slam': None, 'nav': None}

        qos_tel = QoSProfile(depth=10,
                             reliability=QoSReliabilityPolicy.BEST_EFFORT,
                             durability=QoSDurabilityPolicy.VOLATILE)
        # 🔴 BEST_EFFORT: `/scan` lo publica así. Con el RELIABLE por defecto de
        #    rclpy este nodo NO RECIBIRÍA NADA, sin error ni aviso, y pintaría
        #    CIEGO sobre un robot sano. Es la trampa nº 1 de este proyecto.
        self._t_scan = None
        self.create_subscription(LaserScan, 'scan', self._cb_scan, qos_tel)

        self._pub = self.create_publisher(
            EstadoNavegacion, 'estado_navegacion', 10)
        self._latido = 0

        # El testigo de Nav2: un CLIENTE de acción, no `ros2 action list`. El
        # descubrimiento DDS no es autoritativo — ya omitió 1 de 19 servicios.
        self._cli_nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        g = MutuallyExclusiveCallbackGroup()
        self.create_service(SetBool, 'pedir_slam', self._srv_slam,
                            callback_group=g)
        self.create_service(SetBool, 'pedir_nav', self._srv_nav,
                            callback_group=g)
        self.create_timer(1.0, self._publicar, callback_group=g)

        self.get_logger().info(
            f'supervisor_navegacion en pie · slam={self._u_slam} '
            f'nav={self._u_nav} mapa={self._mapa}')

    # ── Utilidades ───────────────────────────────────────────────────────────
    def _cb_scan(self, _msg):
        self._t_scan = time.monotonic()

    def _vivo(self, comm: str) -> bool:
        """¿Hay un proceso con ese `comm`?

        🔴 Por `comm` con `ps`, NUNCA con `pgrep -f`: el patrón de `-f` casa con
           la propia línea de comando de quien lo ejecuta, y en este proyecto eso
           ya ha matado la terminal dos veces.
        """
        try:
            r = subprocess.run(['ps', '-eo', 'comm'], capture_output=True,
                               text=True, timeout=3)
            return comm in r.stdout.split()
        except Exception:                                        # noqa: BLE001
            return False

    def _systemd(self, unidad: str) -> dict:
        """`ActiveState`, `Result` y `NRestarts` de una unidad.

        Lectura pura: NO necesita privilegio. Medido: 0,05 s por consulta.
        """
        try:
            r = subprocess.run(
                ['systemctl', 'show', unidad, '-p', 'ActiveState',
                 '-p', 'Result', '-p', 'NRestarts', '-p', 'LoadState'],
                capture_output=True, text=True, timeout=5)
            d = dict(l.split('=', 1) for l in r.stdout.strip().splitlines()
                     if '=' in l)
            return d
        except Exception as e:                                   # noqa: BLE001
            self.get_logger().warn(f'no se pudo consultar {unidad}: {e}')
            return {}

    def _latcheado(self, unidad: str) -> bool:
        """¿Está `failed` por haber agotado el StartLimit?

        Se mira el journal porque `systemctl show` no expone «start-limit-hit»
        como tal en esta versión: `Result` dice `exit-code` en los dos casos.
        """
        d = self._systemd(unidad)
        if d.get('ActiveState') != 'failed':
            return False
        try:
            r = subprocess.run(
                ['journalctl', '-u', unidad, '--since', '-5 min',
                 '--no-pager', '-q'],
                capture_output=True, text=True, timeout=5)
            return 'repeated too quickly' in r.stdout
        except Exception:                                        # noqa: BLE001
            return False

    def _hay_mapa(self) -> bool:
        """El mapa Y su imagen.

        🔴 Los dos. Un `.yaml` presente con su `.pgm` ausente hace fallar a
           `map_server` **después** de arrancar la unidad, y eso consume el
           presupuesto de reintentos igual que no tener mapa. `image:` es
           relativo al directorio del yaml, no al de trabajo.
        """
        if not os.path.isfile(self._mapa):
            return False
        try:
            with open(self._mapa) as f:
                for linea in f:
                    if linea.startswith('image:'):
                        img = linea.split(':', 1)[1].strip()
                        if not os.path.isabs(img):
                            img = os.path.join(os.path.dirname(self._mapa), img)
                        return os.path.isfile(img)
        except Exception:                                        # noqa: BLE001
            return False
        return False

    # ── Los servicios: PEDIR, no arrancar ────────────────────────────────────
    def _pedir(self, cual: str, unidad: str, encender: bool, resp):
        """Encola el arranque o el paro y VUELVE. Nunca espera al efecto.

        🔴 `success=true` significa **petición ACEPTADA**, jamás «arrancado».
           Quien dice si funciona es `/estado_navegacion`. Este proyecto tiene
           medido tres veces un `success=True` sobre un efecto que no ocurrió.
        """
        if encender:
            # ── Los rechazos, ANTES de tocar systemd ─────────────────────────
            # 🔴 SE DEVUELVEN **TODOS** LOS MOTIVOS, no el primero que salte.
            #    Con «no hay mapa» a secas, quien tuviera ADEMAS SLAM corriendo
            #    iría a hacer un mapa, volvería, y se estrellaría contra la
            #    exclusión: dos viajes para un problema que se podía contar de
            #    una vez. Detectado el 2026-08-07 probando /pedir_nav.
            motivos = []
            if self._systemd(unidad).get('LoadState') == 'not-found':
                motivos.append(f'{unidad} no está instalada en este robot '
                               f'(falta el fichero; no es un fallo pasajero)')
            if self._latcheado(unidad):
                motivos.append(
                    f'{unidad} está bloqueada por reintentos agotados: hace '
                    f'falta «sudo systemctl reset-failed {unidad}» DESDE EL '
                    f'ROBOT, no se puede desde la web')
            if cual == 'nav':
                if not self._hay_mapa():
                    motivos.append(f'no hay mapa legible en {self._mapa}')
                if self._vivo(COMM_SLAM):
                    motivos.append('slam_toolbox está corriendo: SLAM y AMCL '
                                   'publican los dos map->odom y son excluyentes')
            if cual == 'slam' and self._vivo(COMM_AMCL):
                motivos.append('la navegación está levantada (AMCL): SLAM y '
                               'AMCL son excluyentes')
            if motivos:
                resp.success = False
                resp.message = ' · '.join(motivos)
                return resp

        # 🔴 `--no-block` SIEMPRE. Medido: sin él, `systemctl start` bloquea
        #    26,1 s, cinco veces el plazo de 5,0 s de la cadena web, y el botón
        #    daría timeout sobre algo que sí funcionó. Con él, 0,05 s.
        accion = 'start' if encender else 'stop'
        try:
            r = subprocess.run(['systemctl', accion, '--no-block', unidad],
                               capture_output=True, text=True, timeout=10)
        except Exception as e:                                   # noqa: BLE001
            resp.success = False
            resp.message = f'no se pudo encolar {accion} de {unidad}: {e}'
            return resp

        if r.returncode != 0:
            resp.success = False
            # El caso más probable aquí es que falte la regla de polkit. Se dice
            # con todas las letras en vez de devolver un error genérico.
            resp.message = (f'systemctl {accion} devolvió {r.returncode}: '
                            f'{(r.stderr or "").strip()[:200]}')
            return resp

        self._t_pedido[cual] = time.monotonic() if encender else None
        resp.success = True
        resp.message = (f'petición ACEPTADA, no arrancado todavía: mira '
                        f'/estado_navegacion' if encender else
                        f'paro encolado')
        self.get_logger().info(f'{cual}: {accion} encolado')
        return resp

    def _srv_slam(self, req, resp):
        return self._pedir('slam', self._u_slam, bool(req.data), resp)

    def _srv_nav(self, req, resp):
        return self._pedir('nav', self._u_nav, bool(req.data), resp)

    # ── El estado, por EFECTO ────────────────────────────────────────────────
    def _hay_scan(self) -> bool:
        return (self._t_scan is not None
                and time.monotonic() - self._t_scan < SIN_SCAN_S)

    def _evaluar(self, cual: str, unidad: str, m: EstadoNavegacion):
        """Decide el estado de uno de los dos. Nunca por `is-active` a secas."""
        d = self._systemd(unidad)
        estado_sd = d.get('ActiveState', '')
        latcheado = self._latcheado(unidad)
        t_ped = self._t_pedido[cual]
        transcurrido = -1.0 if t_ped is None else float(time.monotonic() - t_ped)

        # El testigo por efecto de cada uno.
        if cual == 'slam':
            vivo = self._vivo(COMM_SLAM)
            funciona = vivo and self._hay_scan()
        else:
            vivo = self._vivo(COMM_AMCL)
            # 🔴 `server_is_ready()` en proceso, no `ros2 action list`. Y el
            #    action server existe desde `on_configure`: por eso se exige
            #    ADEMÁS que el proceso esté.
            funciona = vivo and self._cli_nav.server_is_ready() and self._hay_scan()

        detalle = ''
        if latcheado:
            estado = EstadoNavegacion.FALLO
            detalle = (f'bloqueada por reintentos agotados: hace falta '
                       f'«sudo systemctl reset-failed {unidad}» desde el robot')
            self._t_pedido[cual] = None
        elif estado_sd == 'failed':
            estado = EstadoNavegacion.FALLO
            detalle = f'la unidad falló ({d.get("Result", "?")})'
            self._t_pedido[cual] = None
        elif funciona:
            estado = EstadoNavegacion.FUNCIONANDO
            self._t_pedido[cual] = None
            transcurrido = -1.0
        elif t_ped is not None and transcurrido < TOPE_ARRANQUE_S:
            # 🔴 ARRANCANDO VA ANTES QUE CIEGO, y el orden importa.
            #    Al revés —como estaba— un arranque perfectamente normal pasaba
            #    por CIEGO durante ~1 s: los nodos ya existen y `/scan` todavía
            #    no ha llegado. Medido el 2026-08-07 arrancando Nav2 de verdad:
            #        0,7 s  CIEGO  «encendido pero SIN barrido»
            #        1,4 s  ARRANCANDO
            #        3,4 s  FUNCIONANDO
            #    La web habría pintado una alarma naranja en mitad de un
            #    arranque sano, y el alumno habría ido a buscar una avería que
            #    no existe. CIEGO significa «está levantado y NO puede
            #    funcionar», no «todavía está subiendo».
            estado = EstadoNavegacion.ARRANCANDO
        elif estado_sd in ('active', 'activating') and vivo and not self._hay_scan():
            # 🔴 El estado que `is-active` esconde: encendido y sin barrido. El
            #    collision_monitor bloquea el movimiento y el robot PARECE
            #    averiado (medido: 0,0 cm contra 9,9).
            #    Aquí ya se sabe que NO está arrancando: o nunca se pidió, o el
            #    plazo se agotó. Es un ciego de verdad — típicamente alguien
            #    llamó a `/stop_scan` con la navegación en marcha.
            estado = EstadoNavegacion.CIEGO
            detalle = ('encendido pero SIN barrido: no puede funcionar. '
                       'Enciéndelo con /start_scan')
        elif t_ped is not None:
            estado = EstadoNavegacion.FALLO
            detalle = (f'no llegó a funcionar en {TOPE_ARRANQUE_S:.0f} s '
                       f'(medido: Nav2 tarda ~24 s)')
            self._t_pedido[cual] = None
            transcurrido = -1.0
        elif d.get('LoadState') == 'not-found':
            # 🔴 UNA UNIDAD QUE NO EXISTE NO ESTÁ «APAGADA». `systemctl show`
            #    devuelve ActiveState=inactive para unidades inexistentes, así
            #    que sin mirar LoadState la web pintaría un botón de arranque
            #    para algo que no puede arrancar — y el clic daría un error que
            #    parece del robot. Hoy `atriz-slam.service` NO existe todavía.
            estado = EstadoNavegacion.DESCONOCIDO
            detalle = (f'{unidad} no está instalada en este robot: '
                       f'no se puede arrancar')
        elif estado_sd == 'inactive':
            estado = EstadoNavegacion.APAGADO
        elif estado_sd == '':
            estado = EstadoNavegacion.DESCONOCIDO
            detalle = 'no se pudo consultar a systemd'
        else:
            # `active` sin testigo y sin petición en curso: alguien lo arrancó
            # por SSH y no está funcionando, o acaba de caerse.
            estado = EstadoNavegacion.MUDO
            detalle = f'la unidad dice {estado_sd} pero no hay señal de que funcione'

        if cual == 'slam':
            m.slam, m.slam_detalle = estado, detalle
            m.slam_arrancando_s = transcurrido
            m.slam_latcheado = latcheado
        else:
            m.nav, m.nav_detalle = estado, detalle
            m.nav_arrancando_s = transcurrido
            m.nav_latcheado = latcheado

    def _publicar(self):
        try:
            m = EstadoNavegacion()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = 'base_link'
            m.latido = int(self._latido)
            m.hay_mapa = self._hay_mapa()
            self._evaluar('slam', self._u_slam, m)
            self._evaluar('nav', self._u_nav, m)
            self._pub.publish(m)
            self._latido += 1
        except Exception as e:                                   # noqa: BLE001
            # Nunca se propaga: un latido roto no puede tumbar el nodo, y
            # callarlo sería el fallo silencioso de siempre.
            self.get_logger().error(f'fallo publicando /estado_navegacion: {e}')


def main(args=None):
    rclpy.init(args=args)
    n = Supervisor()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
