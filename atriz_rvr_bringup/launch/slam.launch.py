"""Arranca SLAM sobre el robot ya en marcha.

    # terminal 1 — el robot
    ros2 launch atriz_rvr_bringup robot.launch.py
    # terminal 2 — SLAM
    ros2 launch atriz_rvr_bringup slam.launch.py

Se deja APARTE de `robot.launch.py` a propósito: el robot tiene que poder arrancar
sin SLAM (para teleoperar, para diagnosticar, para los scripts de estudiantes), y
SLAM tiene que poder reiniciarse sin tocar el driver ni soltar el puerto serie.

QUÉ COMPLETA DEL ÁRBOL TF

    map ──(slam_toolbox)──► odom ──(driver)──► base_footprint ──► base_link ──► laser
    └── ESTO es lo que añade                   └────────── ya lo daba la Fase 3

🔴 slam_toolbox ES UN NODO DE CICLO DE VIDA EN JAZZY

  Arranca en estado `unconfigured`: **el proceso vive, ROS lo ve en `node list`, y
  no hace absolutamente nada.** No se suscribe a `/scan`, no publica `/map`, y su
  log se queda en «Node using stack size» sin un solo error.

  Verificado el 2026-07-30: la primera versión de este launch usaba un `Node`
  normal y el nodo se quedaba ahí parado. `ros2 topic info /scan --verbose` decía
  `Subscription count: 0`.

  Se arregla con `LifecycleNode` + los dos eventos de transición
  (`configure` → `activate`), que es exactamente lo que hace el
  `online_async_launch.py` oficial de slam_toolbox. Este fichero sigue ese patrón
  en lugar de inventar otro.

  Para comprobarlo a mano:
      ros2 lifecycle get /slam_toolbox      # debe decir `active [3]`
      ros2 lifecycle set /slam_toolbox configure
      ros2 lifecycle set /slam_toolbox activate

✅ EL QoS DE /scan SÍ EMPAREJA — verificado, ya no es una suposición

  El driver del LIDAR publica `/scan` como BEST_EFFORT, y era un riesgo real que
  slam_toolbox pidiera RELIABLE: DDS no los habría emparejado y **no habría
  recibido ni un barrido, sin dar ningún error**.

  Comprobado el 2026-07-30 con `ros2 topic info /scan --verbose`: slam_toolbox se
  suscribe también con **BEST_EFFORT**. Emparejan.

VERIFICAR

    ros2 lifecycle get /slam_toolbox               # active [3]
    ros2 run tf2_ros tf2_echo map base_footprint   # debe resolver
    ros2 topic hz /map                             # ~0.2 Hz (map_update_interval 5 s)
    ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \\
        "{name: {data: mapa_laboratorio}}"
"""
import subprocess

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, OpaqueFunction,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


#: `comm` de los procesos, que es lo que devuelve `ps` — TRUNCADO A 15
#: CARACTERES por el kernel. `async_slam_toolbox_node` sale como
#: `async_slam_tool`, y comprobarlo con el nombre largo no casaría nunca.
#: Verificado el 2026-08-06 copiando `/bin/sleep` con ese nombre y mirando `ps`.
COMM_SLAM = 'async_slam_tool'
COMM_AMCL = 'amcl'


def _vivo(comm: str) -> bool:
    """¿Hay un proceso con ese `comm`?

    🔴 Por `comm` con `ps`, NUNCA con `pgrep -f`: el patrón de `-f` casa con la
       propia línea de comando de quien lo ejecuta, y en este proyecto eso ya ha
       matado la terminal dos veces (CLAUDE.md).
    """
    r = subprocess.run(['ps', '-eo', 'comm'], capture_output=True, text=True)
    return comm in r.stdout.split()


def _guardia_exclusion(contexto, *args, **kwargs):
    """Se niega a arrancar si ya hay alguien publicando `map -> odom`.

    ═══════════════════════════════════════════════════════════════════════════
    POR QUÉ EXISTE, Y POR QUÉ NO ESTABA
    ═══════════════════════════════════════════════════════════════════════════
    `localizacion.launch.py:70-93` comprueba si hay SLAM vivo antes de arrancar
    AMCL. **Aquí no había nada.** La exclusión funcionaba EN UN SOLO SENTIDO:

        SLAM → luego Nav2    el guardia salta          ✅
        Nav2 → luego SLAM    arrancaba tan contento    🔴

    Y el segundo caso deja **dos publicadores de `map -> odom`**: el árbol TF se
    parte, la pose salta entre las dos estimaciones, y **no aparece un solo
    mensaje de error**. Es el fallo que costó la Fase 4 entera.

    Con SSH era poco probable —hay que teclear dos cosas en el orden malo—. Con
    **dos botones en la web** el orden lo elige el alumno, y ese sentido pasa de
    improbable a rutinario. Por eso se escribe ahora.

    ⚠️ Y hay un tercer camino que este guardia TAMPOCO cubre y conviene decirlo:
       **dos `slam.launch.py` a la vez** —uno por SSH y otro por la unidad de
       systemd— darían dos `slam_toolbox`. También se comprueba.

    ═══════════════════════════════════════════════════════════════════════════
    LO QUE ESTE GUARDIA NO PUEDE HACER
    ═══════════════════════════════════════════════════════════════════════════
    🔴 **No cierra la carrera.** Mira `ps` en un instante; si dos clientes piden
       SLAM y Nav2 a la vez, los dos pueden ver el sistema limpio y pasar. Un
       `ps` no puede resolver eso — hace falta un cerrojo, y ese es trabajo de
       quien orqueste las unidades, no de un launch.
    ⚠️ **`ps` ve el proceso, no si está funcionando.** Un `slam_toolbox` en
       `unconfigured` —proceso vivo, cero suscripciones, cero publicaciones—
       bloquearía AMCL sin estar haciendo nada. Es el sesgo seguro: prefiere
       negarse de más a partir el árbol TF.

    📌 Y NO se usa `Conflicts=` de systemd para esto. Pararía al otro **sin
       decir una palabra**: quien lleve veinte minutos mapeando perdería el mapa
       y nadie se enteraría. Cambiar un fallo ruidoso por uno silencioso va en
       la dirección contraria a la de este proyecto.
    """
    if _vivo(COMM_SLAM):
        raise RuntimeError(
            '\n'
            '🔴 YA HAY UN slam_toolbox CORRIENDO. slam.launch.py no arranca.\n'
            '\n'
            '   Dos slam_toolbox publican los dos `map -> odom`. Juntos parten el\n'
            '   árbol TF SIN DAR NINGÚN ERROR y la pose salta entre las dos\n'
            '   estimaciones — el fallo que costó la Fase 4 (manual, cap. 9.4).\n'
            '\n'
            '   Si el que corre es el que quieres, no hagas nada: ya está mapeando.\n'
            '   Si quieres empezar de cero, párale PRIMERO (perderás su mapa):\n'
            "     for x in $(ps -eo pid,comm | awk '$2==\"async_slam_tool\"{print $1}'); "
            'do kill -INT $x; done\n')

    if _vivo(COMM_AMCL):
        raise RuntimeError(
            '\n'
            '🔴 AMCL ESTÁ CORRIENDO (la navegación está levantada).\n'
            '   slam.launch.py no arranca.\n'
            '\n'
            '   AMCL y slam_toolbox publican los dos `map -> odom`. Son\n'
            '   EXCLUYENTES: mapear y navegar sobre un mapa guardado no se hacen\n'
            '   a la vez.\n'
            '\n'
            '   Para la navegación primero:\n'
            '     sudo systemctl stop atriz-nav\n'
            '\n'
            '   📝 Si lo que querías era navegar, ya lo estás haciendo: esto no\n'
            '      hacía falta.\n')

    return []


def generate_launch_description() -> LaunchDescription:
    bringup = FindPackageShare('atriz_rvr_bringup')

    arg_params = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution(
            [bringup, 'config', 'slam_toolbox_atriz.yaml']),
        description='Configuración de slam_toolbox. La de Atriz está ajustada a lo '
                    'MEDIDO de este robot: 8 m de alcance real del X2, 5 cm de '
                    'resolución y 30 cm entre barridos.',
    )
    arg_ns = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Vacío por defecto: el aislamiento entre robots lo da '
                    'ROS_DOMAIN_ID (ARQUITECTURA.md, D1).',
    )
    arg_sim = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='En el robot real, false. true solo con un simulador publicando /clock.',
    )
    arg_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Llevar el nodo a `active` automáticamente. Con false se queda '
                    'en `unconfigured` y NO HACE NADA hasta activarlo a mano.',
    )

    autostart = LaunchConfiguration('autostart')

    # async_slam_toolbox_node, no el sync: el asíncrono no bloquea esperando a
    # procesar cada barrido, que en un Pi 4 con el driver del RVR ya consumiendo
    # ~29.5 % de un núcleo importa. Es lo que recomienda el plan (Fase 4).
    slam = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time'),
             # Si algún día se usa el lifecycle_manager de Nav2, esto pasa a true
             # y el manager se encarga de las transiciones en vez de este launch.
             'use_lifecycle_manager': False},
        ],
    )

    # unconfigured -> inactive
    evento_configurar = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(autostart),
    )

    # inactive -> active, en cuanto la transición anterior termine. Se hace con un
    # manejador de eventos y no con un sleep: encadenar transiciones por tiempo es
    # lo que hace que un arranque falle una vez de cada diez.
    evento_activar = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        ),
        condition=IfCondition(autostart),
    )

    return LaunchDescription([
        arg_params, arg_ns, arg_sim, arg_autostart,
        # 🔴 EL GUARDIA VA PRIMERO, antes que el nodo. Si se pusiera después,
        #    `slam_toolbox` ya se habría lanzado cuando la excepción salta, y
        #    quedarían dos procesos: exactamente lo que viene a evitar.
        OpaqueFunction(function=_guardia_exclusion),
        slam, evento_configurar, evento_activar,
    ])
