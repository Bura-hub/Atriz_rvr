"""Localización sobre un mapa YA HECHO: map_server + AMCL. Fase 4c.

    # terminal 1 — el robot
    ros2 launch atriz_rvr_bringup robot.launch.py
    # terminal 2 — localización (EN LUGAR DE slam.launch.py)
    ros2 launch atriz_rvr_bringup localizacion.launch.py \\
        mapa:=/home/sphero/atriz_migracion/00_auditoria/evidencia_24_04/mapas/mapa_rodeo.yaml
    # terminal 3 — navegación
    ros2 launch atriz_rvr_bringup nav2.launch.py

═══════════════════════════════════════════════════════════════════════════════
🔴🔴 ESTE LAUNCH SUSTITUYE A `slam.launch.py`. NUNCA LOS DOS A LA VEZ.
═══════════════════════════════════════════════════════════════════════════════
AMCL y `slam_toolbox` publican **los dos** `map -> odom`. Dos nodos publicando el
mismo transform parten el árbol TF **sin dar ningún error**: TF se queda con el
último mensaje que llegue y la pose salta entre las dos estimaciones. Es el mismo
fallo que costó la Fase 4 entera (manual, cap. 9.4).

Por eso este launch **comprueba que slam_toolbox no esté corriendo y aborta si lo
está**. No es paranoia: es el único fallo de este proyecto que nunca ha dado un
mensaje de error.

═══════════════════════════════════════════════════════════════════════════════
CUÁNDO USAR CADA UNO
═══════════════════════════════════════════════════════════════════════════════
    slam.launch.py          construir el mapa. Una vez, con un robot.
    localizacion.launch.py  usarlo. En los 16, todos los días.

Es lo que hace viable la flota: mapear una vez y repartir el `.pgm` con la imagen
dorada, en lugar de 16 SLAM simultáneos construyendo 16 mapas distintos del mismo
laboratorio sin un marco común.

═══════════════════════════════════════════════════════════════════════════════
VERIFICAR TRAS ARRANCAR — y NO basta con que los nodos vivan
═══════════════════════════════════════════════════════════════════════════════
    ros2 lifecycle get /map_server     # active [3]
    ros2 lifecycle get /amcl           # active [3]
    ros2 topic hz /map                 # el mapa se publica (latched)
    ros2 run tf2_ros tf2_echo map odom # ← LA prueba: AMCL está corrigiendo

⚠️ `map -> odom` existiendo **no** prueba que la localización sea buena, solo que
   AMCL publica. Para saber si acertó, mira `/amcl_pose` y su covarianza, o
   compara con dónde está el robot de verdad.

🔴 Y AMCL **no sabe dónde está el robot al arrancar**. `set_initial_pose: true`
   en el YAML le dice (0,0,0). Si el robot no está ahí, hay que corregirlo:

    ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \\
      "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0},
        orientation: {w: 1.0}}}}"
═══════════════════════════════════════════════════════════════════════════════
"""
import os
import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

#: Los nodos que el gestor debe activar, EN ORDEN: primero el mapa, y solo
#: entonces AMCL, que lo necesita para poblar sus partículas.
NODOS_CICLO_VIDA = ['map_server', 'amcl']

#: `comm` del proceso de slam_toolbox (15 caracteres, que es lo que da `ps`).
COMM_SLAM = 'async_slam_tool'


def _slam_vivo() -> bool:
    """¿Está `slam_toolbox` corriendo?

    Se mira por `comm` con `ps` y NO con `pgrep -f`: el patrón de `-f` casa con
    la propia línea de comando de quien lo ejecuta, y en este proyecto eso ya ha
    matado la terminal dos veces (CLAUDE.md).
    """
    r = subprocess.run(['ps', '-eo', 'comm'], capture_output=True, text=True)
    return COMM_SLAM in r.stdout.split()


def _construir(contexto, *args, **kwargs):
    if _slam_vivo():
        raise RuntimeError(
            '\n'
            '🔴 slam_toolbox ESTÁ CORRIENDO. localizacion.launch.py no arranca.\n'
            '\n'
            '   AMCL y slam_toolbox publican los dos `map -> odom`. Juntos parten\n'
            '   el árbol TF SIN DAR NINGÚN ERROR y la pose salta entre las dos\n'
            '   estimaciones. Es el fallo que costó la Fase 4 (manual, cap. 9.4).\n'
            '\n'
            '   Para el SLAM primero:\n'
            "     for x in $(ps -eo pid,comm | awk '$2==\"async_slam_tool\"{print $1}'); "
            'do kill -INT $x; done\n')

    mapa = LaunchConfiguration('mapa').perform(contexto)
    if not os.path.isfile(mapa):
        raise RuntimeError(
            f'\n🔴 el mapa no existe: {mapa}\n'
            '   Hazlo primero con slam.launch.py y guárdalo con el método verificado:\n'
            '     ros2 run nav2_map_server map_saver_cli -f <ruta> '
            '--ros-args -p save_map_timeout:=10.0\n'
            '   (el servicio /slam_toolbox/save_map falla ~1 de cada 3; manual 11.11)\n')

    ns = LaunchConfiguration('namespace')
    sim = LaunchConfiguration('use_sim_time')
    params = LaunchConfiguration('params_file')
    comunes = dict(namespace=ns, output='screen')

    return [
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[params, {'use_sim_time': sim, 'yaml_filename': mapa}],
             **comunes),
        Node(package='nav2_amcl', executable='amcl', name='amcl',
             parameters=[params, {'use_sim_time': sim}], **comunes),

        # 🔴 Sin gestor de ciclo de vida los dos arrancan en `unconfigured`:
        # vivos, listados por `ros2 node list`, y sin publicar ni el mapa ni
        # `map -> odom`. Mismo patrón que slam_toolbox y que Nav2.
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localizacion', namespace=ns, output='screen',
             parameters=[{'use_sim_time': sim,
                          'autostart': LaunchConfiguration('autostart'),
                          'node_names': NODOS_CICLO_VIDA}]),
    ]


def generate_launch_description() -> LaunchDescription:
    bringup = FindPackageShare('atriz_rvr_bringup')
    return LaunchDescription([
        DeclareLaunchArgument(
            'mapa', description='Ruta al .yaml del mapa. OBLIGATORIO.'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [bringup, 'config', 'localizacion_amcl.yaml']),
            description='Configuración de map_server y AMCL, con los valores '
                        'medidos de este robot.'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Llevarlos a `active`. Con false se quedan en '
                        '`unconfigured` y NO HACEN NADA.'),
        OpaqueFunction(function=_construir),
    ])
