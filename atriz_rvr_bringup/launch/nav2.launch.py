"""Arranca Nav2 sobre el robot y SLAM ya en marcha.

    # terminal 1 — el robot
    ros2 launch atriz_rvr_bringup robot.launch.py
    # terminal 2 — SLAM (publica map -> odom)
    ros2 launch atriz_rvr_bringup slam.launch.py
    # terminal 3 — navegación
    ros2 launch atriz_rvr_bringup nav2.launch.py

🔴 **LOS TRES, Y EN ESE ORDEN.** Nav2 no publica el robot ni el mapa: solo
navega. Necesita encontrar ya hechos el árbol TF entero y `map -> odom`. Y
reiniciar el driver por debajo de un `slam_toolbox` ya arrancado lo deja con un
hueco en su buffer TF y deja de procesar, sin dar ningún error (manual, 9.7).

═══════════════════════════════════════════════════════════════════════════════
QUÉ AÑADE, Y QUÉ DA POR HECHO
═══════════════════════════════════════════════════════════════════════════════

    map ──(slam_toolbox)──► odom ──(driver)──► base_footprint ──► base_link ──► laser
    └────────────── esto TIENE que existir ya ──────────────────────────────────┘

Nav2 pone encima: los costmaps, el planificador, el controlador y el árbol de
comportamiento. Y publica `/cmd_vel`, que es lo que el driver ya sabe consumir.

🔴 SON NODOS DE CICLO DE VIDA, IGUAL QUE slam_toolbox

  Arrancan en `unconfigured`: los procesos viven, `ros2 node list` los muestra, y
  **no hacen absolutamente nada**. Es el mismo fallo que costó la Fase 4, y en
  Nav2 lo gestiona el `lifecycle_manager`, que los lleva a `active` en el orden
  correcto — el orden importa, porque los costmaps deben estar activos antes que
  el controlador que los lee.

  Comprobar SIEMPRE tras arrancar:

      ros2 lifecycle get /controller_server     # active [3]
      ros2 lifecycle get /planner_server        # active [3]
      ros2 lifecycle get /bt_navigator          # active [3]

⚠️ EL QoS DE `/scan`, OTRA VEZ

  El driver del LIDAR publica BEST_EFFORT. La capa de obstáculos de Nav2 se
  suscribe con el perfil de datos de sensor, que también es BEST_EFFORT, así que
  **deberían emparejar**. Pero es exactamente el tipo de cosa que este proyecto
  ya ha visto fallar en silencio, y si no emparejan **el costmap se queda vacío
  sin un solo error**: el robot navegaría creyendo que no hay nada delante.

      ros2 topic info /scan --verbose      # deben salir DOS suscriptores:
                                           # slam_toolbox y nav2, ambos BEST_EFFORT

CÓMO MANDARLE UN OBJETIVO, sin RViz

    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \\
      "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0},
        orientation: {w: 1.0}}}}"

  ⚠️ El robot se moverá hasta 0.25 m/s y **no tiene evitación reactiva** más allá
     del costmap: el `collision_monitor` todavía no está configurado. Espacio
     despejado y alguien mirando.
═══════════════════════════════════════════════════════════════════════════════
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

#: Los nodos que el gestor de ciclo de vida debe activar, EN ORDEN.
#: Los costmaps van dentro de controller_server y planner_server, así que estos
#: dos tienen que estar activos antes que bt_navigator, que es quien los usa.
NODOS_CICLO_VIDA = [
    'controller_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'velocity_smoother',
]


def generate_launch_description() -> LaunchDescription:
    bringup = FindPackageShare('atriz_rvr_bringup')

    arg_params = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([bringup, 'config', 'nav2_atriz.yaml']),
        description='Configuración de Nav2. La de Atriz tiene los valores MEDIDOS '
                    'de este robot: radio 0.11 m, 0.40 m/s y 2.0 rad/s máximos, '
                    'LIDAR de 8 m. No uses los del ejemplo de Nav2: su '
                    '`robot_radius` es el doble del real.',
    )
    arg_ns = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Vacío por defecto: el aislamiento entre robots lo da '
                    'ROS_DOMAIN_ID (ARQUITECTURA.md, D1).',
    )
    arg_sim = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='En el robot real, false.',
    )
    arg_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Llevar los nodos a `active` automáticamente. Con false se '
                    'quedan en `unconfigured` y NO HACEN NADA.',
    )

    ns = LaunchConfiguration('namespace')
    params = LaunchConfiguration('params_file')
    sim = LaunchConfiguration('use_sim_time')

    comunes = dict(namespace=ns, output='screen',
                   parameters=[params, {'use_sim_time': sim}])

    nodos = [
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server',
             # `cmd_vel` del controlador va al suavizador, no al robot: el
             # velocity_smoother es quien publica el /cmd_vel final. Sin este
             # remapeo los dos publicarían a la vez y el robot recibiría órdenes
             # contradictorias.
             remappings=[('cmd_vel', 'cmd_vel_nav')], **comunes),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', **comunes),
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', **comunes),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', **comunes),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother',
             remappings=[('cmd_vel', 'cmd_vel_nav'),
                         ('cmd_vel_smoothed', 'cmd_vel')], **comunes),

        # 🔴 El gestor de ciclo de vida. Sin él, los cinco nodos de arriba
        # arrancan en `unconfigured` y se quedan ahí: vivos, listados por
        # `ros2 node list`, y sin hacer nada. Es el mismo fallo que costó la
        # Fase 4 con slam_toolbox (manual, cap. 9.2).
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', namespace=ns, output='screen',
             parameters=[{'use_sim_time': sim,
                          'autostart': LaunchConfiguration('autostart'),
                          'node_names': NODOS_CICLO_VIDA}]),
    ]

    return LaunchDescription([arg_params, arg_ns, arg_sim, arg_autostart] + nodos)
