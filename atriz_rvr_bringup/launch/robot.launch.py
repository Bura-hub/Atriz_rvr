"""Arranca el robot completo: driver del RVR + descripción + LIDAR.

    ros2 launch atriz_rvr_bringup robot.launch.py

    # sin LIDAR (para probar solo el RVR):
    ros2 launch atriz_rvr_bringup robot.launch.py lidar:=false

Es el punto de entrada único del robot. Los tres nodos que arranca se reparten el
árbol TF, y hay que entender el reparto porque es donde estaba el bloqueante raíz
del proyecto:

    odom ──(rvr_driver)──► base_footprint ──(robot_state_publisher)──► base_link ──► laser
                                                                                       │
                                                                       /scan ◄──(ydlidar)

  · `rvr_driver`             publica `odom → base_footprint`, /odom, /imu, /color
  · `robot_state_publisher`  publica los transforms FIJOS desde el URDF
                             (`base_footprint → base_link → laser / imu_link / ruedas`)
  · `ydlidar_ros2_driver`    publica /scan con frame_id = `laser`

🔴 **Los nombres tienen que coincidir en TRES sitios.** El `base_frame` del
driver, el link del URDF y el `frame_id` del LIDAR. Si uno se desalinea, el árbol
se parte y **falla en silencio**: `slam_toolbox` se queda esperando un transform
que nunca llega, sin un solo mensaje de error. Es exactamente lo que le pasó al
sistema de ROS 1, donde el driver decía `rvr_base_link` y el LIDAR colgaba de
`base_link`.

🔴 **El RVR se duerme solo si nadie le habla**, y cuando lo hace el nodo se queda
vivo publicando CERO sin dar un error (manual, cap. 9.8). El driver lo previene
hablándole cada 30 s —y publicando `/battery_state` de paso— y, si aun así deja
de llegar telemetría, lo dice y trata de reanudarla. Para reproducir el fallo a
propósito: `keepalive_period:=0.0`.

VERIFICAR SIEMPRE tras arrancar:

    ros2 run tf2_ros tf2_echo odom base_footprint   # ← LA prueba: es lo que pide SLAM
    ros2 run tf2_ros tf2_echo odom laser            # la cadena completa
    ros2 topic hz /scan                    # ~10 Hz (medido libre: 11.48 Hz)
    ros2 topic hz /odom                    # ~16.7 Hz  ← si es 0, ¿se durmió?
    ros2 topic echo /battery_state --once  # llega cada 30 s (es el keepalive)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    bringup = FindPackageShare('atriz_rvr_bringup')
    descripcion = FindPackageShare('atriz_rvr_description')

    arg_lidar = DeclareLaunchArgument(
        'lidar', default_value='true',
        description='Arrancar el driver del YDLIDAR X2. false para probar solo el RVR.',
    )
    arg_ns = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Namespace del robot. El aislamiento entre robots lo da '
                    'ROS_DOMAIN_ID, no esto: ver ARQUITECTURA.md, Decisión 1.',
    )
    arg_puerto = DeclareLaunchArgument(
        'rvr_port', default_value='/dev/rvr',
        description='Puerto del RVR. /dev/rvr es un symlink de udev a ttyAMA0 (PL011).',
    )
    # 🔴 Estos dos existen porque el RVR SE DUERME SOLO si nadie le habla, y sin
    # ellos el nodo se queda vivo publicando cero sin dar un error (manual 9.8).
    # Se exponen como argumentos sobre todo para poder DESACTIVARLOS a propósito
    # y reproducir el fallo — que es como se mide el timeout real del RVR.
    arg_keepalive = DeclareLaunchArgument(
        'keepalive_period', default_value='30.0',
        description='Cada cuántos segundos se le habla al RVR para que no se '
                    'duerma. 0 lo desactiva: el robot se dormirá y dejará de '
                    'publicar. Publica battery_state en cada latido.',
    )
    arg_silencio = DeclareLaunchArgument(
        'silence_timeout', default_value='3.0',
        description='Segundos sin telemetría del RVR antes de avisar e intentar '
                    'reanudar el streaming. 0 desactiva el detector.',
    )

    ns = LaunchConfiguration('namespace')

    # ── El driver del RVR: publica odom -> base_footprint ─────────────────────
    rvr = Node(
        package='atriz_rvr_driver',
        executable='rvr_driver_node',
        name='rvr_driver',
        namespace=ns,
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('rvr_port'),
            # 🔴 base_footprint, NO base_link. Un frame solo puede tener UN
            # padre en TF, y el URDF ya publica base_footprint->base_link. Si el
            # driver publicara odom->base_link, base_link tendría dos padres y el
            # árbol se partiría en dos otra vez.
            # Es también lo que slam_toolbox pide en su `base_frame`.
            'base_frame': 'base_footprint',
            'odom_frame': 'odom',
            # 60 ms es el mínimo del firmware del RVR: a 50 ms no arranca, y
            # cuantiza a múltiplos de 20. Medido el 2026-07-29.
            'streaming_interval_ms': 60,
            'keepalive_period': ParameterValue(
                LaunchConfiguration('keepalive_period'), value_type=float),
            'silence_timeout': ParameterValue(
                LaunchConfiguration('silence_timeout'), value_type=float),
        }],
    )

    # ── La descripción: base_footprint -> base_link -> laser, imu_link, ruedas ─
    desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([descripcion, 'launch', 'description.launch.py'])
        ),
        launch_arguments={'namespace': ns}.items(),
    )

    # ── El LIDAR: publica /scan con frame_id 'laser' ──────────────────────────
    # La configuración vive en config/ydlidar_x2.yaml, con la procedencia de cada
    # valor documentada. Dos parámetros están SIN VERIFICAR y pueden arruinar el
    # mapa en silencio: `inverted` (mapa espejado) y `frequency` (puede que el X2
    # de canal único lo ignore). Léelo antes de mapear.
    lidar = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        namespace=ns,
        output='screen',
        condition=IfCondition(LaunchConfiguration('lidar')),
        parameters=[PathJoinSubstitution([bringup, 'config', 'ydlidar_x2.yaml'])],
    )

    return LaunchDescription([
        arg_lidar, arg_ns, arg_puerto, arg_keepalive, arg_silencio,
        rvr, desc, lidar,
    ])
