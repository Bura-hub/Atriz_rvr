"""Arranca el robot completo: driver del RVR + descripción + LIDAR.

    ros2 launch atriz_rvr_bringup robot.launch.py

    # sin LIDAR (para probar solo el RVR):
    ros2 launch atriz_rvr_bringup robot.launch.py lidar:=false

Es el punto de entrada único del robot. Los tres nodos que arranca se reparten el
árbol TF, y hay que entender el reparto porque es donde estaba el bloqueante raíz
del proyecto:

    odom ──(rvr_driver)──► base_link ──(robot_state_publisher)──► laser
                                                                    │
                                                    /scan ◄──(ydlidar)

  · `rvr_driver`             publica `odom → base_link`, /odom, /imu, /color
  · `robot_state_publisher`  publica los transforms FIJOS desde el URDF
  · `ydlidar_ros2_driver`    publica /scan con frame_id = `laser`

🔴 **Los nombres tienen que coincidir en TRES sitios.** El `base_frame` del
driver, el link del URDF y el `frame_id` del LIDAR. Si uno se desalinea, el árbol
se parte y **falla en silencio**: `slam_toolbox` se queda esperando un transform
que nunca llega, sin un solo mensaje de error. Es exactamente lo que le pasó al
sistema de ROS 1, donde el driver decía `rvr_base_link` y el LIDAR colgaba de
`base_link`.

VERIFICAR SIEMPRE tras arrancar:

    ros2 run tf2_ros tf2_echo odom laser   # debe resolver
    ros2 topic hz /scan                    # ~10 Hz (medido libre: 11.48 Hz)
    ros2 topic hz /odom                    # ~16.7 Hz
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
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

    ns = LaunchConfiguration('namespace')

    # ── El driver del RVR: publica odom -> base_link ──────────────────────────
    rvr = Node(
        package='atriz_rvr_driver',
        executable='rvr_driver_node',
        name='rvr_driver',
        namespace=ns,
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('rvr_port'),
            # base_frame TIENE que coincidir con el link del URDF y con el
            # frame_id del LIDAR. Ver el aviso de la cabecera.
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            # 60 ms es el mínimo del firmware del RVR: a 50 ms no arranca, y
            # cuantiza a múltiplos de 20. Medido el 2026-07-29.
            'streaming_interval_ms': 60,
        }],
    )

    # ── La descripción: publica base_link -> laser, imu_link, ruedas ──────────
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

    return LaunchDescription([arg_lidar, arg_ns, arg_puerto, rvr, desc, lidar])
