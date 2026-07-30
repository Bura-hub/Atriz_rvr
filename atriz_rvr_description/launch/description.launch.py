"""Publica la descripción del robot y su árbol TF.

    ros2 launch atriz_rvr_description description.launch.py

Arranca `robot_state_publisher`, que lee el URDF y publica los transforms FIJOS
del robot:

    base_footprint -> base_link -> { laser, imu_link, wheel_left, wheel_right }

🔴 **Esto NO publica `odom -> base_link`.** Ese lo publica el driver
(`atriz_rvr_driver`), porque es el único que sabe dónde está el robot. Los dos
juntos forman el árbol completo:

    odom ──(driver)──> base_link ──(robot_state_publisher)──> laser

Antes de la Fase 3 esa cadena estaba **partida**: el driver publicaba
`rvr_base_link` y el LIDAR colgaba de `base_link`, sin nada que los uniera.

CÓMO COMPROBAR QUE EL ÁRBOL ESTÁ ENTERO

    ros2 run tf2_tools view_frames        # genera un PDF con el árbol
    ros2 run tf2_ros tf2_echo odom laser  # debe resolver la cadena completa

Si `tf2_echo odom laser` da «Could not find a connection», el árbol sigue roto:
o no está corriendo el driver, o los nombres de frame no coinciden.

⚠️ **Sin `joint_state_publisher`, y a propósito.** Todos los joints del URDF son
`fixed`, así que no hay estados de articulación que publicar. El RVR no expone la
posición angular de sus ruedas —solo conteos de encoder acumulados—, de modo que
declararlas móviles dejaría a `robot_state_publisher` esperando datos que nunca
llegarían. Ver el comentario de las ruedas en el xacro.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg = FindPackageShare('atriz_rvr_description')
    xacro_por_defecto = PathJoinSubstitution([pkg, 'urdf', 'rvr.urdf.xacro'])

    arg_modelo = DeclareLaunchArgument(
        'model', default_value=xacro_por_defecto,
        description='Ruta al .urdf.xacro del robot',
    )
    arg_ns = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Namespace del robot (p. ej. rvr_01). Vacío = sin namespace. '
                    'Ver ARQUITECTURA.md, Decisión 1: el aislamiento entre robots '
                    'lo da ROS_DOMAIN_ID, no el namespace.',
    )
    arg_sim = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='true solo si hay un simulador publicando /clock. En el robot real, false.',
    )

    # `xacro <fichero>` se ejecuta AL LANZAR, no al compilar: así se pueden pasar
    # argumentos y no hay que regenerar el .urdf a mano cada vez que se toca una
    # medida. El espacio tras 'xacro' es necesario: Command concatena literalmente.
    descripcion = Command(['xacro ', LaunchConfiguration('model')])

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[{
            # ParameterValue con value_type=str evita que rclpy intente adivinar
            # el tipo del XML y lo interprete como algo que no es.
            'robot_description': descripcion,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription([arg_modelo, arg_ns, arg_sim, rsp])
