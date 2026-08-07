"""Instalación del paquete atriz_rvr_driver (ament_python).

Portado de catkin el 2026-07-30. Diferencias con la versión anterior:

  · Ya NO se usa `catkin_pkg.python_setup.generate_distutils_setup()`, sino
    setuptools directamente. Y sí se puede invocar a mano (`python3 setup.py`),
    aunque lo normal es dejar que lo haga `colcon build`.
  · `scripts=[...]` -> `entry_points={'console_scripts': [...]}`. En ROS 2 los
    ejecutables se declaran así, y `ros2 run` los encuentra por ese nombre.
  · Desaparece `sphero_sim` de la lista de paquetes: NO EXISTE en el repositorio.
    Estaba declarado en el setup.py de ROS 1 y era una referencia colgando.

POR QUÉ EL SDK SIGUE EN scripts/ Y NO SE MUEVE

  `scripts/sphero_sdk` son 196 ficheros cuyos imports internos son ABSOLUTOS
  (`from sphero_sdk.xxx import yyy`). Moverlo bajo el paquete, que es la
  disposición habitual en ament_python, obligaría a reescribir esos imports —
  y es la única pieza del proyecto que ya está validada en Python 3.12
  (Etapa D, 🟢 GO). No se toca.

  Con `package_dir={'': 'scripts'}`, tanto `sphero_sdk` como `atriz_rvr_driver`
  quedan como paquetes de nivel superior, importables exactamente igual que
  antes.
"""
import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'atriz_rvr_driver'

setup(
    name=package_name,
    version='1.0.0',
    # find_packages sobre scripts/ recoge 'atriz_rvr_driver' (el nodo) y
    # 'sphero_sdk' con todos sus subpaquetes.
    packages=find_packages(where='scripts'),
    package_dir={'': 'scripts'},
    data_files=[
        # El marcador de resource_index es OBLIGATORIO en ament_python: sin él,
        # `ros2 run` y `ros2 pkg` no encuentran el paquete aunque se instale.
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Atriz Team - Universidad de Nariño',
    maintainer_email='atriz@udenar.edu.co',
    description='Driver ROS 2 del Sphero RVR para el laboratorio remoto Atriz',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ros2 run atriz_rvr_driver rvr_driver_node
            'rvr_driver_node = atriz_rvr_driver.rvr_driver_node:main',
            # Lo arranca nav2.launch.py: cancela los objetivos de Nav2 cuando se
            # pulsa la parada de emergencia. Va aquí y no en el bringup porque
            # este paquete es Python; el bringup es ament_cmake.
            'cancelar_nav2 = atriz_rvr_driver.cancelar_nav2:main',
            # Lo arranca robot.launch.py. Gobierna atriz-slam y atriz-nav y
            # publica /estado_navegacion. Va APARTE del driver para aislar el
            # privilegio: es el unico que llama a systemctl.
            'supervisor_navegacion = atriz_rvr_driver.supervisor_navegacion:main',
        ],
    },
)
