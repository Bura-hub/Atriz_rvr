## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['sphero_sdk', 'sphero_sim'],
    package_dir={'': 'scripts'},
    scripts=[
        'scripts/Atriz_rvr_node.py',
        'scripts/emergency_stop.py',
        'scripts/rvr-ros-restarter.py',
        'scripts/cmd_vel_rviz.py',
        'scripts/rvr_tools.py',
        'scripts/degrees_control_example.py',
        'scripts/example_degrees_control.py',
        'scripts/setup_python_path.py'
    ],
)

setup(**setup_args)
