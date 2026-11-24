"""P3: intercepcion, agarre y levantamiento del cubo.

simulacion_cubo.launch.py (cinta automatica) + la rutina recoger_cubo, que sigue
el cubo por la cinta (estimado con /proximity), alinea la mano, baja siguiendolo,
lo agarra y lo levanta antes del final de la cinta (brazo por /control/servo_p).

Ejemplos:
  ros2 launch grupo2_pkg recoger_cubo.launch.py
  ros2 launch grupo2_pkg recoger_cubo.launch.py cube_speed:=0.12 rviz:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('grupo2_pkg'), 'launch', 'simulacion_cubo.launch.py')),
        launch_arguments={
            'routine': 'recoger_cubo',
            'cube_speed': LaunchConfiguration('cube_speed'),
            'cube_delay': LaunchConfiguration('cube_delay'),
            'rviz': LaunchConfiguration('rviz'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('cube_speed', default_value='0.08',
                              description='Velocidad de la cinta (m/s)'),
        DeclareLaunchArgument('cube_delay', default_value='3.0',
                              description='Segundos tras la pose inicial antes de arrancar la cinta'),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false'],
                              description='Lanzar RViz2 junto a MuJoCo'),
        base,
    ])
