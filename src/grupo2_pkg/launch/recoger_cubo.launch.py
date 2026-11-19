"""P3: intercepcion, agarre y levantamiento del cubo.

simulacion_cubo.launch.py + la rutina recoger_cubo, que mueve el cubo ella misma
por /control/cubo (sin cinta automatica) y controla el brazo via ik_node.

Ejemplos:
  ros2 launch grupo2_pkg recoger_cubo.launch.py
  ros2 launch grupo2_pkg recoger_cubo.launch.py rviz:=true alpha:=0.8
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
            'ik': 'true',
            'cube': 'false',
            'rviz': LaunchConfiguration('rviz'),
            'alpha': LaunchConfiguration('alpha'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false'],
                              description='Lanzar RViz2 junto a MuJoCo'),
        DeclareLaunchArgument('alpha', default_value='0.5',
                              description='Paso de Newton-Raphson de ik_node'),
        base,
    ])
