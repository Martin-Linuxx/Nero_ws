"""Pregunta 2: intercepcion del cubo movil, todo en un solo lanzamiento.

Levanta las tres piezas que antes se arrancaban a mano en tres terminales:

  1. La simulacion (nero_ctrl/nero_can_ctrl.launch.py en modo sim) con la
     escena que incluye el cubo movil.
  2. La cinematica inversa del grupo (grupo2_pkg/ik_node), que publica solo las
     7 articulaciones del brazo para no pisar el gripper.
  3. La rutina de intercepcion (grupo2_pkg/interceptar_cubo).

Ejemplos:
  ros2 launch grupo2_pkg interceptar_cubo.launch.py
  ros2 launch grupo2_pkg interceptar_cubo.launch.py rviz:=true
  ros2 launch grupo2_pkg interceptar_cubo.launch.py alpha:=0.8 start_delay:=8.0
  ros2 launch grupo2_pkg interceptar_cubo.launch.py scene:=scene_nero_arm.xml   # sin cubo

Los nodos esperan por si solos a que la simulacion publique /model_path, asi que
start_delay solo sirve para que los registros salgan ordenados.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ESCENA_CUBO = 'scene_nero_cubo.xml'


def generate_launch_description():
    scene = LaunchConfiguration('scene')
    rviz = LaunchConfiguration('rviz')
    alpha = LaunchConfiguration('alpha')
    model_dir = LaunchConfiguration('model_dir')
    start_delay = LaunchConfiguration('start_delay')

    simulacion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nero_ctrl'),
                'launch',
                'nero_can_ctrl.launch.py',
            )
        ),
        launch_arguments={
            'mode': 'sim',
            'scene': scene,
            'rviz': rviz,
        }.items(),
    )

    ik = Node(
        package='grupo2_pkg',
        executable='ik_node',
        name='inverse_kinematics_node',
        output='screen',
        parameters=[{'alpha': alpha}],
    )

    rutina = Node(
        package='grupo2_pkg',
        executable='interceptar_cubo',
        name='interceptar_cubo',
        output='screen',
        parameters=[{
            'scene': scene,
            'model_dir': model_dir,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'scene',
            default_value=ESCENA_CUBO,
            description='Escena MuJoCo. La de la tarea es la unica con cubo movil',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            choices=['true', 'false'],
            description='Lanzar RViz2 junto a MuJoCo',
        ),
        DeclareLaunchArgument(
            'alpha',
            default_value='0.5',
            description='Paso del metodo de Newton-Raphson de la cinematica inversa',
        ),
        DeclareLaunchArgument(
            'model_dir',
            default_value='',
            description='Carpeta con los XML de MuJoCo (opcional; se localiza sola)',
        ),
        DeclareLaunchArgument(
            'start_delay',
            default_value='6.0',
            description='Segundos de espera antes de arrancar la IK y la rutina',
        ),
        simulacion,
        TimerAction(period=start_delay, actions=[ik, rutina]),
    ])
