"""Base de simulacion para P2 y P3: brazo + cubo en cinta + sensor de proximidad.

Levanta:
  1. MuJoCo con scene_nero_cubo.xml (nero_ctrl, mode:=sim). El puente publica
     /proximity, /feedback/cubo_pose, /feedback/cubo_contacto y /feedback/cubo_agarre,
     y mueve el cubo segun /control/cubo.
  2. La rutina `routine` de grupo2_pkg (por defecto control_test_node).
  3. Opcional: ik_node (ik:=true) y la cinta automatica (cube:=true).

La rutina, ik_node y la cinta arrancan cuando can_module termina la pose inicial.
P2 y P3 incluyen este launch: interceptar_cubo.launch.py y recoger_cubo.launch.py.

Ejemplos:
  ros2 launch grupo2_pkg simulacion_cubo.launch.py
  ros2 launch grupo2_pkg simulacion_cubo.launch.py cube_speed:=0.04 rviz:=true
  ros2 launch grupo2_pkg simulacion_cubo.launch.py routine:=none
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessIO
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ESCENA = 'scene_nero_cubo.xml'

# Cinta automatica (marco mundo): el cubo entra en Y_INICIO y recorre RECORRIDO en +Y.
CUBO_X = -0.55
CUBO_Z = 0.12
Y_INICIO = -0.80
RECORRIDO = 1.60

# Ultima linea de can_module al enviar la pose inicial.
FIN_POSE_INICIAL = 'Published initial joint states (attempt 3/3)'


def nodos_tras_pose_inicial(context, *args, **kwargs):
    """Rutina, ik_node y orden de la cinta, segun los argumentos."""
    def arg(nombre):
        return LaunchConfiguration(nombre).perform(context)

    acciones = []

    if arg('ik') == 'true':
        acciones.append(Node(
            package='grupo2_pkg',
            executable='ik_node',
            name='inverse_kinematics_node',
            output='screen',
            parameters=[{'alpha': float(arg('alpha'))}],
        ))

    if arg('routine') != 'none':
        acciones.append(Node(
            package='grupo2_pkg',
            executable=arg('routine'),
            output='screen',
            parameters=[{'scene': ESCENA}],
        ))

    if arg('cube') == 'true':
        v = float(arg('cube_speed'))
        datos = [CUBO_X, Y_INICIO, CUBO_Z, 0.0, v, 0.0, RECORRIDO / v]
        acciones.append(ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '--once', '-w', '1', '/control/cubo',
                 'std_msgs/msg/Float64MultiArray', f'{{data: {datos}}}'],
            output='screen',
        ))

    return acciones


def al_terminar_pose_inicial(acciones):
    """Lanza `acciones` una sola vez, cuando can_module termina la pose inicial."""
    lanzado = []

    def handler(event):
        if lanzado or FIN_POSE_INICIAL not in event.text.decode(errors='ignore'):
            return None
        lanzado.append(True)
        return acciones

    return RegisterEventHandler(OnProcessIO(on_stdout=handler, on_stderr=handler))


def generate_launch_description():
    simulacion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nero_ctrl'), 'launch', 'nero_can_ctrl.launch.py')),
        launch_arguments={
            'mode': 'sim',
            'scene': ESCENA,
            'rviz': LaunchConfiguration('rviz'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false'],
                              description='Lanzar RViz2 junto a MuJoCo'),
        DeclareLaunchArgument('routine', default_value='control_test_node',
                              description='Ejecutable de grupo2_pkg a lanzar ("none" = ninguno)'),
        DeclareLaunchArgument('ik', default_value='false', choices=['true', 'false'],
                              description='Lanzar ik_node (rutinas que usan /obj_point)'),
        DeclareLaunchArgument('alpha', default_value='0.5',
                              description='Paso de Newton-Raphson de ik_node'),
        DeclareLaunchArgument('cube', default_value='true', choices=['true', 'false'],
                              description='Cinta automatica (false si la rutina mueve el cubo)'),
        DeclareLaunchArgument('cube_speed', default_value='0.08',
                              description='Velocidad de la cinta automatica en +Y (m/s)'),
        al_terminar_pose_inicial([OpaqueFunction(function=nodos_tras_pose_inicial)]),
        simulacion,
    ])
