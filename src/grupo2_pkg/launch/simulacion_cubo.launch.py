"""Base de simulacion para P2 y P3: brazo + cubo en cinta + sensor de proximidad.

Levanta:
  1. MuJoCo con scene_nero_cubo.xml (nero_ctrl, mode:=sim). El puente publica
     /proximity, /feedback/cubo_pose, /feedback/cubo_contacto y /feedback/cubo_agarre,
     y mueve el cubo segun /control/cubo.
  2. waypoint_manager: ejecuta los comandos de /commands/waypoints.
     Una rutina opcional `routine` de grupo2_pkg los publica.
  3. Opcional: la cinta automatica (cube:=true).

La rutina y la cinta arrancan cuando can_module termina la pose inicial. Se reenvia
/start_running_nodes (ver orden_arranque) porque esa pose inicial depende de un
mensaje que can_module puede perderse por una carrera de arranque.
P2 y P3 incluyen este launch: interceptar_cubo.launch.py y recoger_cubo.launch.py.

Ejemplos:
  ros2 launch grupo2_pkg simulacion_cubo.launch.py
  ros2 launch grupo2_pkg simulacion_cubo.launch.py cube_speed:=0.04 rviz:=true
  ros2 launch grupo2_pkg simulacion_cubo.launch.py routine:=interceptar_cubo
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
    TimerAction,
)
from launch.event_handlers import OnProcessIO
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ESCENA = 'scene_nero_cubo.xml'

# Cinta automatica (marco mundo = base_link). La cinta (cinta.xml) esta centrada en
# (-0.20, -0.50), a lo largo de Y (y de -0.872 a -0.128), banda a z = 0.064.
# El cubo entra por el extremo lejano (Y_INICIO) y recorre RECORRIDO en +Y, hacia el brazo.
CUBO_X = -0.20
CUBO_Z = 0.085
Y_INICIO = -0.85
RECORRIDO = 0.70

# Ultima linea de can_module al enviar la pose inicial.
FIN_POSE_INICIAL = 'Published initial joint states (attempt 3/3)'


# Linea de can_module al construir el nodo (justo antes de suscribirse a /start_running_nodes).
CAN_MODULE_LISTO = 'CAN module initialized.'


def orden_arranque():
    """Reenvia /start_running_nodes cuando can_module ya existe.

    main_node lo publica una sola vez (latcheado, TRANSIENT_LOCAL) al segundo de
    arrancar, pero can_module se suscribe VOLATILE, asi que no recibe el latch si
    el descubrimiento DDS se completa despues de esa publicacion: se queda sin
    arrancar, no manda la pose inicial y con ello no arranca ni la rutina ni la
    cinta del cubo.

    Antes se publicaba al lanzar con `-w 1` (esperar un suscriptor), pero un
    suscriptor que aun no ha desaparecido de una ejecucion anterior bastaba para
    publicar antes de que can_module existiera. Ahora se espera a que can_module
    diga que esta construido y se publica 5 veces en 5 s (-w 1 y -t 5 -r 1).
    Reenviarlo es inofensivo: can_module ignora los siguientes
    (guarda `if msg.data and not self.start_requested`).
    """
    publicar = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '-w', '1', '-t', '5', '-r', '1', '/start_running_nodes',
             'std_msgs/msg/Bool', '{data: true}'],
        output='screen',
    )
    lanzado = []

    def handler(event):
        if lanzado or CAN_MODULE_LISTO not in event.text.decode(errors='ignore'):
            return None
        lanzado.append(True)
        return [publicar]

    return RegisterEventHandler(OnProcessIO(on_stdout=handler, on_stderr=handler))


def orden_cubo(datos):
    """Publica una orden [x, y, z, vx, vy, vz, duracion] en /control/cubo."""
    return ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once', '-w', '1', '/control/cubo',
             'std_msgs/msg/Float64MultiArray', f'{{data: {datos}}}'],
        output='screen',
    )


def nodos_tras_pose_inicial(context, *args, **kwargs):
    """Rutina y orden de la cinta, segun los argumentos."""
    def arg(nombre):
        return LaunchConfiguration(nombre).perform(context)

    acciones = [Node(
        package='grupo2_pkg',
        executable='waypoint_manager',
        output='screen',
    )]

    if arg('routine') != 'none':
        # La rutina conoce la velocidad de la cinta: solo estima donde va el cubo.
        acciones.append(Node(
            package='grupo2_pkg',
            executable=arg('routine'),
            output='screen',
            parameters=[{'v_cinta': float(arg('cube_speed'))}],
        ))

    if arg('cube') == 'true':
        v = float(arg('cube_speed'))
        retardo = float(arg('cube_delay'))
        # Cubo quieto a la entrada de la cinta hasta que arranca.
        acciones.append(orden_cubo([CUBO_X, Y_INICIO, CUBO_Z, 0.0, 0.0, 0.0, retardo + 1.0]))
        acciones.append(TimerAction(period=retardo, actions=[
            orden_cubo([CUBO_X, Y_INICIO, CUBO_Z, 0.0, v, 0.0, RECORRIDO / v])]))

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
        DeclareLaunchArgument('routine', default_value='none',
                              description='Rutina de grupo2_pkg que envia waypoints ("none" = ninguna)'),
        DeclareLaunchArgument('cube', default_value='true', choices=['true', 'false'],
                              description='Cinta automatica (false si la rutina mueve el cubo)'),
        DeclareLaunchArgument('cube_speed', default_value='0.08',
                              description='Velocidad de la cinta automatica en +Y (m/s)'),
        DeclareLaunchArgument('cube_delay', default_value='0.0',
                              description='Segundos tras la pose inicial antes de arrancar la cinta'),
        al_terminar_pose_inicial([OpaqueFunction(function=nodos_tras_pose_inicial)]),
        simulacion,
        orden_arranque(),
    ])
