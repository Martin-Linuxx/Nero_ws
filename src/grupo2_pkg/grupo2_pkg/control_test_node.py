#!/usr/bin/env python3
#? ^ Le dice a bash "ejecuta esto con python3"  ^

"""Nodo mínimo para probar cómo se arma una rutina de control.

No hay cinemática ni planificación: solo una lista hardcodeada de setpoints que
se publican a los tópicos del controlador (los mismos de los ejemplos del
README) y se avanza al siguiente punto cuando el feedback confirma la llegada.

Tópicos usados (los que levanta `ros2 launch nero_ctrl nero_can_ctrl.launch.py
mode:=sim rviz:=false`):

  publica  /control/move_p        geometry_msgs/PoseStamped  -> pose objetivo
  publica  /control/joint_states  sensor_msgs/JointState     -> apertura gripper
  escucha  /feedback/tcp_pose     geometry_msgs/PoseStamped  -> pose alcanzada
  escucha  /feedback/joint_states sensor_msgs/JointState     -> apertura real
"""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState


# ============================================================
# RUTINA HARDCODEADA
# ============================================================
#
# Cada punto es una pose de la brida en el marco base_link: la misma convención
# que src/agx_arm_ros/test/nero/test_move_p.yaml, así que los valores de ese
# archivo sirven tal cual.
#
#   pos     : (x, y, z) en metros
#   ori     : cuaternión (x, y, z, w)
#   gripper : None = no tocar | "abrir" | "cerrar"
#
# Las tres orientaciones son la del ejemplo del README girada 0°, +45° y -45°
# sobre el eje Z de la base: la muñeca queda visiblemente distinta en cada punto.

RUTINA = [
    {
        "pos": (-0.17, -0.0235, 0.4156),
        "ori": (0.010348, 0.707043, -0.707019, 0.010348),
        "gripper": "abrir",
    },
    {
        "pos": (-0.15, -0.0235, 0.3656),
        "ori": (-0.261013, 0.657183, -0.649240, 0.280125),
        "gripper": "cerrar",
    },
    {
        "pos": (-0.15, -0.0700, 0.4000),
        "ori": (0.280134, 0.649263, -0.657160, -0.261004),
        "gripper": None,
    },
]

# Aperturas del gripper (ancho total en metros, como en el brazo real)
APERTURA_ABIERTO = 0.08
APERTURA_CERRADO = 0.01

# Fases de la máquina de estados
ESPERA_FEEDBACK = "espera_feedback"
ENVIAR = "enviar"
MOVIENDO = "moviendo"
GRIPPER = "gripper"
FIN = "fin"


class ControlTestNode(Node):
    def __init__(self):
        super().__init__("control_test_node")

        #? --- Atributos ---

        self.idx = 0                 # waypoint actual
        self.fase = ESPERA_FEEDBACK  # fase dentro del waypoint
        self.t0 = None               # inicio de la fase (para el timeout)

        self.tcp = None              # última pose recibida (x, y, z)
        self.apertura = None         # última apertura del gripper recibida

        self.objetivo_pos = None     # setpoint cartesiano enviado
        self.objetivo_gripper = None # setpoint de apertura enviado

        self.setup_parameters()
        self.init_communications()

        self.get_logger().info(f"Rutina cargada con {len(RUTINA)} puntos. Esperando feedback del controlador...")

    def setup_parameters(self):  #? Inicializar parámetros de ejecución
        self.declare_parameter("tol_pos", 0.01)       # m
        self.declare_parameter("tol_gripper", 0.005)  # m
        self.declare_parameter("timeout", 15.0)       # s

        self.tol_pos = self.get_parameter("tol_pos").value
        self.tol_gripper = self.get_parameter("tol_gripper").value
        self.timeout = self.get_parameter("timeout").value

    def init_communications(self):  #? Crear Comunicaciones (Tópicos/Servicios)
        # Publicadores
        self.pub_pose = self.create_publisher(PoseStamped, "/control/move_p", 10)
        self.pub_gripper = self.create_publisher(JointState, "/control/joint_states", 10)

        # Suscriptores
        self.create_subscription(PoseStamped, "/feedback/tcp_pose", self.tcp_cb, 10)
        self.create_subscription(JointState, "/feedback/joint_states", self.joint_states_cb, 10)

        # Motor de la rutina
        self.timer = self.create_timer(0.1, self.rutina_cb)

    # ========================================================
    # CALLBACKS DE FEEDBACK
    # ========================================================

    def tcp_cb(self, msg):          #? Pose alcanzada por la brida
        p = msg.pose.position
        self.tcp = (p.x, p.y, p.z)

    def joint_states_cb(self, msg):  #? Apertura real del gripper
        if "gripper" in msg.name:
            i = msg.name.index("gripper")
            if i < len(msg.position):
                self.apertura = float(msg.position[i])

    # ========================================================
    # ENVÍO DE SETPOINTS
    # ========================================================

    def enviar_pose(self, punto):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = punto["pos"]

        (msg.pose.orientation.x,
         msg.pose.orientation.y,
         msg.pose.orientation.z,
         msg.pose.orientation.w) = punto["ori"]

        self.pub_pose.publish(msg)

        self.objetivo_pos = punto["pos"]

    def enviar_gripper(self, apertura):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["gripper"]
        msg.position = [float(apertura)]

        self.pub_gripper.publish(msg)

        self.objetivo_gripper = float(apertura)

    # ========================================================
    # RUTINA
    # ========================================================

    def cambiar_fase(self, fase):
        self.fase = fase
        self.t0 = time.monotonic()

    def expirado(self):
        return self.t0 is not None and time.monotonic() - self.t0 > self.timeout

    def abortar(self, motivo):
        self.get_logger().warn(motivo)
        self.get_logger().warn("Rutina abortada.")
        self.fase = FIN

    def rutina_cb(self):
        # ----------------------------------------------------
        # Esperar a que el controlador esté escuchando
        # ----------------------------------------------------
        if self.fase == ESPERA_FEEDBACK:
            listo = (
                self.tcp is not None
                and self.pub_pose.get_subscription_count() > 0
                and self.pub_gripper.get_subscription_count() > 0
            )

            if listo:
                self.get_logger().info("Controlador detectado. Comenzando rutina.")
                self.cambiar_fase(ENVIAR)

            return

        # ----------------------------------------------------
        # Publicar el waypoint actual
        # ----------------------------------------------------
        if self.fase == ENVIAR:
            punto = RUTINA[self.idx]

            self.enviar_pose(punto)

            self.get_logger().info(f"Waypoint {self.idx + 1}/{len(RUTINA)} enviado: pos={punto['pos']} ori={punto['ori']}")

            self.cambiar_fase(MOVIENDO)

            return

        # ----------------------------------------------------
        # Esperar la llegada (única comparación del nodo)
        # ----------------------------------------------------
        if self.fase == MOVIENDO:
            error = math.dist(self.tcp, self.objetivo_pos)

            if error < self.tol_pos:
                self.get_logger().info(f"Waypoint {self.idx + 1} alcanzado (error {1000 * error:.1f} mm).")

                accion = RUTINA[self.idx]["gripper"]

                if accion is None:
                    self.siguiente_waypoint()
                else:
                    apertura = APERTURA_ABIERTO if accion == "abrir" else APERTURA_CERRADO

                    self.enviar_gripper(apertura)

                    self.get_logger().info(f"Orden de gripper: {accion} ({100 * apertura:.1f} cm).")

                    self.cambiar_fase(GRIPPER)

            elif self.expirado():
                self.abortar(
                    f"Waypoint {self.idx + 1} no alcanzado en {self.timeout:.0f} s "
                    f"(error {1000 * error:.1f} mm). Revisa si el controlador rechazó la IK."
                )

            return

        # ----------------------------------------------------
        # Esperar que el gripper llegue a la apertura pedida
        # ----------------------------------------------------
        if self.fase == GRIPPER:
            if self.apertura is None:
                return

            error = abs(self.apertura - self.objetivo_gripper)

            if error < self.tol_gripper:
                self.get_logger().info(f"Gripper en {100 * self.apertura:.1f} cm.")

                self.siguiente_waypoint()

            elif self.expirado():
                self.abortar(
                    f"El gripper no alcanzó {100 * self.objetivo_gripper:.1f} cm "
                    f"en {self.timeout:.0f} s (está en {100 * self.apertura:.1f} cm)."
                )

            return

        # ----------------------------------------------------
        # Rutina terminada: no se publica nada más
        # ----------------------------------------------------
        if self.fase == FIN:
            return

    def siguiente_waypoint(self):
        self.idx += 1

        if self.idx >= len(RUTINA):
            self.get_logger().info("Rutina completada.")
            self.fase = FIN
        else:
            self.cambiar_fase(ENVIAR)


def main(args=None):
    rclpy.init(args=args)              #? Inicializa ROS
    node = ControlTestNode()           #? Instancia de un nodo de clase ControlTestNode

    try:
        rclpy.spin(node)               #? Permite ejecución continua de un nodo
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()           #? Cierra el nodo al terminar la ejecución


if __name__ == "__main__":
    main()
