"""
sim_cubo.py — Cinta transportadora del cubo para el puente MuJoCo
==================================================================
Mueve el cubo "cubo_movil" (joint libre "cubo_free") e informa su estado.

  escucha  /control/cubo            Float64MultiArray  [x, y, z, vx, vy, vz, duracion]
  publica  /feedback/cubo_pose      PoseStamped        pose del cubo en base_link
  publica  /feedback/cubo_contacto  Bool               el cubo toca el robot
  publica  /feedback/cubo_agarre    Bool               ambos dedos tocan el cubo

/control/cubo coloca el cubo en (x, y, z) (marco mundo) y lo mueve a velocidad
constante (vx, vy, vz) durante "duracion" segundos de simulacion. Con velocidad 0
lo mantiene quieto en ese punto. Al terminar, o al tocar el robot, la cinta lo
suelta y el cubo queda en manos de la fisica (la pinza puede agarrarlo).
"""

import numpy as np

import mujoco
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float64MultiArray

CUBE_BODY = 'cubo_movil'
CUBE_JOINT = 'cubo_free'
BASE_BODY = 'base_link'
FINGER_BODIES = ('gripper_link1', 'gripper_link2')


class SimCubo:
    def __init__(self, node, model, data, lock, rate_hz=100.0):
        self.node = node
        self.model = model
        self.data = data
        self.lock = lock
        self.traj = None   # (p0, v, t0, t_fin) en tiempo de simulacion

        self.body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, CUBE_BODY)
        joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, CUBE_JOINT)
        if self.body < 0 or joint < 0:
            node.get_logger().info('Cinta del cubo: sin cubo en la escena, desactivada.')
            return

        self.qadr = int(model.jnt_qposadr[joint])
        self.vadr = int(model.jnt_dofadr[joint])

        # Cuerpos del robot: base_link y todos sus descendientes.
        base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, BASE_BODY)
        self.robot_bodies = {b for b in range(model.nbody) if self._desciende(b, base)}
        self.fingers = {mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, n)
                        for n in FINGER_BODIES}

        mujoco.mj_forward(model, data)
        self.base_p = data.xpos[base].copy()
        self.base_R = data.xmat[base].reshape(3, 3).copy()

        self.contacto = False
        self.agarre = False

        node.create_subscription(Float64MultiArray, '/control/cubo', self._on_cmd, 10)
        self.pub_pose = node.create_publisher(PoseStamped, '/feedback/cubo_pose', 10)
        self.pub_contacto = node.create_publisher(Bool, '/feedback/cubo_contacto', 10)
        self.pub_agarre = node.create_publisher(Bool, '/feedback/cubo_agarre', 10)
        node.create_timer(1.0 / rate_hz, self._publish)
        node.get_logger().info('Cinta del cubo: escuchando /control/cubo.')

    def _desciende(self, b, ancestro):
        while b > 0:
            if b == ancestro:
                return True
            b = int(self.model.body_parentid[b])
        return False

    def _on_cmd(self, msg):
        if len(msg.data) != 7:
            self.node.get_logger().warn('/control/cubo espera [x, y, z, vx, vy, vz, duracion].')
            return
        p0 = np.array(msg.data[0:3])
        v = np.array(msg.data[3:6])
        with self.lock:
            t = self.data.time
            self.traj = (p0, v, t, t + float(msg.data[6]))
            self.contacto = False
            self.agarre = False
        self.node.get_logger().info(
            f'Cinta del cubo: desde {np.round(p0, 3)} a {np.round(v, 3)} m/s '
            f'durante {msg.data[6]:.1f} s.')

    def step(self):
        """Llamar antes de mj_step, con el lock tomado."""
        if self.traj is None:
            return
        p0, v, t0, t_fin = self.traj
        q = self.data.qpos
        qd = self.data.qvel
        if self.data.time >= t_fin:
            self.traj = None
            qd[self.vadr:self.vadr + 6] = 0.0
            return
        q[self.qadr:self.qadr + 3] = p0 + v * (self.data.time - t0)
        q[self.qadr + 3:self.qadr + 7] = (1.0, 0.0, 0.0, 0.0)
        qd[self.vadr:self.vadr + 3] = v
        qd[self.vadr + 3:self.vadr + 6] = 0.0

    def _leer_contactos(self):
        robot = False
        dedos = set()
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            b1 = int(self.model.geom_bodyid[c.geom1])
            b2 = int(self.model.geom_bodyid[c.geom2])
            if self.body not in (b1, b2):
                continue
            otro = b2 if b1 == self.body else b1
            if otro in self.robot_bodies:
                robot = True
            if otro in self.fingers:
                dedos.add(otro)
        return robot, dedos == self.fingers

    def _publish(self):
        with self.lock:
            robot, agarre = self._leer_contactos()
            if robot and not self.contacto:
                # Al tocar el robot, la cinta suelta el cubo y lo detiene.
                self.traj = None
                self.data.qvel[self.vadr:self.vadr + 6] = 0.0
                self.node.get_logger().info('Cinta del cubo: contacto con el robot, cubo detenido.')
            self.contacto = self.contacto or robot
            self.agarre = agarre
            p_w = self.data.xpos[self.body].copy()
            R_w = self.data.xmat[self.body].reshape(3, 3).copy()

        p_b = self.base_R.T @ (p_w - self.base_p)
        q = np.zeros(4)
        mujoco.mju_mat2Quat(q, (self.base_R.T @ R_w).flatten())

        msg = PoseStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = BASE_BODY
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, p_b)
        msg.pose.orientation.w, msg.pose.orientation.x = float(q[0]), float(q[1])
        msg.pose.orientation.y, msg.pose.orientation.z = float(q[2]), float(q[3])
        self.pub_pose.publish(msg)
        self.pub_contacto.publish(Bool(data=self.contacto))
        self.pub_agarre.publish(Bool(data=self.agarre))
