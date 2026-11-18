"""Pregunta 3 (primera parte): interceptar, detener y cerrar el gripper.

Se conserva el control de brazo por /obj_point -> IK_node.py.
El cubo es un mocap: cerrar el gripper NO lo adhiere al brazo todavía.
"""

import math
import time

import mujoco
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState
from grupo2_pkg.model_paths import (
    ESCENA_TAREA,
    encontrar_escena,
    encontrar_modelo_ik,
    resolver_modelo,
)
from std_msgs.msg import Bool, Float64MultiArray, String


# ============================================================
# CONFIGURACIÓN
# ============================================================

# Orientación de link7 al interceptar.
#
# El valor original de la pregunta 2 con el cubo avanzando en X era
# [-0.144805, -0.669721, 0.707990, -0.171042]. Con él los dedos se abren a lo
# largo de Y, que ahora es la dirección de avance del cubo: el dedo delantero se
# queda en el carril y golpea el cubo medio ancho de pinza antes de llegar al
# centro (se veía como un contacto en y = -0.05 con la pinza abierta 10 cm).
#
# Este valor es ese mismo giro compuesto con +90° sobre el eje Z de la base, de
# modo que los dedos quedan a los lados del carril (apertura en X) y el cubo
# entra entre ellos. Si la IK se resiste a esta muñeca, el giro de -90° es
# [-0.575957, -0.371172, 0.621570, 0.379680] y sirve igual.
# Construida a medida (no compuesta del valor antiguo): el eje de avance de
# la pinza apunta a -Z de la base (dedos mirando abajo) y la apertura a X
# (los dedos a los lados del carril del cubo). Verificado sobre el modelo.
Q_LINK7 = np.array([-0.012296, 0.674037, 0.018427, 0.738365])

# Movimiento del cubo
# El cubo cruza por delante del robot en el eje Y (cinta transportadora).
VELOCIDAD_CUBO = 0.04
RECORRIDO_CUBO = 0.60
DISTANCIA_INICIO = 0.20

# Movimiento del brazo
ALTURA_APROXIMACION = 0.08
TOLERANCIA = 0.018
MAX_ESPERA = 12.0

# Control del gripper
APERTURA_INICIAL = 0.10
UMBRAL_ABIERTO = 0.095
APERTURA_CIERRE = 0.01

# ============================================================
# FUNCIONES GEOMÉTRICAS
# ============================================================

def matriz_desde_cuaternion(q):

    x, y, z, w = q / np.linalg.norm(q)

    return np.array([
        [
            1 - 2*(y*y + z*z),
            2*(x*y - z*w),
            2*(x*z + y*w)
        ],
        [
            2*(x*y + z*w),
            1 - 2*(x*x + z*z),
            2*(y*z - x*w)
        ],
        [
            2*(x*z - y*w),
            2*(y*z + x*w),
            1 - 2*(x*x + y*y)
        ]
    ])


def rpy_desde_matriz(R):

    """Devuelve roll, pitch y yaw en radianes."""

    pitch = math.asin(float(np.clip(-R[2, 0], -1.0, 1.0)))

    if abs(math.cos(pitch)) > 1e-7:

        roll = math.atan2(float(R[2, 1]), float(R[2, 2]))

        yaw = math.atan2(float(R[1, 0]), float(R[0, 0]))

    else:

        roll = math.atan2(float(-R[1, 2]), float(R[1, 1]))

        yaw = 0.0

    return np.array([roll, pitch, yaw])


def id_cuerpo(modelo, nombre):

    indice = mujoco.mj_name2id(modelo, mujoco.mjtObj.mjOBJ_BODY, nombre)

    if indice < 0:
        raise RuntimeError(f"No existe el cuerpo '{nombre}' en el modelo")

    return indice


# ============================================================
# GEOMETRÍA DEL CUBO Y DEL GRIPPER
# ============================================================

def leer_geometria_simulacion(escena):

    modelo = mujoco.MjModel.from_xml_path(str(escena))

    datos = mujoco.MjData(modelo)

    mujoco.mj_forward(modelo, datos)

    base = id_cuerpo(modelo, "base_link")
    link7 = id_cuerpo(modelo, "link7")
    cubo = id_cuerpo(modelo, "cubo_movil")

    R_base = datos.xmat[base].reshape(3, 3)
    R_link7 = datos.xmat[link7].reshape(3, 3)

    # Posición del cubo respecto de base_link
    cubo_base = R_base.T @ (datos.xpos[cubo] - datos.xpos[base])

    # Posición global del cubo
    cubo_world = datos.xpos[cubo].copy()

    # Calcular el centro geométrico de los dedos
    centros = []

    for nombre in ["gripper_link1", "gripper_link2"]:

        i = id_cuerpo(modelo, nombre)

        primero = int(modelo.body_geomadr[i])
        cantidad = int(modelo.body_geomnum[i])

        if cantidad == 0:
            raise RuntimeError(f"{nombre} no tiene geometrías")

        centros.append(np.mean(datos.geom_xpos[primero:primero + cantidad], axis=0))

    centro_dedos = np.mean(centros, axis=0)

    # Desplazamiento del centro de los dedos respecto de link7
    offset_dedos_link7 = R_link7.T @ (centro_dedos - datos.xpos[link7])

    return (cubo_base, cubo_world, offset_dedos_link7)


# ============================================================
# REFERENCIA UTILIZADA POR IK_NODE.PY
# ============================================================

def desplazamiento_del_site_ik(ruta_modelo, base=None):

    ruta = resolver_modelo(ruta_modelo, base)

    modelo = mujoco.MjModel.from_xml_path(str(ruta))

    datos = mujoco.MjData(modelo)

    mujoco.mj_forward(modelo, datos)

    link7 = id_cuerpo(modelo, "link7")

    site = mujoco.mj_name2id(modelo, mujoco.mjtObj.mjOBJ_SITE, "pinch_site")

    if site < 0:
        raise RuntimeError(f"No existe pinch_site en {ruta}")

    R_link7 = datos.xmat[link7].reshape(3, 3)
    R_site = datos.site_xmat[site].reshape(3, 3)

    # Posición relativa de pinch_site respecto de link7
    offset = R_link7.T @ (datos.site_xpos[site] - datos.xpos[link7])

    # Orientación relativa
    rotacion_relativa = R_link7.T @ R_site

    return offset, rotacion_relativa


# ============================================================
# NODO ROS 2
# ============================================================

class RecogerCubo(Node):

    def __init__(self):

        super().__init__("recoger_cubo")

        # ----------------------------------------------------
        # Escena MuJoCo (se localiza sin rutas absolutas)
        # ----------------------------------------------------

        self.declare_parameter("scene", ESCENA_TAREA)
        self.declare_parameter("model_dir", "")

        nombre_escena = self.get_parameter("scene").get_parameter_value().string_value

        carpeta_escena = self.get_parameter("model_dir").get_parameter_value().string_value

        self.escena = encontrar_escena(nombre_escena, carpeta_escena or None)

        self.get_logger().info(f"Escena MuJoCo: {self.escena}")

        # ----------------------------------------------------
        # Configuración QoS para IK_node.py
        # ----------------------------------------------------

        qos_ik = QoSProfile(depth=10)

        qos_ik.reliability = ReliabilityPolicy.RELIABLE

        qos_ik.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # ----------------------------------------------------
        # Publicadores
        # ----------------------------------------------------

        # Objetivo de cinemática inversa
        self.pub_objetivo = self.create_publisher(Float64MultiArray, "/obj_point", qos_ik)

        # Movimiento del cubo
        self.pub_cubo = self.create_publisher(Float64MultiArray, "/control/cubo", 10)

        # Apertura y cierre del gripper
        self.pub_gripper = self.create_publisher(JointState, "/control/move_j", 10)

        # ----------------------------------------------------
        # Suscriptores
        # ----------------------------------------------------

        # Posición del extremo del robot
        self.sub_tcp = self.create_subscription(PoseStamped, "/feedback/tcp_pose", self.on_tcp, 10)

        # Posiciones articulares y apertura del gripper
        self.sub_joints = self.create_subscription(JointState, "/feedback/joint_states", self.on_joint_states, 10)

        # Contacto entre el cubo y el robot
        self.sub_contacto = self.create_subscription(Bool, "/feedback/cubo_contacto", self.on_contacto, 10)

        # Modelo utilizado por IK_node.py. QoS TRANSIENT_LOCAL: el publicador
        # de main_node es latched, así que el modelo llega aunque este nodo
        # arranque después.
        self.sub_modelo = self.create_subscription(String, "/model_path", self.on_modelo_ik, qos_ik)

        # ----------------------------------------------------
        # Calcular posición de intercepción
        # ----------------------------------------------------

        (self.cubo_base, self.cubo_world, self.offset_dedos) = leer_geometria_simulacion(self.escena)

        self.R_link7_obj = matriz_desde_cuaternion(Q_LINK7)

        # Posición final de link7
        self.obj_link7 = (self.cubo_base - self.R_link7_obj @ self.offset_dedos + np.array([0.0, 0.0, 0.015]))

        # Primera posición de aproximación
        self.aprox_link7 = (self.obj_link7 + np.array([0.0, 0.0, ALTURA_APROXIMACION]))

        self.cierre_estable_desde = None

        # Posición de levantamiento: 10 cm hacia arriba
        self.obj_levantamiento = (self.obj_link7 + np.array([0.0, 0.0, 0.10]))

        # ----------------------------------------------------
        # Variables de control
        # ----------------------------------------------------

        self.tcp = None

        self.apertura_actual = None

        self.contacto_cubo = False

        self.orden_apertura_enviada = False

        self.orden_cierre_enviada = False

        self.offset_site = None

        self.R_site_link7 = None

        self.ruta_modelo_ik = None

        self.intento_fallback = False

        self.etapa = 0

        self.inicio_espera_modelo = time.monotonic()

        self.momento_etapa = None

        self.aviso_espera = False

        # ----------------------------------------------------
        # Temporizador principal
        # ----------------------------------------------------

        self.timer = self.create_timer(0.1, self.control)

        self.get_logger().info(f"Punto de encuentro: " f"{np.round(self.cubo_base, 4)}")

        self.get_logger().info(f"Objetivo link7: " f"{np.round(self.obj_link7, 4)}")

        self.get_logger().info("Esperando MuJoCo, IK_node.py y tópicos ROS 2...")

        self.agarre_confirmado = False

        self.create_subscription(Bool, "/feedback/cubo_agarre", self.on_agarre, 10)

    def on_agarre(self, msg):
        self.agarre_confirmado = bool(msg.data)

    # ========================================================
    # RECIBIR POSICIÓN DEL ROBOT
    # ========================================================

    def on_tcp(self, msg):

        p = msg.pose.position

        self.tcp = np.array([p.x, p.y, p.z])

    # ========================================================
    # RECIBIR APERTURA DEL GRIPPER
    # ========================================================

    def on_joint_states(self, msg):

        if "gripper" in msg.name:

            i = msg.name.index("gripper")

            if i < len(msg.position):

                self.apertura_actual = float(msg.position[i])

    # ========================================================
    # RECIBIR CONTACTO DEL CUBO
    # ========================================================

    def on_contacto(self, msg):

        previo = self.contacto_cubo

        self.contacto_cubo = bool(msg.data)

        if self.contacto_cubo and not previo:

            self.get_logger().info("Contacto detectado: cubo detenido.")

    # ========================================================
    # RECIBIR MODELO DE IK
    # ========================================================

    def on_modelo_ik(self, msg):

        if not msg.data:
            return

        if msg.data == self.ruta_modelo_ik:
            return

        try:

            (self.offset_site, self.R_site_link7) = desplazamiento_del_site_ik(msg.data, self.escena.parent)

        except Exception as error:

            self.get_logger().error(f"No se pudo cargar el modelo IK: {error}")

            return

        self.ruta_modelo_ik = msg.data

        self.get_logger().info(f"Modelo IK: {msg.data}")

    # ========================================================
    # ABRIR O CERRAR EL GRIPPER
    # ========================================================

    def mover_gripper(self, apertura):

        msg = JointState()

        msg.header.stamp = (self.get_clock().now().to_msg())

        msg.name = ["gripper"]

        msg.position = [float(apertura)]

        self.pub_gripper.publish(msg)

        self.get_logger().info(f"Apertura solicitada del gripper: " f"{100 * apertura:.1f} cm")

    # ========================================================
    # INICIAR MOVIMIENTO DEL CUBO
    # ========================================================

    def publicar_cubo(self):

        # El XML conserva X = -0.55 como punto de encuentro.
        # El cubo inicia 20 cm más adelante y avanza hacia +X.

        x, y, z = self.cubo_world

        msg = Float64MultiArray()

        msg.data = [
            float(x),
            float(y - DISTANCIA_INICIO),
            float(z),
            0.0,
            float(VELOCIDAD_CUBO),
            0.0,
            float(RECORRIDO_CUBO / VELOCIDAD_CUBO)
        ]

        self.pub_cubo.publish(msg)

        self.get_logger().info("Cubo iniciado: cruza por delante del robot hacia +Y.")

    # ========================================================
    # ENVIAR OBJETIVO A IK_NODE.PY
    # ========================================================

    def publicar_ik_para_link7(self, objetivo_link7):

        # IK_node.py controla pinch_site.
        # Convertimos el objetivo de link7 a pinch_site.

        p_site = (objetivo_link7 + self.R_link7_obj @ self.offset_site)

        R_site = (self.R_link7_obj @ self.R_site_link7)

        rpy = rpy_desde_matriz(R_site)

        msg = Float64MultiArray()

        msg.data = [float(v) for v in np.concatenate((p_site, rpy))]

        self.pub_objetivo.publish(msg)

        self.get_logger().info(f"Objetivo IK: {np.round(msg.data, 4)}")

    # ========================================================
    # COMPROBAR POSICIÓN DEL BRAZO
    # ========================================================

    def cerca_de(self, objetivo):

        if self.tcp is None:
            return False

        error = np.linalg.norm(self.tcp - objetivo)

        return error < TOLERANCIA

    # ========================================================
    # CONTROL AUTOMÁTICO
    # ========================================================

    def control(self):

        # ----------------------------------------------------
        # Buscar modelo alternativo si no llega /model_path
        # ----------------------------------------------------

        if (self.etapa == 0 and self.offset_site is None):

            if (time.monotonic() - self.inicio_espera_modelo > 3.0 and not self.intento_fallback):

                self.intento_fallback = True

                try:
                    candidato = encontrar_modelo_ik()

                except FileNotFoundError as error:

                    self.get_logger().error(f"Modelo IK no disponible. {error}")

                else:

                    self.get_logger().warn("No se recibió /model_path; usando modelo IK local.")

                    self.on_modelo_ik(String(data=str(candidato)))

        # ====================================================
        # ETAPA 0: ABRIR EL GRIPPER
        # ====================================================

        if self.etapa == 0:

            if (
                self.offset_site is None
                or self.tcp is None
                or self.pub_objetivo.get_subscription_count() == 0
                or self.pub_cubo.get_subscription_count() == 0
                or self.pub_gripper.get_subscription_count() == 0
            ):
                return

            # Abrir la garra antes de comenzar
            if not self.orden_apertura_enviada:

                self.mover_gripper(APERTURA_INICIAL)

                self.orden_apertura_enviada = True
                

                self.get_logger().info("Esperando que el gripper alcance 9.5 cm de apertura...")

                return

            # Esperar a que esté completamente abierta
            if (self.apertura_actual is None or self.apertura_actual < UMBRAL_ABIERTO):
                return

            # Iniciar el movimiento del cubo y del brazo
            self.publicar_cubo()

            self.publicar_ik_para_link7(self.aprox_link7)

            self.momento_etapa = time.monotonic()

            self.etapa = 1

            self.get_logger().info("FASE 1: Cubo moviéndose y brazo aproximándose mediante IK_node.py.")

            return

        # ====================================================
        # ETAPA 1: APROXIMACIÓN FINAL
        # ====================================================

        if self.etapa == 1:

            tiempo = (time.monotonic() - self.momento_etapa)

            llego = self.cerca_de(self.aprox_link7)

            if llego or tiempo > MAX_ESPERA:

                if not llego:

                    self.get_logger().warn("Aproximación sin tolerancia; se intenta movimiento final.")

                self.publicar_ik_para_link7(self.obj_link7)

                self.momento_etapa = time.monotonic()

                self.etapa = 2

                self.get_logger().info("FASE 2: Aproximación final; el cubo continúa moviéndose.")

            return

        # ====================================================
        # ETAPA 2: ESPERAR CONTACTO Y CERRAR
        # ====================================================

        if self.etapa == 2:

            if (self.contacto_cubo and not self.orden_cierre_enviada):

                # Cerrar cuando MuJoCo detecte contacto
                self.mover_gripper(APERTURA_CIERRE)

                self.orden_cierre_enviada = True

                self.momento_etapa = time.monotonic()

                self.etapa = 3

                self.get_logger().info("FASE 3: Cierre automático del gripper tras contacto.")

            elif (not self.aviso_espera and time.monotonic() - self.momento_etapa > MAX_ESPERA):

                self.aviso_espera = True

                self.get_logger().warn("Aún no se detecta contacto. No se cerrará la garra a ciegas.")

            return

        # ====================================================
        # ETAPA 3: ESPERAR AGARRE ESTABLE
        # ====================================================

        if self.etapa == 3:

            # Comprobar contacto simultáneo con ambos dedos
            if self.agarre_confirmado:

                if self.cierre_estable_desde is None:
                    self.cierre_estable_desde = time.monotonic()

                tiempo_estable = (time.monotonic() - self.cierre_estable_desde)

                # Exigir contacto durante 0.7 segundos
                if (tiempo_estable >= 0.7 and self.apertura_actual is not None and self.apertura_actual <= 0.045):

                    self.get_logger().info("Contacto estable con ambos dedos.")

                    self.get_logger().info("Iniciando levantamiento...")

                    self.publicar_ik_para_link7(self.obj_levantamiento)

                    self.momento_etapa = time.monotonic()

                    self.etapa = 4

            else:

                # Si se pierde contacto, reiniciar comprobación
                self.cierre_estable_desde = None

            return


        # ====================================================
        # ETAPA 4: LEVANTAMIENTO DEL CUBO
        # ====================================================

        if self.etapa == 4:

            tiempo = time.monotonic() - self.momento_etapa

            # Comprobar si el brazo alcanzó la altura objetivo
            llego = self.cerca_de(self.obj_levantamiento)

            if llego:

                self.get_logger().info(
                    "FASE 4: Brazo elevado 10 cm. "
                    "Comprobar visualmente que el cubo "
                    "permanezca sujeto."
                )

                self.etapa = 5

            elif tiempo > MAX_ESPERA:

                self.get_logger().warn(
                    "El brazo no alcanzó la posición "
                    "de levantamiento dentro del tiempo esperado."
                )

                self.etapa = 6

            return


        # ====================================================
        # ETAPA 5: MANTENER LA POSICIÓN
        # ====================================================

        if self.etapa == 5:

            # Mantener la posición alcanzada.
            # No enviar nuevos comandos.
            return


        # ====================================================
        # ETAPA 6: FINALIZACIÓN SIN LEVANTAMIENTO CONFIRMADO
        # ====================================================

        if self.etapa == 6:
            return


# ============================================================
# FUNCIÓN PRINCIPAL
# ============================================================

def main():

    rclpy.init()

    nodo = None

    try:

        nodo = RecogerCubo()

        rclpy.spin(nodo)

    except KeyboardInterrupt:
        pass

    finally:

        if nodo is not None:
            nodo.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()