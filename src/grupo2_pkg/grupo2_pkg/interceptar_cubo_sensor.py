"""Pregunta 2: cubo móvil e intercepción con ROS 2 + IK_node.py.

Iniciar MuJoCo (nero), iniciar IK_node.py y ejecutar este archivo.
El nodo NO controla /control/move_p: publica objetivos en /obj_point.
"""

from sensor_msgs.msg import JointState

import math
import time
from collections import deque

import mujoco
import numpy as np
import rclpy

from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy
)
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
Q_LINK7 = np.array([-0.371172, 0.575957, -0.379680, 0.621570])

# El cubo cruza por delante del robot en el eje Y, a X y Z constantes,
# simulando una cinta transportadora. El punto de encuentro es la posición del
# cubo en la escena; el recorrido empieza DISTANCIA_INICIO metros antes en -Y.
VELOCIDAD_CUBO = 0.08       # m/s
DISTANCIA_INICIO = 0.80     # m antes del punto de encuentro
RECORRIDO_CUBO = 1.60       # m: si no se intercepta, sigue de largo

# Control del brazo
ALTURA_APROXIMACION = 0.08  # m
TOLERANCIA = 0.018          # m
MAX_ESPERA = 12.0           # segundos reales

# Disparo de la bajada: se ordena cuando al cubo le queda este tiempo de viaje
# hasta el punto de encuentro. Ajustable; el nodo registra en el log cuánto
# tardó de verdad la bajada ("bajada real") para afinarla.
TIEMPO_BAJADA = 1.0         # s

# Ventana para estimar la velocidad del cubo con /feedback/cubo_pose. Entre
# muestras consecutivas (200 Hz) la estimación es ruidosa; promediada sobre esta
# ventana es estable, y además no le afecta que la simulación no corra
# exactamente a tiempo real.
VENTANA_VELOCIDAD = 0.5     # s

# Margen para dar el cubo por pasado de largo
MARGEN_PASADO = 0.05        # m

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
        raise RuntimeError(f"No existe el cuerpo '{nombre}'")

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

    centros_dedos = []

    for nombre in ["gripper_link1", "gripper_link2"]:

        i = id_cuerpo(modelo, nombre)

        primero = int(modelo.body_geomadr[i])
        cantidad = int(modelo.body_geomnum[i])

        if cantidad == 0:
            raise RuntimeError(f"{nombre} no tiene geometrías")

        centros_dedos.append(np.mean(datos.geom_xpos[primero:primero + cantidad], axis=0))

    # Centro aproximado entre los dedos
    centro_agarre = np.mean(centros_dedos, axis=0)

    # Desplazamiento respecto de link7
    offset_dedos = R_link7.T @ (centro_agarre - datos.xpos[link7])

    return cubo_base, cubo_world, offset_dedos


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
        raise RuntimeError("El modelo IK no contiene pinch_site")

    R_link7 = datos.xmat[link7].reshape(3, 3)

    R_site = datos.site_xmat[site].reshape(3, 3)

    # Posición relativa de pinch_site respecto de link7
    offset_site = R_link7.T @ (datos.site_xpos[site] - datos.xpos[link7])

    # Orientación relativa
    R_site_link7 = R_link7.T @ R_site

    return offset_site, R_site_link7


# ============================================================
# NODO ROS 2
# ============================================================

class InterceptarCubo(Node):

    def __init__(self):

        super().__init__("interceptar_cubo")

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
        # QoS compatible con IK_node.py
        # ----------------------------------------------------

        qos_ik = QoSProfile(depth=10)

        qos_ik.reliability = ReliabilityPolicy.RELIABLE

        qos_ik.durability = DurabilityPolicy.TRANSIENT_LOCAL


        # Comprobar que el gripper esté abierto antes de comenzar
        self.apertura_gripper_actual = None
        self.orden_apertura_enviada = False

        self.create_subscription(JointState, "/feedback/joint_states", self.on_joint_states, 10)

        # ----------------------------------------------------
        # Publicador: objetivo para cinemática inversa
        # ----------------------------------------------------

        self.pub_objetivo = self.create_publisher(Float64MultiArray, "/obj_point", qos_ik)

        # ----------------------------------------------------
        # Publicador: movimiento del cubo
        # ----------------------------------------------------

        self.pub_cubo = self.create_publisher(Float64MultiArray, "/control/cubo", 10)

        # Publicador para controlar el gripper
        self.pub_gripper = self.create_publisher(JointState, "/control/move_j", 10)

        # ----------------------------------------------------
        # Suscriptor: posición del extremo del robot
        # ----------------------------------------------------

        self.create_subscription(PoseStamped, "/feedback/tcp_pose", self.on_tcp, 10)

        # ----------------------------------------------------
        # Suscriptor: posición del cubo en tiempo real
        # ----------------------------------------------------

        self.create_subscription(PoseStamped, "/feedback/cubo_pose", self.on_cubo, 10)

        # Aviso del puente cuando el cubo toca el robot
        self.create_subscription(Bool, "/feedback/cubo_contacto", self.on_contacto, 10)

        # ----------------------------------------------------
        # Suscriptores: modelo utilizado por IK_node.py
        # ----------------------------------------------------

        # QoS TRANSIENT_LOCAL: el publicador de main_node es latched, así que
        # el modelo llega aunque este nodo arranque después.
        self.create_subscription(String, "/model_path", self.on_modelo_ik, qos_ik)

        # ----------------------------------------------------
        # Calcular posición de intercepción
        # ----------------------------------------------------

        (self.cubo_base, self.cubo_world, self.offset_dedos) = leer_geometria_simulacion(self.escena)

        self.R_link7_obj = matriz_desde_cuaternion(Q_LINK7)

        # Objetivo de link7 para alinear los dedos
        self.obj_link7 = (self.cubo_base - self.R_link7_obj @ self.offset_dedos + np.array([0.0, 0.0, 0.015]))

        # Posición de aproximación
        self.aprox_link7 = (self.obj_link7 + np.array([0.0, 0.0, ALTURA_APROXIMACION]))

        # ----------------------------------------------------
        # Variables de estado
        # ----------------------------------------------------

        self.tcp = None

        # Seguimiento del cubo en tiempo real
        self.cubo_y = None          # última posición medida (eje de avance)
        self.cubo_vel = None        # velocidad estimada, m/s
        self.historial_cubo = deque()   # (instante, y) dentro de la ventana
        self.disparo_anunciado = False
        self.momento_disparo = None
        self.contacto = False

        self.ruta_modelo_ik = None
        self.offset_site = None
        self.R_site_link7 = None

        self.etapa = 0
        self.momento_etapa = None

        self.inicio_espera_modelo = time.monotonic()

        self.aviso_modelo = False
        self.intento_fallback = False

        self.timer = self.create_timer(0.1, self.control)

        self.get_logger().info(f"Cubo en base_link: {np.round(self.cubo_base, 4)}")

        self.get_logger().info(f"Objetivo link7: {np.round(self.obj_link7, 4)}")

        self.get_logger().info("Esperando comunicación con IK_node.py...")

    # ========================================================
    # RECIBIR POSICIÓN ACTUAL DEL ROBOT
    # ========================================================

    def on_tcp(self, msg):

        p = msg.pose.position

        self.tcp = np.array([p.x, p.y, p.z])

    # ========================================================
    # RECIBIR MODELO DE IK
    # ========================================================

    def on_modelo_ik(self, msg):
        if not msg.data:
            return

        if self.ruta_modelo_ik == msg.data:
            return

        try:
            offset, rotacion = desplazamiento_del_site_ik(msg.data, self.escena.parent)

        except Exception as error:

            self.get_logger().error(f"No se pudo leer el modelo IK: {error}")
            return

        self.ruta_modelo_ik = msg.data

        self.offset_site = offset
        self.R_site_link7 = rotacion

        self.get_logger().info(f"Modelo IK: {msg.data}")

    # ========================================================
    # PUBLICAR MOVIMIENTO DEL CUBO
    # ========================================================

    def publicar_cubo(self, desplazandose):

        msg = Float64MultiArray()

        x, y, z = self.cubo_world

        # El cubo entra por -Y, a DISTANCIA_INICIO del punto de encuentro, y
        # recorre RECORRIDO_CUBO: si no se intercepta, pasa de largo.
        y_inicial = float(y - DISTANCIA_INICIO)

        if desplazandose:
            msg.data = [float(x), y_inicial, float(z), 0.0, VELOCIDAD_CUBO, 0.0, RECORRIDO_CUBO / VELOCIDAD_CUBO]

        else:
            msg.data = [float(x), y_inicial, float(z), 0.0, 0.0, 0.0, 5.0]

        self.pub_cubo.publish(msg)

    # ========================================================
    # ENVIAR OBJETIVO A IK_NODE.PY
    # ========================================================

    def publicar_ik_para_link7(self, objetivo_link7):

        # Convertir objetivo de link7 al objetivo de pinch_site
        posicion_site = (objetivo_link7 + self.R_link7_obj @ self.offset_site)

        R_site = (self.R_link7_obj @ self.R_site_link7)

        rpy = rpy_desde_matriz(R_site)

        msg = Float64MultiArray()

        # Formato de /obj_point:
        # [x, y, z, roll, pitch, yaw]

        msg.data = [float(v) for v in np.concatenate((posicion_site, rpy))]

        self.pub_objetivo.publish(msg)

        self.get_logger().info(f"Objetivo IK: {np.round(msg.data, 4)}")

    # ========================================================
    # SEGUIMIENTO DEL CUBO EN TIEMPO REAL
    # ========================================================

    def on_cubo(self, msg):

        ahora = time.monotonic()
        y = msg.pose.position.y

        self.cubo_y = y

        # Velocidad media sobre una ventana (ver VENTANA_VELOCIDAD).
        self.historial_cubo.append((ahora, y))

        while (len(self.historial_cubo) > 1
               and ahora - self.historial_cubo[0][0] > VENTANA_VELOCIDAD):
            self.historial_cubo.popleft()

        t_viejo, y_viejo = self.historial_cubo[0]

        if ahora - t_viejo > 0.2:
            self.cubo_vel = (y - y_viejo) / (ahora - t_viejo)

    def on_contacto(self, msg):

        if msg.data:
            self.contacto = True

    def tiempo_hasta_encuentro(self):
        """Segundos que le faltan al cubo para llegar al punto de encuentro.

        None si aún no hay medidas; negativo si ya pasó de largo.
        """
        if self.cubo_y is None:
            return None

        restante = self.cubo_base[1] - self.cubo_y

        # Se usa la velocidad medida; solo se recurre a la ordenada si aún no
        # hay medida o el cubo todavía no se mueve.
        v = self.cubo_vel
        if v is None or v < 0.01:
            v = VELOCIDAD_CUBO

        return restante / v

    # ========================================================
    # VERIFICAR POSICIÓN
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
        # Buscar modelo IK alternativo si es necesario
        # ----------------------------------------------------

        if self.etapa == 0 and self.offset_site is None:

            if (time.monotonic() - self.inicio_espera_modelo > 3.0 and not self.intento_fallback and not self.aviso_modelo):

                try:
                    candidato = encontrar_modelo_ik()

                except FileNotFoundError as error:
                    candidato = None

                    if not self.aviso_modelo:
                        self.aviso_modelo = True

                        self.get_logger().error(f"No se recibió /model_path y no hay modelo IK local. Comprueba que MuJoCo e IK_node.py estén abiertos. {error}")

                if candidato is not None:

                    self.intento_fallback = True

                    self.get_logger().warn("No se recibió /model_path. Usando modelo IK local; comprueba que sea el mismo de IK_node.py.")

                    self.on_modelo_ik(String(data=str(candidato)))

        # ----------------------------------------------------
        # ETAPA 0: Esperar ROS 2 e iniciar aproximación
        # ----------------------------------------------------

        if self.etapa == 0:

            if self.offset_site is None:
                return

            if self.tcp is None:
                return

            if self.pub_objetivo.get_subscription_count() == 0:
                return

            if self.pub_cubo.get_subscription_count() == 0:
                return

            # Abrir el gripper antes de comenzar
            if not self.orden_apertura_enviada:

                self.abrir_gripper()

                self.orden_apertura_enviada = True

                self.get_logger().info("Esperando apertura completa del gripper...")

                return

            # Esperar hasta que la apertura alcance al menos 9.5 cm
            if (self.apertura_gripper_actual is None or self.apertura_gripper_actual < 0.095):
                return

            self.get_logger().info("Gripper abierto. Iniciando intercepción.")

            # El cubo espera en la entrada de la cinta; el brazo se preposiciona.
            # Soltarlo antes seria una carrera perdida: el brazo tarda mas en
            # colocarse que el cubo en cruzar.
            self.publicar_cubo(desplazandose=False)
            self.publicar_ik_para_link7(self.aprox_link7)

            self.momento_etapa = time.monotonic()

            self.etapa = 1

            self.get_logger().info(
                f"FASE 1: preposicionando sobre el punto de encuentro "
                f"(el cubo entra a {DISTANCIA_INICIO:.2f} m, {VELOCIDAD_CUBO:.2f} m/s).")

        # ----------------------------------------------------
        # ETAPA 1: Preposicionarse por encima del encuentro
        # ----------------------------------------------------

        elif self.etapa == 1:

            tiempo = (time.monotonic() - self.momento_etapa)

            llego = self.cerca_de(self.aprox_link7)

            if llego or tiempo > MAX_ESPERA:

                if not llego:
                    self.get_logger().warn("No se alcanzó la aproximación con la tolerancia indicada.")

                # Brazo listo: ahora sí arranca la cinta.
                self.publicar_cubo(desplazandose=True)

                self.get_logger().info(
                    f"FASE 2: cinta en marcha ({VELOCIDAD_CUBO:.2f} m/s desde "
                    f"{DISTANCIA_INICIO:.2f} m). Se baja cuando al cubo le "
                    f"falten {TIEMPO_BAJADA:.2f} s.")

                self.momento_etapa = time.monotonic()

                self.etapa = 2

        # ----------------------------------------------------
        # ETAPA 2: Esperar al cubo y disparar la bajada
        # ----------------------------------------------------

        elif self.etapa == 2:

            restante = self.tiempo_hasta_encuentro()

            if restante is None:

                if not self.disparo_anunciado and time.monotonic() - self.momento_etapa > 3.0:

                    self.disparo_anunciado = True

                    self.get_logger().error(
                        "No llega /feedback/cubo_pose: sin realimentación no se "
                        "puede sincronizar la bajada.")

                return

            if restante < -MARGEN_PASADO / VELOCIDAD_CUBO:

                self.get_logger().warn(
                    "El cubo pasó de largo antes de disparar la bajada. "
                    f"Sube TIEMPO_BAJADA (ahora {TIEMPO_BAJADA:.2f} s).")

                self.etapa = 4

                return

            if restante <= TIEMPO_BAJADA:

                self.publicar_ik_para_link7(self.obj_link7)

                self.momento_disparo = time.monotonic()
                self.momento_etapa = self.momento_disparo

                self.etapa = 3

                self.get_logger().info(
                    f"FASE 3: bajando. Al cubo le faltan {restante:.2f} s "
                    f"(y = {self.cubo_y:.3f} m, v = {self.cubo_vel or VELOCIDAD_CUBO:.3f} m/s).")

        # ----------------------------------------------------
        # ETAPA 3: Bajada e intercepción
        # ----------------------------------------------------

        elif self.etapa == 3:

            if self.cerca_de(self.obj_link7):

                bajada = time.monotonic() - self.momento_disparo

                restante = self.tiempo_hasta_encuentro()

                self.get_logger().info(
                    f"Brazo en el punto de intercepción. Bajada real: {bajada:.2f} s "
                    f"(TIEMPO_BAJADA = {TIEMPO_BAJADA:.2f} s).")

                if restante is not None and restante > 0.0:

                    # El valor correcto de TIEMPO_BAJADA es lo que tarda la
                    # bajada más un margen, NO el tiempo que sobró.
                    self.get_logger().info(
                        f"Llegó {restante:.2f} s antes que el cubo. "
                        f"Ajuste sugerido: TIEMPO_BAJADA = {1.5 * bajada:.2f} s "
                        f"(bajada real + 50 % de margen).")

                elif restante is not None:

                    self.get_logger().warn(
                        f"Llegó {-restante:.2f} s tarde: el cubo ya había pasado. "
                        f"Ajuste sugerido: TIEMPO_BAJADA = "
                        f"{1.2 * (bajada - restante):.2f} s.")

                self.etapa = 4

            elif (time.monotonic() - self.momento_etapa > MAX_ESPERA):

                self.get_logger().warn(
                    "El brazo no alcanzó la tolerancia durante la bajada. "
                    "Revisar convergencia de IK_node.py.")

                self.etapa = 4

        # ----------------------------------------------------
        # ETAPA 4: Fin de la maniobra
        # ----------------------------------------------------

        elif self.etapa == 4:

            if self.contacto:

                self.get_logger().info(
                    f"INTERCEPTADO: el cubo tocó el robot y se detuvo "
                    f"(y = {self.cubo_y:.3f} m).")

                self.etapa = 5

                return

            restante = self.tiempo_hasta_encuentro()

            if restante is not None and restante < -MARGEN_PASADO / VELOCIDAD_CUBO:

                self.get_logger().warn(
                    f"NO INTERCEPTADO: el cubo siguió de largo "
                    f"(y = {self.cubo_y:.3f} m). Ajusta TIEMPO_BAJADA o la "
                    f"velocidad del cubo.")

                self.etapa = 5

    def on_joint_states(self, msg):

        if "gripper" in msg.name:

            i = msg.name.index("gripper")

            if i < len(msg.position):
                self.apertura_gripper_actual = msg.position[i]

    def abrir_gripper(self):

        msg = JointState()

        msg.name = ["gripper"]
        msg.position = [0.10]

        self.pub_gripper.publish(msg)

        self.get_logger().info("Gripper abierto a 10 cm.")

# ============================================================
# PROGRAMA PRINCIPAL
# ============================================================

def main():

    rclpy.init()

    nodo = None

    try:
        nodo = InterceptarCubo()

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
