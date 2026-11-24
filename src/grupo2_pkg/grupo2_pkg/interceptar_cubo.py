#!/usr/bin/env python3
"""P2: seguir el cubo en la cinta e interceptarlo.

Nada mas arrancar, el brazo va con un move_p planificado (el puente busca una
configuracion del brazo que llegue) a la pose de espera: mano ya en la orientacion
de agarre (apertura paralela a la cinta, pinza inclinada 30 grados hacia fuera),
sobrevolando la cinta en s_espera (25 cm del extremo cercano), por delante del
cubo. El cubo viene del extremo lejano hacia el brazo: entra en alcance hacia
y = -0.50 y el agarre tiene que terminar antes de s_limite (cerca del final). Cuando el cubo se acerca, se engancha a el
y lo persigue sobrevolandolo (ya alineado, solo corrige posicion). Estable sobre el
cubo, baja sin dejar de seguirlo hasta tener un dedo delante y otro detras. Ahi el
brazo se detiene y el dedo delantero frena el cubo.

recoger_cubo.py (P3) reutiliza esta clase: en vez de detenerse, cierra la pinza
siguiendo al cubo y lo levanta.

Estimacion: el cubo va siempre por la misma linea de la cinta y a la velocidad de
la cinta (v_cinta, conocida), asi que solo falta saber DONDE va: el sensor de
proximidad (al inicio de la cinta, mirando en su sentido de avance) da la distancia
s del cubo a lo largo de la cinta. Mientras el cubo esta quieto, s es la mediana de
las lecturas; cuando arranca, s(t) = s0 + v_cinta * t y s0 sale de la mediana de
las lecturas en marcha. Las lecturas que no encajan (el sensor ve un dedo en vez
del cubo) se descartan, y desde que empieza la bajada se extrapola sin leer el
sensor (los dedos lo tapan). Con v_cinta <= 0 la velocidad se estima con un ajuste
lineal.

Control: a 20 Hz se manda la pose de la brida por /control/servo_p (IK continua en
el puente). El punto de agarre comandado avanza con la velocidad del cubo mas una
correccion limitada a v_max, y la orientacion gira hacia la de agarre a w_max.

  publica  /control/move_p           geometry_msgs/PoseStamped  pose de espera (planificada)
  publica  /control/servo_p          geometry_msgs/PoseStamped  pose de link7 (flujo)
  publica  /control/joint_states     sensor_msgs/JointState     apertura del gripper
  escucha  /proximity                sensor_msgs/Range          distancia al cubo
  escucha  /feedback/tcp_pose        geometry_msgs/PoseStamped  pose real de link7
  escucha  /feedback/joint_states    sensor_msgs/JointState     apertura real
  escucha  /feedback/cubo_contacto   std_msgs/Bool              (solo simulacion) verificacion

Fases: ESPERA -> PREPARAR -> SEGUIR -> BAJAR -> DETENER -> FIN  (si falla: ABORTAR -> FIN)
"""

import math
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState, Range
from std_msgs.msg import Bool

ESPERA = 'espera'
PREPARAR = 'preparar'
SEGUIR = 'seguir'
BAJAR = 'bajar'
DETENER = 'detener'
ABORTAR = 'abortar'
FIN = 'fin'


# ============================================================
# CUATERNIONES (x, y, z, w)
# ============================================================

def quat_a_matriz(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matriz_a_quat(R):
    t = np.trace(R)
    if t > 0:
        s = 2.0 * math.sqrt(t + 1.0)
        q = [(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s]
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2.0 * math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k])
        q = [0.0, 0.0, 0.0, (R[k, j] - R[j, k]) / s]
        q[i] = 0.25 * s
        q[j] = (R[j, i] + R[i, j]) / s
        q[k] = (R[k, i] + R[i, k]) / s
    q = np.array(q)
    return q / np.linalg.norm(q)


def angulo_entre(q0, q1):
    return 2.0 * math.acos(min(1.0, abs(float(np.dot(q0, q1)))))


def girar_hacia(q0, q1, max_ang):
    """Slerp de q0 hacia q1 avanzando como mucho max_ang radianes."""
    if np.dot(q0, q1) < 0.0:
        q1 = -q1
    ang = angulo_entre(q0, q1)
    if ang <= max_ang or ang < 1e-9:
        return q1.copy()
    t = max_ang / ang
    th = ang / 2.0
    q = (math.sin((1 - t) * th) * q0 + math.sin(t * th) * q1) / math.sin(th)
    return q / np.linalg.norm(q)


class InterceptarCubo(Node):
    def __init__(self, nombre='interceptar_cubo', **defaults):
        super().__init__(nombre)

        self.setup_parameters(defaults)
        self.init_communications()

        self.fase = ESPERA
        self.t0 = time.monotonic()

        self.tcp = None                 # (p, q) de link7 medidos
        self.apertura = None
        self.contacto_sim = None
        self.muestras = deque()         # (t, s) lecturas aceptadas
        self.estimacion = None          # (s, v, t) de la ultima estimacion
        self.s_reposo = None            # s del cubo quieto antes de arrancar la cinta
        self.en_marcha = False
        self.congelada = False          # desde la bajada no se lee el sensor

        self.p_cmd = None               # punto de agarre comandado (mundo)
        self.q_cmd = None               # orientacion comandada de link7
        self.altura = self.altura_sobrevuelo   # del punto de agarre sobre su objetivo
        self.desde_alineado = None
        self.aproximando = False
        self.correccion = np.zeros(3)   # m, compensa el error horizontal medido (retraso)
        self.v_cmd = np.zeros(3)        # m/s, velocidad del punto de agarre comandado

        self.get_logger().info(
            f'Siguiendo el cubo: cinta desde {np.round(self.sensor_p, 3)} hacia '
            f'{np.round(self.dir, 3)}, agarre antes de s = {self.s_limite:.2f} m. '
            f'Pinza inclinada {math.degrees(self.inclinacion):.0f} grados.')

    # ========================================================
    # PARAMETROS Y COMUNICACIONES
    # ========================================================

    def setup_parameters(self, defaults):
        p = {
            # Cinta y sensor (marco base_link). El cubo esta en sensor_pos + s * sensor_dir.
            'sensor_pos': [-0.20, -0.92, 0.085],   # m, a la altura del centro del cubo
            'sensor_dir': [0.0, 1.0, 0.0],        # sentido de avance de la cinta (hacia el brazo)
            'medio_cubo': 0.02,                   # m, de la cara al centro del cubo
            'v_cinta': 0.08,                      # m/s, velocidad conocida (<= 0: estimarla)
            's_limite': 0.67,                     # m, agarrar antes (y = -0.25, cerca del extremo)
            's_espera': 0.54,                     # m, donde espera alineado (y = -0.38, 25 cm del extremo cercano)
            'd_enganche': 0.15,                   # m, se engancha al cubo a esta distancia
            # Pinza
            'inclinacion': math.radians(-20.0),   # rad, + hacia la base, - hacia fuera (mas alcance)
            'eje_apertura': [0.0, -1.0, 0.0],     # Z de link7 (a lo largo de la cinta; signo con IK probada)
            'punto_agarre_link7': [0.135, 0.0, -0.0235],  # m, centro de los dedos en link7
            'offset_agarre': [0.0, 0.0, 0.04],    # m, punto de agarre - centro del cubo (>= 16 mm
                                                  #    de holgura de toda la pinza a la cinta)
            # Limites duros de seguridad para el punto de agarre comandado (base_link).
            'z_min_agarre': 0.12,                 # m, nunca mas bajo (agarre = 0.125)
            'caja_min': [-0.32, -0.62, 0.12],     # m, zona de trabajo sobre la cinta
            'caja_max': [-0.08, -0.10, 0.50],
            'altura_aproximacion': 0.12,          # m, primero llega por encima de la espera
            'altura_sobrevuelo': 0.05,            # m sobre el punto de agarre
            'apertura_abierta': 0.10,             # m
            'umbral_abierto': 0.085,              # m
            # Seguimiento
            'rate': 20.0,                         # Hz de /control/servo_p
            'v_max': 0.15,                        # m/s, correccion sobre la velocidad del cubo
            'a_max': 0.5,                         # m/s^2, aceleracion maxima del comando (suavidad)
            'kp': 2.0,                            # 1/s, correccion proporcional hacia el objetivo
            'w_max': 1.2,                         # rad/s, giro de la mano
            'v_bajada': 0.08,                     # m/s
            't_adelanto': 0.3,                    # s, apunta por delante del cubo (retraso del seguidor)
            'ki': 1.5,                            # 1/s, integral del error medido (retraso, IK)
            'max_correccion': 0.05,               # m, limite de esa correccion
            'tol_agarre': 0.012,                  # m, precision abajo antes de cerrar (los dedos
                                                  #    abiertos libran el cubo hasta 2 cm)
            'tol_pos': 0.02,                      # m, alineado en posicion
            'tol_ang': 0.12,                      # rad (~7 grados), alineado en orientacion
            't_alineacion': 0.3,                  # s alineado antes de bajar
            # Estimacion
            'ventana': 1.0,                       # s de lecturas para la estimacion
            'min_muestras': 3,
            'atipico': 0.025,                     # m, residuo para descartar una lectura
            'umbral_marcha': 0.01,                # m sobre la posicion de reposo = en marcha
            # P2
            't_detenido': 1.5,                    # s quieto con el cubo entre los dedos
            'timeout': 30.0,                      # s por fase
        }
        p.update(defaults)
        for k, v in p.items():
            setattr(self, k, self.declare_parameter(k, v).value)

        self.sensor_p = np.array(self.sensor_pos, dtype=float)
        self.dir = np.array(self.sensor_dir, dtype=float)
        self.dir /= np.linalg.norm(self.dir)
        self.g = np.array(self.punto_agarre_link7, dtype=float)
        self.offset = np.array(self.offset_agarre, dtype=float)
        self.q_agarre = self.orientacion_agarre()
        self.caja_lo = np.array(self.caja_min, dtype=float)
        self.caja_hi = np.array(self.caja_max, dtype=float)
        self.caja_lo[2] = max(self.caja_lo[2], float(self.z_min_agarre))

    def orientacion_agarre(self):
        """link7: X (hacia los dedos) abajo e inclinado `inclinacion` hacia la base (negativo:
        hacia fuera); Z (apertura) a lo largo de la cinta."""
        hacia_base = -self.sensor_p.copy()
        hacia_base[2] = 0.0
        hacia_base -= (hacia_base @ self.dir) * self.dir
        hacia_base /= np.linalg.norm(hacia_base)
        x7 = math.sin(self.inclinacion) * hacia_base - math.cos(self.inclinacion) * np.array([0, 0, 1.0])
        z7 = np.array(self.eje_apertura, dtype=float)
        z7 /= np.linalg.norm(z7)
        y7 = np.cross(z7, x7)
        return matriz_a_quat(np.column_stack([x7, y7, z7]))

    def init_communications(self):
        self.pub_servo = self.create_publisher(PoseStamped, '/control/servo_p', 10)
        self.pub_move_p = self.create_publisher(PoseStamped, '/control/move_p', 10)
        self.pub_directo = self.create_publisher(JointState, '/control/joint_states', 10)

        self.create_subscription(Range, '/proximity', self.proximity_cb, 10)
        self.create_subscription(PoseStamped, '/feedback/tcp_pose', self.tcp_cb, 10)
        self.create_subscription(JointState, '/feedback/joint_states', self.joint_states_cb, 10)
        self.create_subscription(Bool, '/feedback/cubo_contacto', self.contacto_cb, 10)

        self.dt = 1.0 / float(self.rate)
        self.timer = self.create_timer(self.dt, self.rutina_cb)

    # ========================================================
    # CALLBACKS
    # ========================================================

    def tcp_cb(self, msg):
        p, o = msg.pose.position, msg.pose.orientation
        self.tcp = (np.array([p.x, p.y, p.z]), np.array([o.x, o.y, o.z, o.w]))

    def joint_states_cb(self, msg):
        if 'gripper' in msg.name:
            i = msg.name.index('gripper')
            if i < len(msg.position):
                self.apertura = float(msg.position[i])

    def contacto_cb(self, msg):
        self.contacto_sim = bool(msg.data)

    def proximity_cb(self, msg):
        if self.congelada or not (msg.min_range < msg.range < msg.max_range):
            return
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        s = msg.range + self.medio_cubo
        # Con el cubo en marcha, una lectura que no encaja con la prediccion es un dedo.
        if self.en_marcha and abs(s - self.s_en(t)) > self.atipico:
            return
        if not self.en_marcha and self.s_reposo is not None \
                and s - self.s_reposo > self.umbral_marcha:
            self.en_marcha = True
            self.muestras.clear()   # las lecturas en reposo no sirven para s0
            # La prediccion de reposo (v = 0) ya no vale: se parte de esta lectura.
            self.estimacion = (s, max(0.0, float(self.v_cinta)), t)
            self.get_logger().info(f'El cubo arranco (s = {s:.3f} m).')
        self.muestras.append((t, s))
        while self.muestras and t - self.muestras[0][0] > self.ventana:
            self.muestras.popleft()
        est = self.ajustar()
        if est is not None:
            self.estimacion = est

    # ========================================================
    # ESTIMACION DEL CUBO
    # ========================================================

    def ajustar(self):
        """Estimacion (s, v, t) en la ultima muestra.

        Quieto: mediana de las lecturas. En marcha: s(t) = s0 + v * t con v = v_cinta
        (o ajustada si v_cinta <= 0) y s0 de la mediana, robusta a lecturas raras.
        """
        if len(self.muestras) < self.min_muestras:
            return None
        t, s = np.array(self.muestras).T
        t_ult = t[-1]
        if not self.en_marcha:
            s_med = float(np.median(s))
            self.s_reposo = s_med
            return s_med, 0.0, float(t_ult)
        if self.v_cinta > 0.0:
            v = float(self.v_cinta)
        else:
            v = max(0.0, float(np.polyfit(t - t_ult, s, 1)[0]))
        return float(np.median(s - v * (t - t_ult))), v, float(t_ult)

    def ahora(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def s_en(self, t):
        s, v, t_est = self.estimacion
        return s + v * (t - t_est)

    def cubo(self, adelanto=0.0):
        """Centro estimado del cubo (mundo) dentro de `adelanto` segundos."""
        return self.sensor_p + self.s_en(self.ahora() + adelanto) * self.dir

    def velocidad_cubo(self):
        return self.estimacion[1] * self.dir

    # ========================================================
    # ORDENES
    # ========================================================

    def enviar_servo(self, pub=None):
        """Publica la pose de link7 que pone el punto de agarre en p_cmd con q_cmd.

        Limite duro: el punto de agarre comandado nunca sale de la caja de trabajo ni
        baja de z_min_agarre, pase lo que pase en la estimacion o el control.
        """
        self.p_cmd = np.clip(self.p_cmd, self.caja_lo, self.caja_hi)
        p7 = self.p_cmd - quat_a_matriz(self.q_cmd) @ self.g
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, p7)
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = map(float, self.q_cmd)
        (pub or self.pub_servo).publish(msg)

    def enviar_gripper(self, apertura):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['gripper']
        msg.position = [float(apertura)]
        self.pub_directo.publish(msg)

    def punto_espera(self):
        return self.sensor_p + self.s_espera * self.dir

    def espera(self):
        """Punto de agarre en la pose de espera (sobrevolando la cinta en s_espera)."""
        return self.punto_espera() + self.offset + np.array([0.0, 0.0, self.altura_sobrevuelo])

    def objetivo_agarre(self):
        """Punto de agarre deseado: sobre el cubo predicho, a la altura actual, mas la
        correccion integral que compensa el error medido (retraso del seguidor, IK)."""
        return (self.cubo(self.t_adelanto) + self.offset + np.array([0.0, 0.0, self.altura])
                + self.correccion)

    def seguir(self, v_extra=None):
        """Un paso del seguimiento, suave: la velocidad deseada es la del objetivo (cubo +
        v_extra, p. ej. la bajada) mas una correccion proporcional limitada a v_max, y la
        velocidad comandada solo cambia a a_max (sin tirones que el seguidor articular
        convierta en cabeceos). La mano gira hacia q_agarre a w_max."""
        err = self.objetivo_agarre() - self.p_cmd
        corr = self.kp * err
        n = float(np.linalg.norm(corr))
        if n > self.v_max:
            corr *= self.v_max / n
        v_des = self.velocidad_cubo() + corr + (np.zeros(3) if v_extra is None else v_extra)
        dv = v_des - self.v_cmd
        n = float(np.linalg.norm(dv))
        lim = self.a_max * self.dt
        self.v_cmd = self.v_cmd + (dv if n <= lim else dv * lim / n)
        self.p_cmd = self.p_cmd + self.v_cmd * self.dt
        self.q_cmd = girar_hacia(self.q_cmd, self.q_agarre, self.w_max * self.dt)
        self.enviar_servo()
        # Integral del error medido a lo largo de la cinta (solo cerca: lejos manda v_max).
        if self.tcp is not None:
            deseado = self.cubo() + self.offset + np.array([0.0, 0.0, self.altura])
            e = deseado - self.agarre_medido()
            e[2] = 0.0   # solo en horizontal: en z peleaba con el seguidor y cabeceaba
            if np.linalg.norm(e) < 0.05:
                c = self.correccion + self.ki * e * self.dt
                n = float(np.linalg.norm(c))
                self.correccion = c if n <= self.max_correccion else c * self.max_correccion / n

    # ========================================================
    # MEDIDAS
    # ========================================================

    def agarre_medido(self):
        p7, q7 = self.tcp
        return p7 + quat_a_matriz(q7) @ self.g

    def error_seguimiento(self):
        """(error del punto de agarre respecto a su sitio sobre el cubo, error de orientacion)."""
        deseado = self.cubo() + self.offset + np.array([0.0, 0.0, self.altura])
        return (float(np.linalg.norm(self.agarre_medido() - deseado)),
                angulo_entre(self.tcp[1], self.q_agarre))

    # ========================================================
    # RUTINA
    # ========================================================

    def cambiar_fase(self, fase):
        self.fase = fase
        self.t0 = time.monotonic()

    def expirado(self):
        return time.monotonic() - self.t0 > self.timeout

    def terminar(self, mensaje, aviso=False):
        (self.get_logger().warn if aviso else self.get_logger().info)(mensaje)
        self.fase = FIN

    def fallar(self, mensaje):
        """Aborta: deja de seguir y sube la pinza 10 cm en vertical, poco a poco (un salto
        de golpe lo rechazaria la seguridad del puente y el brazo se quedaria abajo)."""
        self.get_logger().warn(mensaje)
        if self.p_cmd is None:
            self.fase = FIN
            return
        self.p_abortar = self.p_cmd + np.array([0.0, 0.0, 0.10])
        self.get_logger().warn('Abortando: subiendo la pinza...')
        self.cambiar_fase(ABORTAR)

    def tarde(self):
        """True si el cubo llego al limite de la cinta."""
        return self.estimacion is not None and self.s_en(self.ahora()) > self.s_limite

    def al_bajar(self):
        """Pinza abajo con el cubo entre los dedos: P2 detiene el brazo."""
        self.get_logger().info('Cubo entre los dedos. Deteniendo el brazo para frenarlo...')
        self.cambiar_fase(DETENER)

    def rutina_cb(self):
        if self.fase == FIN:
            return

        if self.fase == ABORTAR:
            err = self.p_abortar - self.p_cmd
            n = float(np.linalg.norm(err))
            lim = self.v_max * self.dt
            self.p_cmd = self.p_cmd + (err if n <= lim else err * lim / n)
            self.enviar_servo()
            if n < 1e-4 or self.expirado():
                self.fase = FIN
            return

        if self.fase == ESPERA:
            # No hace falta el cubo para alinearse: solo el brazo y el puente.
            if self.tcp is not None and self.pub_move_p.get_subscription_count() > 0:
                # Pose ya alineada, altura_aproximacion por encima de la espera, en un solo
                # movimiento planificado; luego baja en vertical hasta la espera.
                self.q_cmd = self.q_agarre.copy()
                self.p_cmd = self.espera() + np.array([0.0, 0.0, self.altura_aproximacion])
                self.enviar_servo(self.pub_move_p)
                self.aproximando = True
                self.enviar_gripper(self.apertura_abierta)
                self.get_logger().info(
                    f'Abriendo la pinza y alineando la mano con la cinta sobre s = '
                    f'{self.s_espera:.2f} m mientras llega el cubo...')
                self.cambiar_fase(PREPARAR)
            return

        if self.fase == PREPARAR:
            if self.aproximando:
                # Esperando el move_p por encima de la espera.
                if np.linalg.norm(self.agarre_medido() - self.p_cmd) < self.tol_pos:
                    self.aproximando = False
                    self.get_logger().info('Sobre la cinta y alineado. Bajando a la espera...')
                elif self.expirado():
                    self.fallar('No se llego sobre la cinta (move_p rechazado por IK o seguridad?).')
                return
            # Baja en vertical hasta la espera, a v_max.
            err = self.espera() - self.p_cmd
            n = float(np.linalg.norm(err))
            lim = self.v_max * self.dt
            self.p_cmd = self.p_cmd + (err if n <= lim else err * lim / n)
            self.enviar_servo()
            deseado = self.espera()
            e_pos = float(np.linalg.norm(self.agarre_medido() - deseado))
            e_ang = angulo_entre(self.tcp[1], self.q_agarre)
            listo = e_pos < self.tol_pos and e_ang < self.tol_ang
            self.get_logger().info(
                f'Preparando: error {1000 * e_pos:.0f} mm / {math.degrees(e_ang):.1f} grados'
                + ('' if self.estimacion is None else
                   f', cubo en s = {self.s_en(self.ahora()):.3f} m') + '.',
                throttle_duration_sec=1.0)
            if self.estimacion is None:
                return
            s_cubo = self.s_en(self.ahora())
            if listo and s_cubo >= self.s_espera - self.d_enganche:
                self.get_logger().info(
                    f'Alineado y esperando. Cubo en s = {s_cubo:.3f} m: enganchandose a el...')
                self.v_cmd = np.zeros(3)
                self.cambiar_fase(SEGUIR)
            elif s_cubo >= self.s_espera:
                # El cubo ya paso el punto de espera: se le persigue aunque no este alineado.
                self.get_logger().warn(
                    f'El cubo llego a s = {s_cubo:.3f} m sin terminar de alinear '
                    f'({1000 * e_pos:.0f} mm / {math.degrees(e_ang):.1f} grados): persiguiendolo.')
                self.cambiar_fase(SEGUIR)
            elif self.expirado():
                self.fallar('No se logro la pose de espera (IK rechazada?).')
            return

        if self.fase in (SEGUIR, BAJAR) and self.tarde():
            self.fallar(f'NO AGARRADO: el cubo llego a s = {self.s_limite:.2f} m (final de la '
                        f'cinta) antes de terminar. Sube v_max o baja la velocidad de la cinta.')
            return

        if self.fase == SEGUIR:
            self.seguir()
            e_pos, e_ang = self.error_seguimiento()
            abierta = self.apertura is not None and self.apertura >= self.umbral_abierto
            self.get_logger().info(
                f'Siguiendo: cubo en s = {self.s_en(self.ahora()):.3f} m, error '
                f'{1000 * e_pos:.0f} mm / {math.degrees(e_ang):.1f} grados, pinza '
                f'{100 * (self.apertura or 0.0):.1f} cm.', throttle_duration_sec=1.0)
            if e_pos < self.tol_pos and e_ang < self.tol_ang and abierta:
                if self.desde_alineado is None:
                    self.desde_alineado = time.monotonic()
                elif time.monotonic() - self.desde_alineado >= self.t_alineacion:
                    self.congelada = True
                    self.get_logger().info(
                        f'Alineado sobre el cubo (error {1000 * e_pos:.0f} mm, '
                        f'{math.degrees(e_ang):.1f} grados, cubo en s = '
                        f'{self.s_en(self.ahora()):.3f} m a {self.estimacion[1]:.3f} m/s). Bajando...')
                    self.cambiar_fase(BAJAR)
            else:
                self.desde_alineado = None
                if self.expirado():
                    self.fallar(f'No se logro alinear sobre el cubo (error {1000 * e_pos:.0f} mm, '
                                f'{math.degrees(e_ang):.1f} grados). IK rechazada?')

        elif self.fase == BAJAR:
            bajando = self.altura > 0.0
            self.altura = max(0.0, self.altura - self.v_bajada * self.dt)
            self.seguir(np.array([0.0, 0.0, -self.v_bajada]) if bajando else None)
            e_pos, _ = self.error_seguimiento()
            self.get_logger().info(
                f'Bajando: {100 * self.altura:.1f} cm sobre el agarre, error {1000 * e_pos:.0f} mm.',
                throttle_duration_sec=0.5)
            if self.altura == 0.0 and e_pos < self.tol_agarre:
                self.get_logger().info(f'Pinza abajo (error {1000 * e_pos:.0f} mm).')
                self.al_bajar()
            elif self.expirado():
                self.fallar(f'No se alcanzo la altura de agarre (error {1000 * e_pos:.0f} mm).')

        elif self.fase == DETENER:
            # No se envia nada: el seguidor del puente se queda en la ultima pose.
            if time.monotonic() - self.t0 >= self.t_detenido:
                if self.contacto_sim is None:
                    self.terminar('INTERCEPTADO: brazo detenido con el cubo entre los dedos.')
                elif self.contacto_sim:
                    self.terminar('INTERCEPTADO: el cubo choco con la pinza y quedo detenido.')
                else:
                    self.terminar('Brazo detenido, pero el cubo no toco la pinza.', aviso=True)


def main(args=None):
    rclpy.init(args=args)
    node = InterceptarCubo()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
