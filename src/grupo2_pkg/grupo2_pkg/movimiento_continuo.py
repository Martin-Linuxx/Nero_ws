import time
import math
import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Float64MultiArray


class MovimientoContinuo(Node):

    def __init__(self):
        super().__init__('movimiento_continuo')

        # Publicador ROS 2
        self.publicador = self.create_publisher(Float64MultiArray, '/obj_point', QoSProfile(
                depth=10,
                durability=DurabilityPolicy.TRANSIENT_LOCAL
            )
        )

        # Posición central
        self.x = -0.17
        self.y_centro = -0.0235
        self.z = 0.4156

        # Orientación
        self.roll = -1.5708
        self.pitch = 0.02927
        self.yaw = 3.14159

        # Parámetros del movimiento
        self.amplitud = 0.1
        self.periodo = 5.0
        self.intervalo = 0.2

        self.t_inicio = None
        self.contador = 0

        # Temporizador ROS 2
        self.timer = self.create_timer(
            self.intervalo,
            self.actualizar_movimiento_cb
        )

        self.get_logger().info( 'Control de movimiento continuo iniciado.')

    def actualizar_movimiento_cb(self):

        # Comprobar conexion con IK_node
        if self.publicador.get_subscription_count() == 0:
            return

        # Iniciar tiempo al establecer conexion
        if self.t_inicio is None:
            self.t_inicio = time.monotonic()

            self.get_logger().info('Conexion establecida. Iniciando trayectoria.')

        # Tiempo transcurrido
        t = time.monotonic() - self.t_inicio

        # Movimiento hacia adelante y atrás
        x = (self.x - self.amplitud / 2 * (1 - math.cos(2 * math.pi * t / self.periodo)))

        # Crear mensaje de posicion objetivo
        mensaje = Float64MultiArray()
        mensaje.data = [
            float(x),
            float(self.y_centro),
            float(self.z),
            float(self.roll),
            float(self.pitch),
            float(self.yaw)
        ]

        # Publicar objetivo mediante ROS 2
        self.publicador.publish(mensaje)

        self.get_logger().info(f'Tiempo: {t:.1f} s | Posicion X objetivo: {x:.4f} m')


def main():

    rclpy.init()

    nodo = MovimientoContinuo()

    try:
        print("Movimiento continuo del robot Nero")
        print("Presiona Ctrl + C para detener.")

        rclpy.spin(nodo)

    except KeyboardInterrupt:
        print("\nMovimiento detenido.")

    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
