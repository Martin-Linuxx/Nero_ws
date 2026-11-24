#!/usr/bin/env python3
"""P3: seguir el cubo en la cinta, agarrarlo y levantarlo.

Igual que P2 (interceptar_cubo.py) hasta tener la pinza abajo con el cubo entre los
dedos: el brazo se alinea con la cinta, persigue el cubo y baja siguiendolo.
Despues cierra la pinza siguiendo el cubo SOLO hasta que los dedos lo tocan (la
apertura baja al ancho del cubo). En ese momento deja de seguir la cinta y lo
levanta en vertical; el agarre se confirma durante la subida (la apertura se
mantiene en el ancho del cubo).

Todo debe pasar antes de que el cubo llegue a s_limite (final de la cinta).

Fases: ESPERA -> SEGUIR -> BAJAR -> CERRAR -> LEVANTAR -> FIN
"""

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException

from grupo2_pkg.interceptar_cubo import InterceptarCubo

CERRAR = 'cerrar'
LEVANTAR = 'levantar'

class RecogerCubo(InterceptarCubo):
    def __init__(self):
        super().__init__('recoger_cubo')
        p = {
            'apertura_cerrada': 0.0,      # m, orden de cierre
            'ancho_min': 0.03,            # m, apertura minima con el cubo entre los dedos
            'ancho_max': 0.05,            # m
            'cierre_detenido': 0.001,     # m por ciclo: menos que esto = los dedos ya no cierran
            'altura_levantar': 0.10,      # m
        }
        for k, v in p.items():
            setattr(self, k, self.declare_parameter(k, v).value)
        self.levantado = None

    def al_bajar(self):
        self.enviar_gripper(self.apertura_cerrada)
        self.apertura_prev = None
        self.get_logger().info('Cerrando la pinza siguiendo el cubo hasta tocarlo...')
        self.cambiar_fase(CERRAR)

    def rutina_cb(self):
        if self.fase == CERRAR:
            if self.tarde():
                self.fallar('NO AGARRADO: el cubo llego al final de la cinta mientras se cerraba.')
                return
            a, a_prev = self.apertura, self.apertura_prev
            self.apertura_prev = a
            # Los dedos tocan el cubo: apertura en el ancho del cubo y ya no se cierra mas.
            tocando = (a is not None and a_prev is not None and a <= self.ancho_max
                       and a_prev - a < self.cierre_detenido)
            if a is not None and a < self.ancho_min:
                self.fallar(f'NO AGARRADO: la pinza cerro hasta {100 * a:.1f} cm sin cubo.')
            elif tocando:
                # Se deja de seguir la cinta y se sube desde aqui.
                self.levantado = self.p_cmd + np.array([0.0, 0.0, self.altura_levantar])
                self.get_logger().info(
                    f'Cubo entre los dedos en s = {self.s_en(self.ahora()):.3f} m '
                    f'(apertura {100 * a:.1f} cm). Dejando la cinta y levantando...')
                self.cambiar_fase(LEVANTAR)
            elif self.expirado():
                self.fallar('La pinza no termino de cerrar.')
            else:
                self.seguir()

        elif self.fase == LEVANTAR:
            # Sube en vertical desde donde agarro, a v_max.
            err = self.levantado - self.p_cmd
            n = float(np.linalg.norm(err))
            lim = self.v_max * self.dt
            self.p_cmd = self.p_cmd + (err if n <= lim else err * lim / n)
            self.enviar_servo()
            if self.apertura is not None and self.apertura < self.ancho_min:
                self.terminar('NO LEVANTADO: el cubo se solto al subir.', aviso=True)
            elif n < 1e-4 and np.linalg.norm(self.agarre_medido() - self.levantado) < self.tol_pos:
                self.terminar(
                    f'LEVANTADO: cubo sujeto {100 * self.altura_levantar:.0f} cm arriba '
                    f'(apertura {100 * self.apertura:.1f} cm).')
            elif self.expirado():
                self.terminar('No se alcanzo la altura de levantamiento.', aviso=True)

        else:
            super().rutina_cb()

def main(args=None):
    rclpy.init(args=args)
    node = RecogerCubo()
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
