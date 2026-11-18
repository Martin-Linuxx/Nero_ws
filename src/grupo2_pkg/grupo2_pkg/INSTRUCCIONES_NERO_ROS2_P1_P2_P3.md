TAREA ROS 2 + MUJOCO — NERO
Instrucciones para ejecutar las preguntas 1, 2 y 3 en OTRO computador
---
# IMPORTANTE
- Sistema de referencia: Ubuntu 24.04 + ROS 2 Jazzy.
- El compañero debe tener ROS 2 Jazzy, colcon y las dependencias de los paquetes ROS instaladas.
- Ejecutar UNA pregunta a la vez. Cerrar los procesos anteriores con Ctrl+C antes de reiniciar la simulación.
- Si se guarda el proyecto en otra ubicación, sustituir $HOME/Robotica/Nero_ws por esa ruta en los comandos.
- Las rutinas viven en el paquete `grupo2_pkg` y se ejecutan con `ros2 run` desde cualquier directorio: localizan la escena de MuJoCo por sí solas (ver `grupo2_pkg/model_paths.py`).
- Si el workspace está en otra ubicación no hay que cambiar nada. Para usar otra carpeta de modelos: `export NERO_MUJOCO_DIR=<carpeta>` o `ros2 run grupo2_pkg interceptar_cubo --ros-args -p model_dir:=<carpeta>`.
- Las preguntas 2 y 3 necesitan la escena con el cubo móvil (`scene:=scene_nero_cubo.xml`); la escena base `scene_nero_arm.xml` no tiene cubo.

---

# A) MOVIMIENTO DEL ROBOT MEDIANTE ROS 2
Abrir TRES terminales. Correr los bloques en orden.

## TERMINAL 1 — MuJoCo + puente + nodos ROS:
```bash
source "$HOME/mujoco-env/bin/activate"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch nero_ctrl nero_can_ctrl.launch.py mode:=sim
```

## TERMINAL 2 — Cinemática inversa:
```bash
source "$HOME/mujoco-env/bin/activate"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run grupo2_pkg ik_node
```

## TERMINAL 3 — Movimientos de la pregunta 1:
```bash
source "$HOME/mujoco-env/bin/activate"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run grupo2_pkg movimiento_continuo
```
---

# B) INTERCEPCION DEL CUBO

## Todo en uno (recomendado): un solo terminal
```bash
ros2 launch grupo2_pkg interceptar_cubo.launch.py
```
Levanta la simulacion con la escena del cubo, la cinematica inversa del grupo y
la rutina de intercepcion. Argumentos utiles: `rviz:=true`, `alpha:=0.8`,
`start_delay:=8.0`, `scene:=...`. Ctrl+C cierra las tres cosas.

## O a mano, en tres terminales:
Cerrar los procesos previos con Ctrl+C; reiniciar desde estado inicial.

## TERMINAL 1 — MuJoCo + puente + nodos ROS:
```bash
source "$HOME/mujoco-env/bin/activate"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch nero_ctrl nero_can_ctrl.launch.py mode:=sim scene:=scene_nero_cubo.xml
```

## TERMINAL 2 — Cinemática inversa:
```bash
source "$HOME/mujoco-env/bin/activate"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run grupo2_pkg ik_node
```

## TERMINAL 3 — Intercepción del cubo:
```bash 
source "$HOME/mujoco-env/bin/activate"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run grupo2_pkg interceptar_cubo
```
## OPCIONAL, TERMINAL 4 — Señal de contacto del cubo:
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic echo /feedback/cubo_contacto
```

---

# C) CIERRE DE GARRA Y LEVANTAMIENTO

Cerrar los procesos previos con Ctrl+C; reiniciar desde estado inicial.

## TERMINAL 1 — MuJoCo + puente + nodos ROS:
```bash
source "$HOME/mujoco-env/bin/activate"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch nero_ctrl nero_can_ctrl.launch.py mode:=sim scene:=scene_nero_cubo.xml
```

## TERMINAL 2 — Cinemática inversa:
```bash
source "$HOME/mujoco-env/bin/activate"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run grupo2_pkg ik_node
```

## TERMINAL 3 — Agarre físico y levantamiento:
```bash
source "$HOME/mujoco-env/bin/activate"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run grupo2_pkg recoger_cubo
```

## OPCIONAL, TERMINAL 4 — Contacto simultáneo de ambos dedos:
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic echo /feedback/cubo_agarre
```
---

# COMPROBACIONES SI ALGO NO FUNCIONA
## Terminal extra (con la simulación ejecutándose):
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 node list
ros2 topic list
ros2 topic echo /feedback/tcp_pose --once
```

## Si el robot no se mueve, comprobar si IK publica:
```bash
ros2 topic echo /ik/ik_result --once
```
## Si no llegan comandos al puente:
```bash
ros2 topic echo /control/move_j --once
```
## Si MuJoCo o los scripts no encuentran módulos Python:
```bash
source "$HOME/mujoco-env/bin/activate"
python3 -c 'import mujoco, rclpy, numpy; print("Imports OK")'
```
## Otras comprobaciones:
- Si un nodo no encuentra la escena, el error lista todas las carpetas probadas:
  define NERO_MUJOCO_DIR=<carpeta> o pasa -p model_dir:=<carpeta>.
- Tras cambiar código: colcon build --symlink-install && source install/setup.bash.
- El error de RViz asociado a Snap/libpthread puede ser independiente de MuJoCo.
---

# NOTAS SOBRE LOS ARCHIVOS
- P1: src/grupo2_pkg/grupo2_pkg/movimiento_continuo.py (`ros2 run grupo2_pkg movimiento_continuo`)
- P2: src/grupo2_pkg/grupo2_pkg/interceptar_cubo.py (`ros2 run grupo2_pkg interceptar_cubo`)
- P3: src/grupo2_pkg/grupo2_pkg/recoger_cubo.py (`ros2 run grupo2_pkg recoger_cubo`)
- Localización de los modelos MuJoCo: src/grupo2_pkg/grupo2_pkg/model_paths.py
- Control inverso: src/nero_ctrl/nero_ctrl/IK_node.py
- Puente MuJoCo: src/agx_arm_description/scripts/mujoco_viewer.py
- Escena MuJoCo con el cubo: src/mujoco/agilex_arm/agilex_nero/scene_nero_cubo.xml
- Escena MuJoCo base (sin cubo): src/mujoco/agilex_arm/agilex_nero/scene_nero_arm.xml
- Brazo y gripper: src/mujoco/agilex_arm/agilex_nero/nero_arm.xml
- Lanzamiento: paquete nero_ctrl, archivo nero_can_ctrl.launch.py, modo sim.


