"""Localizacion robusta de los modelos MuJoCo usados por grupo2_pkg.

Los nodos de este paquete necesitan dos modelos distintos:

* La *escena de la tarea* (``scene_nero_cubo.xml``): la que carga el simulador,
  con el brazo y el cubo movil. Vive junto a ``nero_arm.xml`` y ``assets/`` en
  ``mujoco/agilex_arm/agilex_nero/`` porque MuJoCo resuelve ``<include>`` y
  ``meshdir`` relativos al archivo principal.
* El *modelo de la cinematica inversa*: el que usa ``IK_node.py``, que es el
  unico que define el site ``pinch_site``
  (``models_for_simulation/agilex_nero/scene.xml`` -> ``nero.xml``). Normalmente
  llega por el topico ``/model_path``; esto es solo el respaldo.

Orden de busqueda de las carpetas candidatas (mismo patron que
``nero_ctrl/main_node.py:find_scene_xml`` y
``agx_arm_description/launch/display_mujoco.launch.py``):

1. ``model_dir`` explicito (parametro del nodo ``-p model_dir:=<carpeta>``).
2. Variable de entorno ``NERO_MUJOCO_DIR``.
3. ``share/<paquete>/<subdir>`` de los paquetes instalados.
4. Subiendo por las carpetas padre de este archivo, probando cada subdir
   relativo. Funciona con ``--symlink-install`` (``__file__`` apunta a ``src/``)
   y con instalacion normal (desde ``install/grupo2_pkg/lib/.../grupo2_pkg/``
   sube hasta la raiz del workspace y encuentra ``src/mujoco/...``).

Ademas, una variable de entorno que apunte directamente al archivo tiene
prioridad absoluta: ``NERO_ESCENA_CUBO`` para la escena de la tarea y
``NERO_SCENE_XML`` para el modelo de la IK (la misma que usa ``nero_ctrl``).

Asi los nodos funcionan con ``ros2 run`` desde cualquier directorio y sin rutas
absolutas.
"""

import os
from pathlib import Path

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)


# Escena con el cubo movil (la que debe cargar tambien el simulador):
#   ros2 launch nero_ctrl nero_can_ctrl.launch.py mode:=sim scene:=scene_nero_cubo.xml
ESCENA_TAREA = 'scene_nero_cubo.xml'

# Modelo de la IK: el unico con pinch_site.
ESCENA_IK = 'scene.xml'

# Carpetas relativas donde pueden estar los modelos, en orden de preferencia.
SUBDIRS_ESCENA = ('mujoco/agilex_arm/agilex_nero', 'src/mujoco/agilex_arm/agilex_nero',)

SUBDIRS_IK = ('models_for_simulation/agilex_nero', 'src/models_for_simulation/agilex_nero',)

# Paquetes que podrian instalar los modelos en su carpeta share/.
PAQUETES = ('grupo2_pkg', 'agx_arm_description',)


def _share(paquete):
    try:
        return get_package_share_directory(paquete)
    except (PackageNotFoundError, KeyError, ValueError):
        return None


def directorios_candidatos(subdirs=SUBDIRS_ESCENA, model_dir=None):
    """Carpetas donde buscar un modelo, en orden de prioridad (sin repetidos)."""
    candidatos = []

    if model_dir:
        candidatos.append(Path(model_dir).expanduser())

    entorno = os.environ.get('NERO_MUJOCO_DIR')
    if entorno:
        candidatos.append(Path(entorno).expanduser())

    for paquete in PAQUETES:
        share = _share(paquete)
        if share:
            for subdir in subdirs:
                candidatos.append(Path(share) / subdir)

    for padre in Path(__file__).resolve().parents:
        for subdir in subdirs:
            candidatos.append(padre / subdir)

    # Eliminar duplicados conservando el orden.
    unicos = []
    vistos = set()
    for ruta in candidatos:
        clave = str(ruta)
        if clave not in vistos:
            vistos.add(clave)
            unicos.append(ruta)

    return unicos


def _buscar(nombre, subdirs, model_dir=None, pista='', env_archivo=None):
    """Primer <carpeta candidata>/<nombre> que exista, o FileNotFoundError claro."""
    if env_archivo:
        env = os.environ.get(env_archivo)
        if env:
            ruta = Path(env).expanduser()
            if ruta.is_file():
                return ruta.resolve()

    carpetas = directorios_candidatos(subdirs, model_dir)

    for carpeta in carpetas:
        ruta = carpeta / nombre
        if ruta.is_file():
            return ruta.resolve()

    raise FileNotFoundError(
        'No se encontro el modelo MuJoCo "{}". Carpetas probadas:\n  {}\n'
        'Define NERO_MUJOCO_DIR=<carpeta>, pasa -p model_dir:=<carpeta> o '
        'apunta {}=<archivo>.{}'.format(
            nombre,
            '\n  '.join(str(c) for c in carpetas),
            env_archivo or 'NERO_ESCENA_CUBO',
            pista,
        )
    )


def encontrar_escena(nombre=ESCENA_TAREA, model_dir=None):
    """Ruta absoluta de la escena de la tarea (brazo + cubo movil)."""
    return _buscar(
        nombre,
        SUBDIRS_ESCENA,
        model_dir,
        env_archivo='NERO_ESCENA_CUBO',
        pista=(
            '\nLa escena con el cubo es {} y vive junto a nero_arm.xml y assets/.'
        ).format(ESCENA_TAREA),
    )


def encontrar_modelo_ik(nombre=ESCENA_IK, model_dir=None):
    """Ruta absoluta del modelo que usa IK_node.py (el que define pinch_site)."""
    return _buscar(
        nombre,
        SUBDIRS_IK,
        model_dir,
        env_archivo='NERO_SCENE_XML',
        pista='\nEs el modelo de IK_node.py, el unico con el site pinch_site.',
    )


def resolver_modelo(ruta, base=None):
    """Normaliza la ruta recibida por /model_path.

    Si es relativa se resuelve contra ``base`` (por defecto, el directorio de
    trabajo). Lanza FileNotFoundError si el archivo no existe.
    """
    modelo = Path(ruta).expanduser()

    if not modelo.is_absolute() and base is not None:
        modelo = Path(base) / modelo

    modelo = modelo.resolve()

    if not modelo.is_file():
        raise FileNotFoundError('No se encontro el modelo: {}'.format(modelo))

    return modelo
