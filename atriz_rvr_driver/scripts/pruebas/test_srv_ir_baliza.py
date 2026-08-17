"""`/set_ir_baliza`: la baliza IR que POR CONSTRUCCIÓN no puede conducir.

    python3 -m pytest atriz_rvr_driver/scripts/pruebas/ -q

Encargo del PC (F5, 2026-08-16): la web quiere dejar un robot emitiendo como
baliza continua, pero `broadcasting` y `following` viajan en el MISMO
`set_ir_mode` con `mode` como cadena libre, y la lista blanca de rosbridge
filtra por servicio, no por argumento — abrir ese servicio abriría también lo
que conduce. El servicio nuevo resuelve el dilema en el tipo: su petición es
`bool encender`, así que no existe la cadena con la que pedir `following`.

Estas pruebas fijan el contrato: `_srv_ir_baliza` DELEGA todo en
`_srv_ir_modo` (validación de rango, contabilidad de `/estado_ir`, «off apaga
las tres cosas») y el modo que le pasa solo puede ser `broadcasting` u `off`.

🔴 Qué las refutaría: un `_srv_ir_baliza` capaz de producir `following` o
   `evading`; o uno que no delegue — una validación duplicada aquí acabaría
   divergiendo de la de `_srv_ir_modo`, que es la que está medida.
"""

from __future__ import annotations

import sys
from pathlib import Path

RAIZ = Path(__file__).resolve().parents[1]          # .../atriz_rvr_driver/scripts
sys.path.insert(0, str(RAIZ))

from atriz_rvr_msgs.srv import SetIRBaliza  # noqa: E402
from atriz_rvr_driver.rvr_driver_node import RvrDriverNode  # noqa: E402


class _NodoFalso:
    """Registra la delegación; no hay RVR ni ROS detrás.

    El método se invoca SIN enlazar (`RvrDriverNode._srv_ir_baliza(falso, ...)`)
    para probar el mapeo sin construir el nodo entero — el camino completo,
    con la validación real de `_srv_ir_modo`, se comprueba en vivo contra el
    robot (VALIDAR §2ter).
    """

    def __init__(self):
        self.delegadas = []

    def _srv_ir_modo(self, req, resp):
        self.delegadas.append(req)
        resp.success = True
        resp.message = f'delegado modo={req.mode}'
        return resp


def _llamar(encender, far=0, near=0):
    nodo = _NodoFalso()
    req = SetIRBaliza.Request()
    req.encender = encender
    req.far_code = far
    req.near_code = near
    resp = RvrDriverNode._srv_ir_baliza(nodo, req, SetIRBaliza.Response())
    return nodo, resp


def test_encender_delega_en_broadcasting_con_los_codigos():
    nodo, resp = _llamar(True, far=3, near=5)
    assert len(nodo.delegadas) == 1
    peticion = nodo.delegadas[0]
    assert peticion.mode == 'broadcasting'
    assert (peticion.far_code, peticion.near_code) == (3, 5)
    assert resp.success and 'broadcasting' in resp.message


def test_apagar_delega_en_off_e_ignora_los_codigos():
    # far/near basura con encender=False: apagar NUNCA puede fallar por un
    # código inválido, porque `_srv_ir_modo` valida el rango salvo en 'off' y
    # aquí ni siquiera se copian.
    nodo, resp = _llamar(False, far=255, near=255)
    peticion = nodo.delegadas[0]
    assert peticion.mode == 'off'
    assert (peticion.far_code, peticion.near_code) == (0, 0)
    assert resp.success


def test_no_existe_forma_de_pedir_following():
    # La propiedad de seguridad entera, barrida: para CUALQUIER petición
    # expresable con el tipo, el modo delegado cae en {broadcasting, off}.
    for encender in (True, False):
        for codigo in (0, 7, 255):
            nodo, _ = _llamar(encender, far=codigo, near=codigo)
            assert nodo.delegadas[0].mode in ('broadcasting', 'off')
