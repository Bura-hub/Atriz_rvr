"""`/set_ir_conduccion`: seguir/huir CON PLAZO — la petición «para siempre» no existe.

    python3 -m pytest atriz_rvr_driver/scripts/pruebas/ -q

Encargo del PC (2026-08-17, noche): `following` y `evading` conducen el robot
desde el FIRMWARE, sin pasar por `cmd_vel` — ni el watchdog ni el
`collision_monitor` los ven. La identidad de la Fase B dice quién responde,
pero no quita el peligro: lo que lo reduce es que NO PUEDA QUEDARSE ENCENDIDO.
Por eso el servicio exige `segundos` con tope, y un temporizador de un disparo
lo apaga solo. Es la pieza que justifica el servicio entero.

Contrato fijado por estas pruebas (espejo de `conduccion_ir.ts` del PC):
  · modo entero, NO cadena: 0=off · 1=seguir · 2=huir. Y 0 es off A PROPÓSITO:
    un campo que falte llega como 0, y 0 tiene que ser APAGAR.
  · rechazar, no recortar — recortar 120 s a 30 pondría el robot a conducir un
    tiempo que nadie pidió.
  · apagar no valida nada y no puede fallar por códigos basura.
  · rearmar, no acumular: dos peticiones de 5 s apagan a los ~5 s de la
    SEGUNDA, no a los 10.
  · la parada de emergencia cancela el plazo pendiente.

🔴 Qué las refutaría: un manejador que acepte `segundos` fuera de (0, TOPE];
   uno que recorte en vez de rechazar; uno que deje DOS temporizadores vivos
   tras un rearme; o una parada que deje un `stop` diferido armado.
"""

from __future__ import annotations

import math
import sys
import threading
from pathlib import Path

RAIZ = Path(__file__).resolve().parents[1]          # .../atriz_rvr_driver/scripts
sys.path.insert(0, str(RAIZ))

from atriz_rvr_msgs.srv import SetIRConduccion  # noqa: E402
from atriz_rvr_driver.rvr_driver_node import RvrDriverNode  # noqa: E402

TOPE = SetIRConduccion.Request.TOPE_SEGUNDOS


class _TemporizadorFalso:
    def __init__(self, periodo, cb):
        self.periodo, self.cb = periodo, cb
        self.cancelado = False
        self.destruido = False

    def cancel(self):
        self.cancelado = True


class _RvrFalso:
    """Los `stop_*` devuelven un centinela para que `_enviar` los registre."""

    def stop_robot_to_robot_infrared_following(self):
        return 'stop_following'

    def stop_robot_to_robot_infrared_evading(self):
        return 'stop_evading'

    def drive_stop(self):
        return 'drive_stop'


class _LogFalso:
    def error(self, *a, **k):
        pass

    warn = info = error


class _NodoFalso:
    """Solo lo que el manejador toca. El camino completo se comprueba en vivo."""

    def __init__(self):
        self.delegadas = []          # (nombre, req)
        self.enviados = []           # etiquetas de _enviar
        self.timers = []             # todos los creados, vivos o no
        self._lock = threading.Lock()
        self._rvr = _RvrFalso()
        self._ir_plazo = None
        self._ir_plazo_gen = 0
        self._ir_modo = 'off'
        self._ir_conduciendo = False
        self._parada_emergencia = False
        self._conduciendo = False

    # — delegados —
    def _srv_ir_modo(self, req, resp):
        self.delegadas.append(('ir_modo', req))
        resp.success, resp.message = True, f'delegado modo={req.mode}'
        return resp

    def _srv_ir_evasion(self, req, resp):
        self.delegadas.append(('ir_evasion', req))
        resp.success, resp.message = True, 'delegado evasion'
        return resp

    # — infraestructura que usa el manejador —
    def create_timer(self, periodo, cb, callback_group=None):
        t = _TemporizadorFalso(periodo, cb)
        self.timers.append(t)
        return t

    def destroy_timer(self, t):
        t.destruido = True

    def _enviar(self, comando, etiqueta):
        self.enviados.append((comando, etiqueta))

    def get_logger(self):
        return _LogFalso()

    # — métodos reales bajo prueba, sin enlazar —
    _srv_ir_conduccion = RvrDriverNode._srv_ir_conduccion
    _cancelar_plazo_ir = RvrDriverNode._cancelar_plazo_ir
    _vencer_plazo_ir = RvrDriverNode._vencer_plazo_ir
    _g_plazo_ir = None


def _llamar(nodo, modo, far=0, near=0, segundos=0.0):
    req = SetIRConduccion.Request()
    req.modo, req.far_code, req.near_code = modo, far, near
    req.segundos = float(segundos)
    return nodo._srv_ir_conduccion(req, SetIRConduccion.Response())


def _vivos(nodo):
    return [t for t in nodo.timers if not t.cancelado and not t.destruido]


def test_seguir_delega_en_following_y_arma_el_plazo():
    nodo = _NodoFalso()
    resp = _llamar(nodo, 1, far=3, near=5, segundos=5.0)
    assert resp.success
    nombre, peticion = nodo.delegadas[0]
    assert nombre == 'ir_modo' and peticion.mode == 'following'
    assert (peticion.far_code, peticion.near_code) == (3, 5)
    (timer,) = _vivos(nodo)
    assert timer.periodo == 5.0


def test_huir_delega_en_evasion_y_arma_el_plazo():
    nodo = _NodoFalso()
    resp = _llamar(nodo, 2, far=1, near=2, segundos=8.0)
    assert resp.success
    nombre, peticion = nodo.delegadas[0]
    assert nombre == 'ir_evasion'
    assert (peticion.far_code, peticion.near_code) == (1, 2)
    assert _vivos(nodo)[0].periodo == 8.0


def test_apagar_no_valida_cancela_el_plazo_y_delega_en_off():
    nodo = _NodoFalso()
    _llamar(nodo, 1, far=0, near=0, segundos=5.0)
    resp = _llamar(nodo, 0, far=255, near=255, segundos=9999.0)
    assert resp.success
    assert nodo.delegadas[-1][1].mode == 'off'
    assert _vivos(nodo) == []


def test_rechaza_no_recorta_el_plazo_fuera_de_rango():
    # 🔴 Recortar 120 s a 30 sería conducir un tiempo que nadie pidió. Y NaN
    #    tiene que caer aquí: es la trampa de `limitar(nan)` (2026-08-03).
    for malo in (0.0, -1.0, TOPE + 1.0, float('nan'), math.inf):
        nodo = _NodoFalso()
        resp = _llamar(nodo, 1, segundos=malo)
        assert not resp.success, f'aceptó segundos={malo}'
        assert str(TOPE) in resp.message      # el motivo nombra el tope
        assert nodo.delegadas == [] and _vivos(nodo) == []
    # y el tope EXACTO sí pasa
    nodo = _NodoFalso()
    assert _llamar(nodo, 1, segundos=float(TOPE)).success


def test_modo_desconocido_se_rechaza():
    nodo = _NodoFalso()
    resp = _llamar(nodo, 3, segundos=5.0)
    assert not resp.success and nodo.delegadas == []


def test_rearmar_no_acumular():
    nodo = _NodoFalso()
    _llamar(nodo, 1, segundos=5.0)
    _llamar(nodo, 1, segundos=7.0)
    vivos = _vivos(nodo)
    assert len(vivos) == 1 and vivos[0].periodo == 7.0
    assert nodo.timers[0].cancelado


def test_al_vencer_apaga_los_dos_modos_y_anota_off():
    nodo = _NodoFalso()
    _llamar(nodo, 1, segundos=5.0)
    nodo._ir_modo, nodo._ir_conduciendo = 'following', True
    _vivos(nodo)[0].cb()                      # vence el plazo
    etiquetas = [c for c, _ in nodo.enviados]
    assert 'stop_following' in etiquetas and 'stop_evading' in etiquetas
    assert nodo._ir_modo == 'off' and nodo._ir_conduciendo is False
    assert _vivos(nodo) == []


def test_un_vencimiento_rancio_no_apaga_la_peticion_nueva():
    # El rearme y el vencimiento corren en grupos distintos: si el viejo
    # dispara en la carrera, la generación lo delata y NO toca nada.
    nodo = _NodoFalso()
    _llamar(nodo, 1, segundos=5.0)
    viejo = nodo.timers[0]
    _llamar(nodo, 1, segundos=7.0)
    nodo._ir_modo = 'following'
    viejo.cb()                                # dispara el RANCIO
    assert nodo.enviados == [] and nodo._ir_modo == 'following'
    assert _vivos(nodo)[0].periodo == 7.0     # el nuevo sigue armado


def test_la_parada_de_emergencia_cancela_el_plazo():
    # Requisito 5 del encargo: sin esto quedaría un `stop` diferido armado
    # sobre un robot ya parado. Se prueba con el MANEJADOR REAL de la parada.
    nodo = _NodoFalso()
    _llamar(nodo, 1, segundos=5.0)
    RvrDriverNode._cb_parada_emergencia(nodo, None)
    assert _vivos(nodo) == []
    assert nodo._parada_emergencia is True and nodo._ir_modo == 'off'


def test_la_constante_del_srv_es_la_que_la_web_copia():
    # El PC copió TOPE_SEGUNDOS=30 a mano y va a atar su constante a ESTA.
    assert TOPE == 30
