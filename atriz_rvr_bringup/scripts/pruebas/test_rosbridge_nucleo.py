"""Quién puede abrir rosbridge — los caminos NEGATIVOS, sin tornado y sin ROS.

    python3 -m pytest atriz_rvr_bringup/scripts/pruebas/ -q

Se prueban aquí y no contra el robot porque son justo lo que un robot delante no
deja recorrer: fabricar un testigo de otro robot, o llegar sin ninguno.

🔴 Y la más importante es `elegir_subprotocolo`, por una razón medida: el mismo
   fallo en el agente del Taller convertía un cierre `4401` en un **HTTP 500**, y
   su prueba contra el doble estaba EN VERDE porque el doble no ejecuta el
   `assert` de tornado (evidencia 120).
"""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from rosbridge_nucleo import (  # noqa: E402
    CIERRE_OTRO_ROBOT, CIERRE_RELOJ, CIERRE_SIN_TESTIGO, CIERRE_TESTIGO_MALO,
    EXENTOS, PREFIJO_TESTIGO, SUBPROTOCOLO, Decision, decidir,
    elegir_subprotocolo, extraer_testigo,
)


@dataclass
class VeredictoFalso:
    """La forma de `atriz_testigo.Veredicto`, sin firmar nada."""

    valido: bool
    codigo: int | None = None
    motivo: str = ''
    sujeto: str = ''


def _acepta(sujeto='ana'):
    return lambda _crudo: VeredictoFalso(True, sujeto=sujeto)


def _rechaza(codigo, motivo='no', sujeto=''):
    return lambda _crudo: VeredictoFalso(False, codigo, motivo, sujeto)


# ─────────────────────────────────────────────────────────────────────────────
# elegir_subprotocolo — la que evita el HTTP 500
# ─────────────────────────────────────────────────────────────────────────────

def test_sin_ofrecer_nada_no_se_elige_nada():
    """Si el cliente no ofrece subprotocolos, tornado exige `None`."""
    assert elegir_subprotocolo([]) is None


def test_si_ofrece_el_nuestro_se_elige_el_nuestro():
    assert elegir_subprotocolo([f'{PREFIJO_TESTIGO}abc', SUBPROTOCOLO]) == SUBPROTOCOLO


def test_si_no_ofrece_el_nuestro_se_elige_uno_SUYO():
    """Nunca uno inventado: el `assert` de tornado da HTTP 500."""
    assert elegir_subprotocolo(['otro.cosa']) == 'otro.cosa'


@pytest.mark.parametrize('ofrecidos', [
    [],
    [SUBPROTOCOLO],
    ['otro.cosa'],
    [f'{PREFIJO_TESTIGO}xyz'],
    [f'{PREFIJO_TESTIGO}xyz', SUBPROTOCOLO],
    ['a', 'b', 'c'],
])
def test_LA_INVARIANTE_lo_elegido_SIEMPRE_lo_ofrecio_el_cliente(ofrecidos):
    """La propiedad que impide que vuelva el fallo, sobre todos los casos.

    Es la comprobación que hay que romper para que el HTTP 500 reaparezca.
    """
    elegido = elegir_subprotocolo(ofrecidos)
    assert elegido is None or elegido in ofrecidos


# ─────────────────────────────────────────────────────────────────────────────
# extraer_testigo
# ─────────────────────────────────────────────────────────────────────────────

def test_saca_el_testigo_del_subprotocolo():
    assert extraer_testigo([SUBPROTOCOLO, f'{PREFIJO_TESTIGO}eyJhbG.xxx']) == 'eyJhbG.xxx'


def test_sin_prefijo_no_hay_testigo():
    """Un subprotocolo cualquiera NO se confunde con un testigo."""
    assert extraer_testigo([SUBPROTOCOLO, 'eyJhbG.xxx']) is None


# ─────────────────────────────────────────────────────────────────────────────
# decidir — los cuatro rechazos, que son el producto
# ─────────────────────────────────────────────────────────────────────────────

def test_sin_testigo_se_cierra_con_4401_Y_CON_MOTIVO():
    d = decidir([SUBPROTOCOLO], '192.168.1.2', verificar=_acepta())
    assert d.admitir is False
    assert d.codigo == CIERRE_SIN_TESTIGO
    #: 🔴 El motivo NO es decorativo. Este proyecto tiene medido lo que cuesta un
    #:    rechazo mudo: el alumno va a buscar al profesor en vez de mirar.
    assert 'web' in d.motivo


def test_sin_testigo_NO_se_llama_al_verificador():
    """Control: no se verifica lo que no ha llegado."""
    def explota(_crudo):
        raise AssertionError('no debería haberse llamado')
    d = decidir([SUBPROTOCOLO], '192.168.1.2', verificar=explota)
    assert d.codigo == CIERRE_SIN_TESTIGO


def test_testigo_de_OTRO_robot_se_cierra_con_4404():
    d = decidir([f'{PREFIJO_TESTIGO}x'], '192.168.1.2',
                verificar=_rechaza(CIERRE_OTRO_ROBOT, 'es para el robot 3', 'ana'))
    assert (d.admitir, d.codigo, d.sujeto) == (False, CIERRE_OTRO_ROBOT, 'ana')


def test_firma_mala_se_cierra_con_4403():
    d = decidir([f'{PREFIJO_TESTIGO}x'], '192.168.1.2',
                verificar=_rechaza(CIERRE_TESTIGO_MALO))
    assert d.codigo == CIERRE_TESTIGO_MALO


def test_reloj_sin_sincronizar_se_cierra_con_1013_y_NO_con_403():
    """La Pi no tiene RTC: «aún no sé la hora» no es «tu testigo es malo».

    Distinguirlos es lo que hace que la web pueda decir «espera unos segundos»
    en vez de mandar a rehacer la sesión.
    """
    d = decidir([f'{PREFIJO_TESTIGO}x'], '192.168.1.2',
                verificar=_rechaza(CIERRE_RELOJ, 'aún no tengo la hora'))
    assert d.codigo == CIERRE_RELOJ


def test_testigo_bueno_ENTRA_y_se_sabe_quien_es():
    d = decidir([f'{PREFIJO_TESTIGO}x', SUBPROTOCOLO], '192.168.1.2',
                verificar=_acepta('ana'))
    assert (d.admitir, d.sujeto, d.subprotocolo) == (True, 'ana', SUBPROTOCOLO)


# ─────────────────────────────────────────────────────────────────────────────
# La exención de localhost, en las DOS direcciones
# ─────────────────────────────────────────────────────────────────────────────

@pytest.mark.parametrize('ip', EXENTOS)
def test_desde_el_propio_robot_se_entra_sin_testigo(ip):
    """Las herramientas de banco corren en la Pi y no tienen sesión de web.

    No regala nada: quien corre dentro alcanza `raw_motors` con `rclpy`.
    """
    d = decidir([], ip, verificar=_rechaza(CIERRE_TESTIGO_MALO))
    assert d.admitir is True


def test_EL_CONTROL_una_ip_parecida_NO_esta_exenta():
    """Sin esto, «exento» podría estar comparando de una forma que casa de más."""
    d = decidir([], '127.0.0.2', verificar=_rechaza(CIERRE_TESTIGO_MALO))
    assert d.admitir is False


def test_EL_CONTROL_la_ip_de_la_red_NO_esta_exenta():
    d = decidir([], '192.168.1.200', verificar=_rechaza(CIERRE_TESTIGO_MALO))
    assert d.admitir is False


def test_exento_pero_el_subprotocolo_se_sigue_eligiendo_bien():
    """Un cliente local que ofrece algo también necesita su respuesta válida."""
    d = decidir(['otro.cosa'], '127.0.0.1', verificar=_acepta())
    assert d.admitir is True
    assert d.subprotocolo == 'otro.cosa'


# ─────────────────────────────────────────────────────────────────────────────
# Y que los códigos NO se hayan separado del otro repositorio
# ─────────────────────────────────────────────────────────────────────────────

def test_los_codigos_coinciden_con_atriz_testigo():
    """Dos copias de una constante se separan en silencio.

    ⚠️ Si `atriz_testigo` no está (es del repositorio de migración), esto SALTA
       y lo dice — pero el lanzador NO salta: se niega a arrancar. El sitio donde
       importa que coincidan es el robot, no esta prueba.
    """
    candidatos = [
        Path.home() / 'atriz_migracion/scripts',
        Path(__file__).resolve().parents[3].parent / 'atriz_migracion/scripts',
    ]
    ruta = next((str(c) for c in candidatos if (c / 'atriz_testigo.py').is_file()), None)
    if ruta is None:
        pytest.skip(
            'no encuentro atriz_testigo.py (repositorio de migración). '
            f'He mirado en: {[str(c) for c in candidatos]}'
        )
    sys.path.insert(0, ruta)
    import atriz_testigo as at

    assert (CIERRE_RELOJ, CIERRE_SIN_TESTIGO, CIERRE_TESTIGO_MALO, CIERRE_OTRO_ROBOT) == \
           (at.CIERRE_RELOJ, at.CIERRE_SIN_TESTIGO, at.CIERRE_TESTIGO_MALO, at.CIERRE_OTRO_ROBOT)
    assert PREFIJO_TESTIGO == at.PREFIJO_TESTIGO
    assert SUBPROTOCOLO == at.SUBPROTOCOLO


def test_Decision_es_inmutable():
    """Para que nadie 'arregle' un rechazo cambiándolo sobre la marcha."""
    d = Decision(False, CIERRE_SIN_TESTIGO, 'no')
    with pytest.raises(Exception):
        d.admitir = True            # type: ignore[misc]
