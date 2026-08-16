#!/usr/bin/env python3
"""rosbridge que EXIGE un testigo — Fase B de `SEGURIDAD_ROSBRIDGE.md`.

No es un servidor nuevo: es el nodo de rosbridge de siempre, con su `open`
parcheado antes de arrancar. Se lanza igual que antes y escucha en el mismo
puerto; lo único que cambia es quién puede abrir la conexión.

    Node(package='atriz_rvr_bringup', executable='atriz_rosbridge.py', ...)

El porqué de esta forma —y por qué NO es un proxy ni una subclase— está en
`rosbridge_nucleo.py`, que además es donde vive la decisión y sus 24 pruebas.

═══════════════════════════════════════════════════════════════════════════════
🔴 QUÉ PASA SI ALGO FALTA: SE NIEGA A ARRANCAR, Y LO DICE
═══════════════════════════════════════════════════════════════════════════════
Este guion **falla cerrado**, al revés que el vigía de DDS o las esperas de
`atriz-robot.sh`, que fallan abiertos a propósito. La diferencia es qué protege
cada uno: un vigía que se equivoca deja un robot inservible, y eso es peor que el
fallo que evita; **un control de acceso que se salta solo no es un control**, es
la familia de «configuración que existe y no hace nada» que este proyecto
persigue —el `chmod` sobre vfat, el `usercfg.txt` de 24.04, el drop-in con el
nombre que pierde el orden—.

⚠️ Y como fallar cerrado deja al robot **inalcanzable desde la web**, y rosbridge
   deniega EN SILENCIO, hay una salida de emergencia explícita:

       ATRIZ_ROSBRIDGE_SIN_TESTIGO=1   -> arranca el nodo ORIGINAL, sin testigo

   Existe para no tener que entrar por SSH a 16 robots si el parche se rompe.
   `verificar_robot.sh` da **FALLO** si la encuentra puesta: una salida de
   emergencia que nadie ve es una puerta trasera.
"""

from __future__ import annotations

import os
import runpy
import sys
from pathlib import Path

#: El nodo de verdad. Se ejecuta, no se copia: copiar 300 líneas de un paquete de
#: terceros las deja rancias en la primera actualización de ROS.
NODO_ORIGINAL = Path(
    os.environ.get(
        'ATRIZ_ROSBRIDGE_NODO',
        '/opt/ros/jazzy/lib/rosbridge_server/rosbridge_websocket.py',
    )
)


def _morir(*lineas: str) -> None:
    for i, l in enumerate(lineas):
        print(('🔴 ' if i == 0 else '   ') + l, file=sys.stderr)
    sys.exit(1)


def _correr_original() -> None:
    if not NODO_ORIGINAL.is_file():
        _morir(
            f'no encuentro el nodo de rosbridge en {NODO_ORIGINAL}',
            'Se instala con `ros-jazzy-rosbridge-suite`. Si tu ROS está en otro',
            'sitio, pon ATRIZ_ROSBRIDGE_NODO con la ruta.',
        )
    runpy.run_path(str(NODO_ORIGINAL), run_name='__main__')


# ═══════════════════════════════════════════════════════════════════════════
# La salida de emergencia, antes que nada
# ═══════════════════════════════════════════════════════════════════════════
if os.environ.get('ATRIZ_ROSBRIDGE_SIN_TESTIGO') == '1':
    print(
        '⚠️  ATRIZ_ROSBRIDGE_SIN_TESTIGO=1: rosbridge arranca SIN pedir testigo.\n'
        '    Cualquiera en la red puede teleoperar este robot. Quítalo en cuanto\n'
        '    puedas: `verificar_robot.sh` dará FALLO mientras siga puesto.',
        file=sys.stderr,
    )
    _correr_original()
    sys.exit(0)


# ═══════════════════════════════════════════════════════════════════════════
# 1 · La decisión (al lado de este fichero, instalada por CMakeLists)
# ═══════════════════════════════════════════════════════════════════════════
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    import rosbridge_nucleo as nucleo
except ImportError as e:                                        # pragma: no cover
    _morir(f'no encuentro `rosbridge_nucleo` al lado de este guion: {e}')


# ═══════════════════════════════════════════════════════════════════════════
# 2 · El verificador de firmas, que vive en el OTRO repositorio
# ═══════════════════════════════════════════════════════════════════════════
# Se importa en vez de copiarse, por la misma razón que en el agente del Taller:
# dos copias de un verificador de firmas se separan en silencio, y el día que
# pase el síntoma sería «este robot no deja entrar a nadie», buscado en el sitio
# equivocado.
# 📝 Sin repetidos: el mensaje de error los enseña, y listar dos veces el mismo
#    sitio parece un fallo del propio guion y resta credibilidad al aviso.
_CANDIDATOS = list(dict.fromkeys([
    Path(os.environ.get('ATRIZ_MIGRACION', str(Path.home() / 'atriz_migracion'))) / 'scripts',
    Path.home() / 'atriz_migracion/scripts',
]))
for _c in _CANDIDATOS:
    if (_c / 'atriz_testigo.py').is_file():
        sys.path.insert(0, str(_c))
        break
try:
    import atriz_testigo as testigo
except ImportError:
    _morir(
        'no encuentro `atriz_testigo`, que vive en el repositorio de migración.',
        'He mirado en: ' + ', '.join(str(c) for c in _CANDIDATOS),
        'Clónalo, o pon ATRIZ_MIGRACION. Sin él no se pueden verificar los',
        'testigos de la web, y arrancar sin verificarlos sería peor que no',
        'arrancar. Para salir del paso: ATRIZ_ROSBRIDGE_SIN_TESTIGO=1',
    )

# 🔴 Y que las dos copias de las constantes NO se hayan separado. Es barato y
#    cierra la única grieta que deja repetirlas: un código de cierre distinto en
#    cada lado daría un rechazo que la web no sabría interpretar.
_MIAS = (nucleo.CIERRE_RELOJ, nucleo.CIERRE_SIN_TESTIGO,
         nucleo.CIERRE_TESTIGO_MALO, nucleo.CIERRE_OTRO_ROBOT,
         nucleo.PREFIJO_TESTIGO, nucleo.SUBPROTOCOLO)
_SUYAS = (testigo.CIERRE_RELOJ, testigo.CIERRE_SIN_TESTIGO,
          testigo.CIERRE_TESTIGO_MALO, testigo.CIERRE_OTRO_ROBOT,
          testigo.PREFIJO_TESTIGO, testigo.SUBPROTOCOLO)
if _MIAS != _SUYAS:
    _morir(
        'las constantes del testigo se han separado entre los dos repositorios.',
        f'  rosbridge_nucleo.py: {_MIAS}',
        f'  atriz_testigo.py:    {_SUYAS}',
        'Alinéalas antes de seguir: con códigos distintos, la web recibiría un',
        'cierre que no sabe interpretar y lo pintaría como «no responde».',
    )


# ═══════════════════════════════════════════════════════════════════════════
# 3 · La identidad de este robot y la clave pública de la web
# ═══════════════════════════════════════════════════════════════════════════
# 🔴 Sin número no se arranca. El testigo lleva `rob` dentro y esto lo compara
#    con el suyo: con el número equivocado rechazaría a TODO EL MUNDO, y con uno
#    inventado dejaría entrar a quien no toca. Mismo criterio que `atriz-agente.sh`.
_ID = os.environ.get('ATRIZ_ROBOT_ID', '').strip()
if not _ID.isdigit():
    _morir(
        f'ATRIZ_ROBOT_ID no es un número (vale {_ID!r}).',
        'Lo pone /etc/profile.d/atriz-robot.sh, que instala `fase_7_systemd.sh --id NN`.',
        'Sin él no se puede comprobar que un testigo es PARA ESTE robot, que es',
        'el requisito 1 entero de SEGURIDAD_ROSBRIDGE.md.',
    )
ROBOT = int(_ID)

_RUTA_CLAVE = Path(os.environ.get('ATRIZ_CLAVE_PUB', '/etc/atriz/testigo.pub'))
if not _RUTA_CLAVE.is_file():
    _morir(
        f'no está la clave pública en {_RUTA_CLAVE}.',
        'Es la mitad pública de la que firma el servidor de la web. Se saca con:',
        '    node herramientas/publicar_clave.mjs   (en el PC, dentro de atriz-lab)',
        'y se instala con `sudo tee /etc/atriz/testigo.pub`.',
    )
try:
    from cryptography.hazmat.primitives.serialization import load_pem_public_key
    CLAVE = load_pem_public_key(_RUTA_CLAVE.read_bytes())
except Exception as e:                                          # noqa: BLE001
    _morir(f'no puedo leer la clave pública de {_RUTA_CLAVE}: {e}')


# ═══════════════════════════════════════════════════════════════════════════
# 4 · El parche
# ═══════════════════════════════════════════════════════════════════════════
try:
    from rosbridge_server import RosbridgeWebSocket
except ImportError as e:                                        # pragma: no cover
    _morir(f'no encuentro `rosbridge_server`: {e}')

_open_original = RosbridgeWebSocket.open


def _log():
    """El logger del nodo, si ya está puesto. No es seguro que lo esté."""
    nodo = getattr(RosbridgeWebSocket, 'node_handle', None)
    return nodo.get_logger() if nodo is not None else None


def _select_subprotocol(self, subprotocolos):
    """Decide AQUÍ, que es donde tornado da la lista ofrecida.

    tornado llama a esto durante el apretón y **antes** de `open`, así que la
    decisión se calcula aquí y se guarda; `open` solo la ejecuta.

    🔴 Lo que se devuelve tiene que estar en `subprotocolos`: tornado hace
       `assert self.selected_subprotocol in subprotocols`, y saltárselo da un
       HTTP 500 en vez del cierre con motivo (evidencia 120).
    """
    ofrecidos = [s.strip() for s in (subprotocolos or []) if s.strip()]
    self._atriz = nucleo.decidir(
        ofrecidos,
        self.request.remote_ip or '',
        verificar=lambda crudo: testigo.verificar(crudo, ROBOT, CLAVE),
    )
    return self._atriz.subprotocolo


def _open_con_testigo(self, *args, **kwargs):
    d = getattr(self, '_atriz', None)

    if d is None:
        # No debería pasar: tornado siempre llama a select_subprotocol antes.
        # Si pasa, se cierra — fallar cerrado también aquí.
        log = _log()
        if log:
            log.error('rosbridge: conexión sin decisión previa; se rechaza')
        self.close(nucleo.CIERRE_TESTIGO_MALO, 'no se pudo comprobar el testigo')
        return

    if not d.admitir:
        log = _log()
        if log:
            log.warning(
                f'rosbridge: RECHAZADA la conexión de {self.request.remote_ip} '
                f'[{d.sujeto or "sin identificar"}] · {d.codigo} · {d.motivo}'
            )
        # 🔴 CON CÓDIGO Y MOTIVO. Un cierre mudo manda al alumno a buscar al
        #    profesor en vez de a leer la pantalla, y este proyecto lo tiene
        #    medido.
        self.close(d.codigo, d.motivo)
        return

    _open_original(self, *args, **kwargs)
    log = _log()
    if log:
        # Quién, no solo cuántos. Hoy no hay forma de responder a «¿quién movió
        # el robot 7 a las 11:40?», y con 16 robots y una clase eso hace falta.
        log.info(f'rosbridge: admitido {d.sujeto} desde {self.request.remote_ip}')


def _rechazada(self) -> bool:
    """¿Esta conexión NO llegó a abrirse de verdad?"""
    d = getattr(self, '_atriz', None)
    return d is None or not d.admitir


def _on_close_con_testigo(self):
    """🔴 SIN ESTO, CADA RECHAZO DEJA UNA TRAZA ENTERA EN EL JOURNAL.

    Medido contra rvr-01 el 2026-08-15. `websocket_handler.py:192` hace

        self.protocol.outgoing = lambda *_args, **_kwargs: None

    y `self.protocol` **solo existe si el `open` original llegó a correr**. Al
    rechazar no corre, así que cerrar disparaba

        AttributeError: 'RosbridgeWebSocket' object has no attribute 'protocol'

    …y con él un «Uncaught exception GET /» de tornado por cada intento. Dos
    consecuencias, las dos malas: el journal se llena —lo mismo que costó A11— y
    **el cliente recibe un 1006 en vez del código y el motivo**, o sea que el
    rechazo con explicación se convierte en un corte mudo, que es justo lo que
    todo este diseño existe para evitar.

    📝 Y es de la misma familia que el `AssertionError` de `select_subprotocol`:
       parchear la puerta de entrada de una clase obliga a mirar **todo lo que
       daba por hecho que la puerta se había abierto**.
    """
    if _rechazada(self):
        return
    _on_close_original(self)


def _on_message_con_testigo(self, message):
    """Mismo caso con `self.incoming_queue`, que tampoco existe si no se abrió.

    Un cliente puede mandar datos entre el apretón y el cierre; sin esto, esos
    bytes acaban en `incoming_queue.push` y revientan igual.
    """
    if _rechazada(self):
        return
    _on_message_original(self, message)


_on_close_original = RosbridgeWebSocket.on_close
_on_message_original = RosbridgeWebSocket.on_message

RosbridgeWebSocket.select_subprotocol = _select_subprotocol
RosbridgeWebSocket.open = _open_con_testigo
RosbridgeWebSocket.on_close = _on_close_con_testigo
RosbridgeWebSocket.on_message = _on_message_con_testigo

print(
    f'[atriz-rosbridge] testigo EXIGIDO · robot {ROBOT} · clave {_RUTA_CLAVE} · '
    f'exentos {", ".join(nucleo.EXENTOS)}',
    file=sys.stderr,
)

_correr_original()
