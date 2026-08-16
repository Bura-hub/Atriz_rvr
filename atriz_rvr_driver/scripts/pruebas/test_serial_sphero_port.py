"""El puerto serie del SDK ante `connection_lost` — la evidencia 126, en banco.

    python3 -m pytest atriz_rvr_driver/scripts/pruebas/ -q

El 2026-08-15 el puerto de rvr-01 murió a las 23:01:45 y el SDK se tragó la
excepción con un `pass`: 14 minutos de «sin señal de vida» con el robot sano y
la causa perdida para siempre. Estas pruebas usan un PTY como dispositivo serie
de verdad (serial.Serial lo abre igual que a /dev/ttyAMA0), así que ejercitan
el código real, no un doble del transporte.

🔴 Qué las refutaría: que pasen con el `pass` original. Se comprobó: con él,
   la 1 y la 2 FALLAN (no hay log y no hay reapertura).
"""

from __future__ import annotations

import asyncio
import logging
import os
import pty
import sys
import time
from pathlib import Path

import pytest

RAIZ = Path(__file__).resolve().parents[1]          # .../atriz_rvr_driver/scripts
sys.path.insert(0, str(RAIZ))

from sphero_sdk.asyncio.server.port.serial_sphero_port import SerialSpheroPort  # noqa: E402
from sphero_sdk.asyncio.server.parser.sphero_parser_base import SpheroParserBase  # noqa: E402
from sphero_sdk.asyncio.server.handler.sphero_handler_base import SpheroHandlerBase  # noqa: E402


class _ParserNulo(SpheroParserBase):
    """La base exige la herencia (`issubclass`), no solo la forma."""
    def feed(self, data):
        pass


class _HandlerNulo(SpheroHandlerBase):
    async def message_handler(self, msg):
        pass

    async def error_handler(self, msg):
        pass


def _crear_puerto(loop):
    """Un SerialSpheroPort sobre un PTY: dispositivo serie real, sin robot."""
    maestro, esclavo = pty.openpty()
    dev = os.ttyname(esclavo)
    puerto = SerialSpheroPort(loop, 1, _ParserNulo, _HandlerNulo, dev, 115200)
    return puerto, maestro, dev


def _correr(loop, segundos):
    loop.call_later(segundos, loop.stop)
    loop.run_forever()


def test_connection_lost_registra_la_causa(capfd):
    """La excepción es la ÚNICA copia de la causa: tiene que quedar en el log.

    Se comprueba en STDERR y no con caplog a propósito: el SDK no configura
    logging, así que en producción el mensaje llega al journal por el
    `lastResort` de logging → stderr del proceso. Stderr ES el efecto.
    """
    loop = asyncio.new_event_loop()
    try:
        puerto, maestro, _ = _crear_puerto(loop)
        puerto.connection_lost(OSError(5, 'Input/output error'))
        err = capfd.readouterr().err
        assert 'Input/output error' in err and 'PERDIDO' in err, (
            f'connection_lost tiene que registrar exc (la única copia de la '
            f'causa) donde el journal lo vea. stderr: {err!r}'
        )
        puerto.close()
    finally:
        loop.close()
        os.close(maestro)


def test_connection_lost_reabre_el_puerto():
    """Tras perder la conexión, el puerto tiene que volver a abrirse SOLO."""
    loop = asyncio.new_event_loop()
    try:
        puerto, maestro, dev = _crear_puerto(loop)
        # Simula lo que hace pyserial-asyncio ante un error de E/S: PRIMERO
        # cierra el transporte, DESPUÉS entrega la excepción. El dispositivo
        # (el PTY) sigue existiendo, igual que /dev/ttyAMA0 el 2026-08-15.
        # 📝 La primera versión de esta prueba llamaba a connection_lost con el
        #    transporte aún vivo y PASABA con el `pass` original: el arnés
        #    mentía — el send() salía por el transporte viejo.
        puerto.close()
        puerto.connection_lost(OSError(5, 'Input/output error'))
        _correr(loop, 2.5)                    # da tiempo al primer reintento
        # El efecto, no la intención: escribir por el puerto reabierto tiene
        # que LLEGAR al otro extremo del PTY.
        class _Msg:
            @staticmethod
            def serialise():
                return b'\x8d\x0a'
        puerto.send(_Msg)
        _correr(loop, 0.5)
        os.set_blocking(maestro, False)
        try:
            llegado = os.read(maestro, 16)
        except BlockingIOError:
            llegado = b''
        assert llegado.startswith(b'\x8d'), (
            f'el puerto no se reabrió: no llegó nada al PTY (recibido {llegado!r})'
        )
        puerto.close()
    finally:
        loop.close()
        os.close(maestro)


def test_cierre_intencional_no_reabre():
    """`close()` del apagado ordenado NO debe resucitar el puerto."""
    loop = asyncio.new_event_loop()
    try:
        puerto, maestro, _ = _crear_puerto(loop)
        puerto.close()                         # cierre INTENCIONAL
        # pyserial-asyncio llamará a connection_lost(None) tras un close():
        puerto.connection_lost(None)
        _correr(loop, 2.5)
        # El efecto: tras el cierre intencional, NADA debe llegar al PTY.
        class _Msg:
            @staticmethod
            def serialise():
                return b'\x8d\x0a'
        try:
            puerto.send(_Msg)
        except Exception:
            pass                               # un puerto cerrado puede lanzar: vale
        _correr(loop, 0.5)
        os.set_blocking(maestro, False)
        try:
            llegado = os.read(maestro, 16)
        except BlockingIOError:
            llegado = b''
        assert llegado == b'', (
            f'un close() intencional NO debe resucitar el puerto (llegó {llegado!r})'
        )
    finally:
        loop.close()
        os.close(maestro)
