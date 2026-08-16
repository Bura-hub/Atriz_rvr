#!/usr/bin/env python3

import asyncio
import serial
import logging
from serial_asyncio import SerialTransport
from .sphero_port_base import SpheroPortBase

logger = logging.getLogger(__name__)


class SerialSpheroPort(SpheroPortBase, asyncio.Protocol):
    __slots__ = ['__loop', '__transport', '_dev', '_baud', '_reintentos_reapertura']

    def __init__(self, loop, port_id,
                 parser_factory, handler_factory, dev, baud=115200):
        """Class that moves bytes from a serial port to a Parser
            and messages to that serial port (typically from the Handler)

        Args:
            loop: asyncio event loop
            port_id: an ID for the port, used mostly by Handler
            parser_factory: Parser Class
            handler_factory: Handler Class
            dev: Serial Device
            baud: Serial Device baud rate
        """
        SpheroPortBase.__init__(self, port_id, parser_factory, handler_factory)
        asyncio.Protocol.__init__(self)
        self.__loop = loop
        self._dev = dev
        self._baud = baud
        self._reintentos_reapertura = 0
        ser = serial.Serial(dev, baud)
        self.__transport = SerialTransport(loop, self, ser)

    def connection_made(self, transport):

        self.__transport = transport

    def connection_lost(self, exc):
        """El transporte murió. Registrar POR QUÉ y reabrir el puerto solo.

        🔴 Esto era `pass`, y costó 14 minutos de robot «muerto» con el RVR
           sano (2026-08-15, evidencia 126 de atriz_migracion): pyserial-asyncio
           CIERRA el puerto ante un error de E/S y entrega aquí la excepción —
           la ÚNICA copia de la causa—. Con `pass`, la causa se perdía para
           siempre, el puerto no se reabría nunca, y cada wake/stop/start
           posterior escribía a un transporte muerto mientras el driver
           culpaba al RVR («apagado, cargando o el cable fuera»).

        `exc is None` significa cierre INTENCIONAL (nuestro propio `close()`,
        p. ej. el apagado ordenado del driver): ahí no se resucita nada.
        """
        if exc is None:
            return
        logger.error(
            'puerto serie %s PERDIDO: %s: %s — se reabrirá solo (evidencia 126)',
            self._dev, type(exc).__name__, exc, exc_info=exc)
        self._reintentos_reapertura = 0
        # `call_soon_threadsafe`: pyserial-asyncio llama desde el loop, pero
        # esta ruta no debe depender de desde qué hilo llegue la mala noticia.
        self.__loop.call_soon_threadsafe(self.__loop.call_later,
                                         1.0, self._reintentar_apertura)

    def _reintentar_apertura(self):
        """Reconstruye serial.Serial + SerialTransport, con espera creciente.

        La espera (2, 4, 8, 16, 30, 30… s) es para el caso sin arreglo
        automático —el dispositivo desapareció—; en el caso medido (evidencia
        126: /dev/ttyAMA0 seguía existiendo) el PRIMER intento basta.
        """
        try:
            ser = serial.Serial(self._dev, self._baud)
        except (OSError, serial.SerialException) as e:
            self._reintentos_reapertura += 1
            espera = min(30.0, 2.0 ** self._reintentos_reapertura)
            logger.error('reapertura de %s fallida (%s); reintento en %.0f s',
                         self._dev, e, espera)
            self.__loop.call_later(espera, self._reintentar_apertura)
            return
        self.__transport = SerialTransport(self.__loop, self, ser)
        logger.warning('puerto serie %s REABIERTO (tras %d reintento(s) fallido(s))',
                       self._dev, self._reintentos_reapertura)

    def send(self, msg):
        """Send a Message instance to the port

        Args:
            msg (Message): Instance of Message

        """
        data = msg.serialise()
        logger.debug('Writing serial data: [{}]'.format(', '.join('0x{:02x}'.format(x) for x in data)))
        self.__transport.write(data)

    def data_received(self, data):
        logger.debug('Reading serial data: [{}]'.format(', '.join('0x{:02x}'.format(x) for x in data)))
        self._parser.feed(data)

    def pause_writing(self):
        pass

    def resume_writing(self):
        """Not implemented

        """
        pass

    def eof_received(self):
        pass

    def close(self):
        self.__transport.close()
