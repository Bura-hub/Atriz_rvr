"""Quién puede abrir rosbridge — la decisión, sin tornado y sin ROS.

Es la mitad probable de la Fase B (`SEGURIDAD_ROSBRIDGE.md`). El lanzador
`atriz_rosbridge.py` es pegamento delgado: aquí vive lo que hay que poder
recorrer sin un robot delante, que son **los caminos negativos** — sin testigo,
testigo de otro robot, firma mala, reloj sin sincronizar.

═══════════════════════════════════════════════════════════════════════════════
POR QUÉ ESTO NO ES UN PROXY, QUE ES LO QUE DECÍA EL DISEÑO ORIGINAL
═══════════════════════════════════════════════════════════════════════════════
`SEGURIDAD_ROSBRIDGE.md` planteaba la Fase B como un proxy en otro puerto que
relevara hasta `ws://127.0.0.1:9090`. Se escribió antes de que existiera el
Taller, y al mirar el fuente de rosbridge (2026-08-15) apareció algo mejor:

    rosbridge_websocket.py:54   from rosbridge_server import ... RosbridgeWebSocket
    rosbridge_websocket.py:221  handlers = [(r"/", RosbridgeWebSocket), ...]

La clase se importa **por nombre**, así que basta con parchear su `open` antes
de que el nodo arranque. Ventajas medidas contra el proxy:

  · **cero relevo**: el proxy metía un salto de Python en la ruta de 80,7 kB/s
    por robot, sobre una Pi que con el stack completo ya va al ~89 % de un
    núcleo. El documento afirmaba que «los datos siguen yendo robot → navegador
    directos», y con un relevo eso dejaba de ser cierto dentro de la Pi;
  · **sin puerto nuevo ni unidad nueva**;
  · **TLS sale gratis**: rosbridge ya soporta `certfile`/`keyfile`
    (`rosbridge_websocket.py:66-67`, usados en `:234-239`).

🔴 Y SE PARCHEA EL MÉTODO EN EL SITIO, NO SE SUBCLASA. No es estilo: es que
   `websocket_handler.py:60`, dentro de `_log_exception()`, hace

       node_handle = RosbridgeWebSocket.node_handle
       assert isinstance(node_handle, Node), "Node handle was not set"

   sobre la clase ORIGINAL, mientras el nodo (`:133`) asigna ese atributo sobre
   **el nombre que importó**. Con una subclase esos dos dejan de ser la misma
   clase, el atributo queda sin poner, y ese `assert` revienta — justo al
   registrar una excepción, o sea en el peor momento posible.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Sequence

#: El testigo viaja en el subprotocolo del WebSocket, como en el Taller. No hay
#: otro sitio: el navegador no deja poner cabeceras en un `new WebSocket(...)`,
#: y rosbridge 2.7.0 **no tiene autenticación** — ni `rosauth`, ni parámetro
#: `authenticate`, ni la capacidad en el protocolo, y `check_origin()` devuelve
#: `True` incondicionalmente.
PREFIJO_TESTIGO = 'atriz.token.'

#: Lo que se responde en el apretón. Mismo valor que el Taller a propósito: son
#: dos servidores del mismo robot hablando con el mismo cliente.
SUBPROTOCOLO = 'atriz.v1'

#: Códigos de cierre. Se repiten aquí en vez de importarse de `atriz_testigo`
#: para que este módulo se pueda probar sin el otro repositorio; el lanzador
#: comprueba que coinciden y **se niega a arrancar si no**, que es lo que impide
#: que las dos copias se separen en silencio.
CIERRE_RELOJ = 1013
CIERRE_SIN_TESTIGO = 4401
CIERRE_TESTIGO_MALO = 4403
CIERRE_OTRO_ROBOT = 4404

#: 🔴 QUIÉN ENTRA SIN TESTIGO, Y POR QUÉ NO REGALA NADA.
#:
#: Quien ya corre **dentro** de la Pi alcanza `raw_motors` y `set_ir_mode` con
#: `import rclpy`, saltándose el `collision_monitor` — está escrito en el README
#: del agente como límite conocido del Taller. Exigir testigo desde `127.0.0.1`
#: no cerraría nada y en cambio dejaría muertas las herramientas de banco que
#: corren en el robot (`probar_rosbridge.py`, `probar_lista_blanca.py`,
#: `probar_color_por_websocket.py`).
#:
#: ⚠️ Lo que SÍ hay que decir: esto **no** exime a las dos herramientas HTML
#:    (`probar_conexion_web.html`, `medir_aula.html`), que corren en el
#:    navegador del PC. Esas necesitan testigo o dejan de funcionar.
EXENTOS = ('127.0.0.1', '::1')


@dataclass(frozen=True)
class Decision:
    """Qué hacer con una conexión entrante."""

    admitir: bool
    codigo: int | None = None
    motivo: str = ''
    #: Quién es, cuando se sabe. Va al log del robot para poder responder a
    #: «¿quién movió el robot 7 a las 11:40?», que hoy no tiene respuesta.
    sujeto: str = ''
    #: Lo que hay que devolver en `Sec-WebSocket-Protocol`, o `None`.
    subprotocolo: str | None = None


def elegir_subprotocolo(ofrecidos: Sequence[str]) -> str | None:
    """Devuelve algo que el cliente HAYA ofrecido, o `None`.

    🔴 NUNCA UN VALOR FIJO. tornado ejecuta

        assert self.selected_subprotocol in subprotocols

    así que devolver `atriz.v1` a un cliente que no lo ofreció da un
    `AssertionError` y un **HTTP 500** en vez del cierre con código y motivo que
    esta clase promete — o sea, el camino del 4401 se vuelve inalcanzable y cada
    intento deja la traza entera en el journal.

    Pasó de verdad el 2026-08-15 en el agente del Taller (evidencia 120), y su
    prueba contra el doble **estaba en verde**: el doble escribe la cabecera del
    apretón a mano y no tiene ese `assert`. La regla que salió de ahí: *lo que un
    doble no puede reproducir es su manejo de errores, porque el error lo produce
    la BIBLIOTECA del original.*
    """
    if not ofrecidos:
        return None
    if SUBPROTOCOLO in ofrecidos:
        return SUBPROTOCOLO
    return ofrecidos[0]


def extraer_testigo(ofrecidos: Sequence[str]) -> str | None:
    """Saca el testigo crudo del subprotocolo, si viene.

    📝 Se llamaba `testigo_de` y hubo que renombrarla: **pytest recoge cualquier
       función cuyo nombre empiece por `test`**, y `testigo` empieza por `test`.
       Se colaba en la suite como una prueba sin fixture. En castellano esa
       colisión es fácil de repetir -`testear`, `testigo`, `testimonio`-.
    """
    for s in ofrecidos:
        if s.startswith(PREFIJO_TESTIGO):
            return s[len(PREFIJO_TESTIGO):]
    return None


def decidir(
    ofrecidos: Sequence[str],
    ip_remota: str,
    *,
    verificar: Callable[[str], object],
    exentos: Sequence[str] = EXENTOS,
) -> Decision:
    """La decisión entera, en un sitio y sin efectos.

    `verificar` recibe el testigo crudo y devuelve algo con `.valido`, `.codigo`,
    `.motivo` y `.sujeto` — la forma de `atriz_testigo.Veredicto`. Se inyecta en
    vez de importarse para que estas pruebas no dependan del otro repositorio, y
    para poder recorrer los cuatro rechazos sin fabricar firmas.
    """
    sub = elegir_subprotocolo(ofrecidos)

    if ip_remota in exentos:
        # Ver el comentario de EXENTOS: no regala nada, y mantiene vivas las
        # herramientas de banco que corren en el propio robot.
        return Decision(True, sujeto=f'local({ip_remota})', subprotocolo=sub)

    crudo = extraer_testigo(ofrecidos)
    if crudo is None:
        return Decision(
            False, CIERRE_SIN_TESTIGO,
            'no llegó ningún testigo: abre el robot desde la web, con la sesión '
            'iniciada',
            subprotocolo=sub,
        )

    v = verificar(crudo)
    if getattr(v, 'valido', False):
        return Decision(True, sujeto=getattr(v, 'sujeto', ''), subprotocolo=sub)

    return Decision(
        False,
        getattr(v, 'codigo', CIERRE_TESTIGO_MALO),
        getattr(v, 'motivo', 'testigo rechazado'),
        getattr(v, 'sujeto', ''),
        subprotocolo=sub,
    )
