#!/usr/bin/env python3
"""EL AGENTE DE SESIÓN: el servicio que ejecuta el código del alumno.

    python3 agente_sesion.py --robot 7 --clave /etc/atriz/testigo.pub

🔴🔴 **NADA DE ESTE FICHERO ESTÁ EJECUTADO NI MEDIDO.** Escrito desde el PC el
2026-08-14, sin robot donde instalarlo y sin Linux donde probar el PTY. Lo que sí
está probado es todo lo que hay debajo: `agente_nucleo.py` (31 pruebas, en el PC)
y `agente_pty.py` (13 pruebas que **se saltan en Windows** y quedan pendientes de
un Linux cualquiera).

Este fichero es el pegamento: tornado, el bucle de eventos y la comprobación de
efecto. Su lógica se mantuvo deliberadamente delgada para que lo que decide viva
en los otros dos, donde se puede probar.

════════════════════════════════════════════════════════════════════════════
🔴 LO QUE ESTE AGENTE **NO** ES
════════════════════════════════════════════════════════════════════════════
**No es una frontera de seguridad.** El programa del alumno corre como `sphero`,
igual que el driver: puede leer lo que `sphero` lea, y con `rclpy` nativo alcanza
`raw_motors` y `set_ir_mode`, saltándose la capa de seguridad. Protege **errores
honestos** —el guion colgado, el que llena la pantalla, el que olvida apagar el
barrido—, no a alguien hostil. Va escrito también en la pantalla.

════════════════════════════════════════════════════════════════════════════
DEPENDENCIAS, Y NINGUNA ES NUEVA
════════════════════════════════════════════════════════════════════════════
· `tornado` — llega con rosbridge, que es justo el argumento para usarlo.
· `cryptography` — para verificar el testigo.
· `atriz_testigo` — vive en el OTRO repositorio (`~/atriz_migracion/scripts`), y
  el envoltorio lo pone en `PYTHONPATH`. Se importa en vez de copiarse: dos
  copias de un verificador de firmas se separan en silencio, y el día que pase
  el síntoma sería «el terminal no abre en ese robot».
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import signal
import sys
import time
import uuid

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from agente_nucleo import (  # noqa: E402
    ARRANCANDO, CORRIENDO, LIBRE, PARANDO, Decodificador, Limitador, Ranura,
    entorno_de_ejecucion, es_el_dueno, interpretar,
)
import agente_pty  # noqa: E402

try:
    import tornado.ioloop
    import tornado.web
    import tornado.websocket
except ImportError:                                              # pragma: no cover
    sys.exit('🔴 falta `tornado`. Llega con rosbridge: comprueba que ROS 2 está sourceado.')

try:
    from atriz_testigo import (
        CIERRE_SIN_TESTIGO, PREFIJO_TESTIGO, SUBPROTOCOLO, verificar,
    )
except ImportError:                                              # pragma: no cover
    sys.exit(
        '🔴 no encuentro `atriz_testigo`, que vive en el repositorio de migración.\n'
        '   Se pone en PYTHONPATH: export PYTHONPATH=$HOME/atriz_migracion/scripts:$PYTHONPATH\n'
        '   Se importa en vez de copiarse a propósito: dos copias de un verificador de\n'
        '   firmas se separan en silencio.',
    )

#: Donde viven las prácticas del alumno, en el robot.
DIR_PRACTICAS = os.path.expanduser(
    '~/atriz_ws/src/Atriz_rvr/scripts/estudiantes')

#: tmpfs. Sin desgaste de microSD y sin restos tras reiniciar.
#:
#: 🔴🔴 LA UNIDAD DEBE LLEVAR `RuntimeDirectoryPreserve=yes`. `atriz-robot.service`
#:    declara este MISMO `RuntimeDirectory=atriz`, y ahí vive la marca del vigía
#:    de DDS que garantiza «una sola cura por arranque». Sin el `Preserve`,
#:    **parar el agente borraría `/run/atriz` entero** y con él esa marca. Es un
#:    fallo cruzado entre dos unidades que ningún `systemd-analyze verify` ve.
DIR_RUN = '/run/atriz'

ranura = Ranura()
#: La ejecución viva. Uno por proceso: la ranura ya garantiza que sea una.
actual: dict | None = None
clientes: set = set()
#: True desde que systemd nos manda parar: no se aceptan ejecuciones nuevas y
#: el bucle se detiene en cuanto la que haya termine sus peldaños.
apagando = False


def difundir(mensaje: dict) -> None:
    for c in list(clientes):
        try:
            c.write_message(json.dumps(mensaje))
        except Exception:                                        # noqa: BLE001
            clientes.discard(c)


def difundir_estado() -> None:
    """El estado, pero **cada cliente el suyo**.

    🔴🔴 `soy_el_dueno` ES POR DESTINATARIO, Y SE DIFUNDÍA CALCULADO PARA UNO.

    Estaba escrito `difundir(estado_actual(actual["sujeto"]))`: el MISMO mensaje
    a todos los clientes, con `soy_el_dueno` calculado para el dueño. O sea que
    **todos** lo recibían en `True`.

    Medido desde el navegador el 2026-08-15, con dos alumnos y un solo robot: la
    pantalla del segundo decía «Ya tienes un programa corriendo. Párralo antes.»
    sobre el programa del primero, y le enseñaba su PID.

    ⚠️ **No era un agujero**: el `parar` del segundo se rechazó aquí y el
       programa siguió vivo — la comprobación de dueño nunca dependió de este
       campo. Lo que fallaba era lo que la PANTALLA podía afirmar.

    📌 Es la forma de siempre en este proyecto: **una cosa compartida sirviendo a
       varios clientes**, igual que rosbridge con su única suscripción por topic,
       donde el QoS del primero se lo impone a todos. Cuando un campo depende de
       QUIÉN pregunta, no se puede difundir.
    """
    for c in list(clientes):
        try:
            # `getattr` y no `c.sujeto`: un AttributeError aqui lo tragaria el
            # `except` de abajo y **descartaria al cliente en silencio**. Sin
            # nombre, `es_el_dueno` dice que no, que es la respuesta prudente.
            c.write_message(json.dumps(estado_actual(getattr(c, 'sujeto', ''))))
        except Exception:                                        # noqa: BLE001
            clientes.discard(c)


def estado_actual(para_sujeto: str = '') -> dict:
    if actual is None:
        return {'op': 'atriz_estado', 'estado': LIBRE, 'sujeto': '', 'pid': None,
                'soy_el_dueno': False, 'nombre': '', 'huella': '',
                'restante_s': None, 'lineas_descartadas': 0}
    queda = actual['tope_pared_s'] - (time.time() - actual['arranco'])
    return {
        'op': 'atriz_estado',
        'estado': actual['estado'],
        'sujeto': actual['sujeto'],
        'pid': actual['ej'].pid,
        'soy_el_dueno': es_el_dueno(actual['sujeto'], para_sujeto),
        'nombre': actual['nombre'],
        'huella': actual['huella'],
        # 🔴 Lo manda el AGENTE. El navegador NO puede calcularlo: la Pi no tiene
        #    RTC y arranca con el reloj hasta 19,5 h en el pasado.
        'restante_s': max(0, int(queda)),
        'lineas_descartadas': actual['limitador'].lineas_descartadas,
    }


def ocupacion_para(sujeto: str) -> dict | None:
    if actual is None:
        return None
    return {
        'sujeto': actual['sujeto'], 'estado': actual['estado'],
        'pid': actual['ej'].pid, 'nombre': actual['nombre'],
        'desde_s': int(time.time() - actual['arranco']),
    }


def listado() -> dict:
    """Los ficheros REALES del robot.

    🔴 Nunca una lista escrita a mano. La web tenía una y **cinco de sus diez
       nombres no existían aquí**; mientras solo decía cuánto despejar era
       cosmético, con el terminal ejecutando es un botón que falla.
    """
    ficheros = []
    try:
        for n in sorted(os.listdir(DIR_PRACTICAS)):
            if n.endswith('.py') and n != 'atriz.py':
                ruta = os.path.join(DIR_PRACTICAS, n)
                ficheros.append({'nombre': n, 'bytes': os.path.getsize(ruta)})
    except OSError as e:
        return {'op': 'atriz_aviso', 'nivel': 'ATENCION',
                'mensaje': f'no puedo leer {DIR_PRACTICAS}: {e}'}
    return {'op': 'atriz_listado', 'directorio': DIR_PRACTICAS, 'ficheros': ficheros}


def terminar(motivo: str, senal: str | None = None) -> None:
    """Cosecha, comprueba el efecto y lo cuenta. Suelta la ranura."""
    global actual
    if actual is None:
        return
    ej = actual['ej']
    codigo, senal_real = agente_pty.cosechar(ej.pid)
    # 🔴 QUITAR EL HANDLER ANTES DE CERRAR EL FD (auditoría 2026-08-14). Sin
    #    esto, tornado conserva el registro del fd cerrado, el kernel REUTILIZA
    #    ese número para el PTY de la SIGUIENTE ejecución, y el `add_handler`
    #    nuevo choca con el rancio: el agente sobrevive a una ejecución y
    #    revienta en la segunda — que es justo la que nadie prueba.
    try:
        tornado.ioloop.IOLoop.current().remove_handler(ej.maestro)
    except (KeyError, ValueError, OSError):
        pass
    try:
        os.close(ej.maestro)
    except OSError:
        pass
    efecto = comprobar_efecto()
    difundir({
        'op': 'atriz_fin',
        'motivo': motivo,
        'codigo': codigo,
        'senal': senal_real or senal,
        'duracion_s': int(time.time() - actual['arranco']),
        'lineas_descartadas': actual['limitador'].lineas_descartadas,
        'efecto': efecto,
    })
    # La carpeta de la sesión es tmpfs = RAM: una clase entera de ejecuciones
    # la iría comiendo sin devolverla hasta el reinicio. Se recoge al terminar.
    shutil.rmtree(os.path.join(DIR_RUN, actual['sid']), ignore_errors=True)
    actual = None
    ranura.soltar()
    difundir(estado_actual())
    # Si systemd nos pidió parar, este era el último encargo: ahora sí.
    if apagando:
        tornado.ioloop.IOLoop.current().stop()


def comprobar_efecto() -> dict:
    """Lo que pasó DESPUÉS. Cada campo puede ser `None` = «no lo sé».

    🔴🔴 Y AQUI NO SE LLAMA A `/stop_scan` A CIEGAS.

    `atriz.py` solo apaga el barrido si lo encendió él, precisamente para no
    dejar ciega a una navegación en curso. Un agente que lo apagara después de
    cada ejecución haría eso mismo: SLAM o Nav2 corriendo y de pronto sin `/scan`,
    sin que nada lo explique.

    ⏳ **SIN IMPLEMENTAR, y por eso devuelve «no lo sé» en vez de inventar.** Hacer
       esto bien exige hablar con rosbridge en `127.0.0.1:9090` —suscribirse a
       `/scan` y `/odom` un par de segundos, leer `/estado_navegacion`— y eso no
       se puede escribir a ciegas: hay que medirlo en el robot. Devolver `false`
       aquí sería afirmar «he mirado y no pasa nada», que es lo que este proyecto
       persigue.
    """
    return {
        'scan_llegaba': None,
        'navegacion_en_marcha': None,
        'stop_scan_llamado': False,
        'odom_max_lineal': None,
        'odom_max_angular': None,
    }


def bombear(fd: int, sucesos: int) -> None:
    """Lee el PTY y reparte. Lo llama el bucle de tornado, por evento."""
    global actual
    if actual is None:
        return
    datos = agente_pty.leer(actual['ej'].maestro)
    if datos is None:
        terminar('SALIDA_NORMAL')
        return
    if not datos:
        return
    texto = actual['decodificador'].alimentar(datos)
    antes = actual['limitador'].lineas_descartadas
    emitir = actual['limitador'].admitir(texto)
    if emitir:
        difundir({'op': 'atriz_salida', 'texto': emitir})
    if actual['limitador'].lineas_descartadas != antes:
        difundir({
            'op': 'atriz_recorte',
            'lineas_descartadas': actual['limitador'].lineas_descartadas,
            'bytes_descartados': actual['limitador'].bytes_descartados,
        })


def latir() -> None:
    """1 Hz: el estado, y el tope de pared."""
    if actual is None:
        return
    difundir_estado()
    if time.time() - actual['arranco'] > actual['tope_pared_s']:
        parar('TOPE_PARED')
    else:
        agente_pty.cosechar(actual['ej'].pid)


def parar(motivo: str) -> None:
    """Los cuatro peldaños, uno por vuelta del temporizador."""
    global actual
    if actual is None:
        return
    actual['estado'] = PARANDO
    actual.setdefault('parada_desde', time.time())
    actual.setdefault('enviadas', ())
    from agente_nucleo import plan_de_parada
    paso = plan_de_parada(actual['enviadas'], actual['parada_desde'], time.time())
    if paso == 'HECHO' or not agente_pty.vive(actual['ej'].pgid):
        terminar(motivo, senal=actual['enviadas'][-1] if actual['enviadas'] else None)
        return
    if paso != 'ESPERAR':
        agente_pty.senalar(actual['ej'].pgid, paso)
        actual['enviadas'] = (*actual['enviadas'], paso)
    difundir_estado()
    tornado.ioloop.IOLoop.current().call_later(1.0, lambda: parar(motivo))


class Sesion(tornado.websocket.WebSocketHandler):
    """Una conexión. La RANURA es del proceso, no de la conexión."""

    sujeto = ''

    def check_origin(self, origin: str) -> bool:
        # Igual que rosbridge: el navegador viene de otro origen siempre. Lo que
        # autoriza es el testigo, no la cabecera `Origin` — que además es
        # trivial de falsificar desde algo que no sea un navegador.
        return True

    def select_subprotocol(self, subprotocols):
        """🔴 SE DEVUELVE SIEMPRE, incluso al ir a rechazar.

        Si no se devuelve ninguno, el navegador cierra por su cuenta con 1006 y
        **sin motivo**: el alumno vería «la conexión se cortó» en vez de «esa
        credencial es de otro robot».
        """
        self._ofrecidos = list(subprotocols or [])
        return SUBPROTOCOLO

    def open(self) -> None:                                      # noqa: A003
        crudo = next((s for s in getattr(self, '_ofrecidos', [])
                      if s.startswith(PREFIJO_TESTIGO)), None)
        if crudo is None:
            self.close(CIERRE_SIN_TESTIGO, 'no llegó ningún testigo')
            return
        v = verificar(crudo[len(PREFIJO_TESTIGO):], self.application.robot,
                      self.application.clave)
        if not v.ok:
            self.close(v.codigo, v.motivo)
            return
        self.sujeto = v.sujeto
        clientes.add(self)
        self.write_message(json.dumps({
            'op': 'atriz_bienvenida',
            'robot': self.application.robot,
            'sujeto': self.sujeto,
            'reloj_fiable': True,          # si no lo fuera, `verificar` no habría abierto
            'directorio': DIR_PRACTICAS,
            'sesion': ocupacion_para(self.sujeto),
        }))

    def on_close(self) -> None:
        clientes.discard(self)
        # 🔴 NO se para nada al cerrar el socket. El proceso vive en su propia
        #    sesión y sobrevive a una recarga o a una caída de WiFi: matarlo aquí
        #    convertiría un F5 en un robot que se para a mitad de práctica.

    def on_message(self, crudo) -> None:
        global actual
        try:
            mensaje = json.loads(crudo)
        except Exception:                                        # noqa: BLE001
            mensaje = None
        orden = interpretar(mensaje, ranura, self.sujeto)
        if not orden.ok:
            self.write_message(json.dumps({
                'op': 'atriz_rechazo', 'codigo': orden.codigo, 'motivo': orden.motivo,
            }))
            return

        if orden.op == 'atriz_adjuntar':
            self.write_message(json.dumps(estado_actual(self.sujeto)))
        elif orden.op == 'atriz_listar':
            self.write_message(json.dumps(listado()))
        elif orden.op == 'atriz_leer':
            # El docstring de `nombre_seguro` promete que el nombre «solo sirve
            # para BUSCARLO en la lista que el propio agente produjo» — y esta
            # línea lo cumple de verdad (auditoría 2026-08-14): primero se
            # comprueba que el fichero ESTÁ en el directorio real, y solo
            # entonces se abre. La regex ya impedía rutas; esto alinea el código
            # con su contrato.
            try:
                presentes = set(os.listdir(DIR_PRACTICAS))
            except OSError:
                presentes = set()
            if orden.datos['fichero'] not in presentes:
                self.write_message(json.dumps({
                    'op': 'atriz_rechazo', 'codigo': 'NO_ESTA',
                    'motivo': f'{orden.datos["fichero"]} no está en este robot',
                }))
                return
            ruta = os.path.join(DIR_PRACTICAS, orden.datos['fichero'])
            try:
                with open(ruta, encoding='utf8') as f:
                    self.write_message(json.dumps({
                        'op': 'atriz_fichero', 'nombre': orden.datos['fichero'],
                        'texto': f.read(),
                    }))
            except OSError as e:
                self.write_message(json.dumps({
                    'op': 'atriz_rechazo', 'codigo': 'NO_ESTA', 'motivo': str(e),
                }))
        elif orden.op == 'atriz_exec':
            if apagando:
                # El agente está cerrando (reinicio de la unidad): aceptar una
                # ejecución nueva aquí la mataría a los pocos segundos con un
                # SIGINT que el alumno no pidió. Mejor negarse y decirlo.
                self.write_message(json.dumps({
                    'op': 'atriz_rechazo', 'codigo': 'AGENTE_PARANDO',
                    'motivo': 'el agente se está reiniciando: reintenta en unos segundos',
                }))
                return
            self._arrancar(orden.datos)
        elif orden.op == 'atriz_stdin':
            if actual is not None:
                agente_pty.escribir(actual['ej'].maestro, orden.datos['texto'])
        elif orden.op == 'atriz_signal':
            if actual is not None:
                agente_pty.senalar(actual['ej'].pgid, orden.datos['senal'])
        elif orden.op == 'atriz_parar':
            parar('PARADA_PEDIDA')
        elif orden.op == 'atriz_tamano':
            if actual is not None:
                agente_pty.redimensionar(actual['ej'].maestro,
                                         orden.datos['columnas'], orden.datos['filas'])

    def _arrancar(self, datos: dict) -> None:
        global actual
        sid = uuid.uuid4().hex[:12]
        carpeta = os.path.join(DIR_RUN, sid)
        try:
            os.makedirs(carpeta, exist_ok=True)
        except OSError as e:
            self.close(1011, f'no puedo crear {carpeta}: {e}')
            return

        # 🔴 La biblioteca se COPIA en cada lanzamiento. Ver `copiar_biblioteca`.
        if not agente_pty.copiar_biblioteca(DIR_PRACTICAS, carpeta):
            self.write_message(json.dumps({
                'op': 'atriz_aviso', 'nivel': 'ATENCION',
                'mensaje': 'no he podido copiar atriz.py: `from atriz import Robot` fallará',
            }))
        ruta = os.path.join(carpeta, datos['nombre'])
        with open(ruta, 'w', encoding='utf8') as f:
            f.write(datos['codigo'])

        ranura.tomar(self.sujeto, sid, time.time())
        ej = agente_pty.lanzar([sys.executable, ruta],
                               entorno_de_ejecucion(dict(os.environ), carpeta), carpeta)
        actual = {
            'sid': sid, 'sujeto': self.sujeto, 'ej': ej, 'estado': CORRIENDO,
            'nombre': datos['nombre'], 'huella': datos['huella'],
            'arranco': time.time(), 'tope_pared_s': datos['tope_pared_s'],
            'limitador': Limitador(), 'decodificador': Decodificador(),
        }
        if datos.get('tope_recortado'):
            self.write_message(json.dumps({
                'op': 'atriz_aviso', 'nivel': 'NOTA',
                'mensaje': f'he recortado el tope a {datos["tope_pared_s"]} s',
            }))
        tornado.ioloop.IOLoop.current().add_handler(
            ej.maestro, bombear, tornado.ioloop.IOLoop.READ)
        difundir_estado()


def apagar_ordenado() -> None:
    """Lo que la unidad PROMETE en su `KillSignal=SIGINT` — y hasta la auditoría
    del 2026-08-14 no existía: sin manejador, el SIGINT era un KeyboardInterrupt
    que tumbaba tornado a mitad de bucle, y los «cuatro peldaños al hijo» del
    comentario de la unidad no los aplicaba nadie (el hijo recibía la señal de
    rebote por `KillMode=control-group`, y al vencer el timeout, SIGKILL a pelo
    sin pasar por SIGTERM).

    Ahora: sin ejecución → parar el bucle ya. Con ejecución → recorrer los
    peldaños de `plan_de_parada` y detener el bucle cuando `terminar()` acabe.
    Cabe en los 45 s de `TimeoutStopSec` (10 + 5 + margen).
    """
    global apagando
    apagando = True
    if actual is None:
        tornado.ioloop.IOLoop.current().stop()
        return
    difundir({'op': 'atriz_aviso', 'nivel': 'ATENCION',
              'mensaje': 'el agente se reinicia: parando tu programa con Ctrl-C'})
    parar('AGENTE_PARANDO')


def main() -> None:
    p = argparse.ArgumentParser(description='Agente de sesión del taller Atriz')
    p.add_argument('--robot', type=int, required=True, help='número de este robot (1-16)')
    p.add_argument('--clave', default='/etc/atriz/testigo.pub',
                   help='clave pública Ed25519 del servidor de la web')
    p.add_argument('--puerto', type=int, default=9443)
    args = p.parse_args()

    from cryptography.hazmat.primitives.serialization import load_pem_public_key
    try:
        with open(args.clave, 'rb') as f:
            clave = load_pem_public_key(f.read())
    except Exception as e:                                       # noqa: BLE001
        sys.exit(
            f'🔴 no puedo leer la clave pública de {args.clave}: {e}\n'
            '   Sin ella este agente no puede verificar a nadie, y arrancar sin verificar\n'
            '   sería peor que no arrancar: dejaría ejecutar código a cualquiera del aula.',
        )

    os.makedirs(DIR_RUN, exist_ok=True)
    app = tornado.web.Application([(r'/', Sesion)])
    app.robot = args.robot
    app.clave = clave
    app.listen(args.puerto)
    print(f'agente de sesión del robot {args.robot} en ws://0.0.0.0:{args.puerto}')
    print(f'  prácticas: {DIR_PRACTICAS}')
    # Las señales de systemd (SIGINT por KillSignal=; SIGTERM por si alguien lo
    # cambia) entran por el camino seguro-para-señales del bucle.
    for s in (signal.SIGINT, signal.SIGTERM):
        signal.signal(s, lambda _n, _f: tornado.ioloop.IOLoop.current()
                      .add_callback_from_signal(apagar_ordenado))
    tornado.ioloop.PeriodicCallback(latir, 1000).start()
    tornado.ioloop.IOLoop.current().start()


if __name__ == '__main__':
    main()
