#!/usr/bin/env python3
"""EL PTY: lanzar el programa del alumno y hablar con él. POSIX, sin tornado.

Necesita Linux —`pty` no existe en Windows— pero **no necesita ROS ni el RVR**,
así que se puede probar en cualquier Linux: es donde quedan medidos los dos
primeros requisitos del taller sin tocar un robot.

════════════════════════════════════════════════════════════════════════════
🔴 POR QUE PTY Y NO UNA TUBERIA. LOS TRES MOTIVOS, MEDIDOS
════════════════════════════════════════════════════════════════════════════
1. **`print()` contra una tubería es de bloque.** `05_sensor_color.py` imprime
   una fila cada 0,5 s y el seguidor gira a 10 Hz: contra una tubería la salida
   aparecería a bloques de 4 KiB, o sea **pantalla congelada con el robot en
   marcha**. Con PTY, línea a línea.
   ⚠️ `PYTHONUNBUFFERED=1` ayuda pero **no basta por sí solo**: hay bibliotecas
   que miran `isatty()` y deciden por su cuenta.
2. **`input()` funciona.** Cuatro en `04_giro_preciso.py` y uno en
   `99_test_ctrl_c.py`. Sin terminal, `sys.stdin.readline()` sobre una tubería
   cerrada devuelve `''` y el programa **se salta la pausa sin avisar** — este
   proyecto ya pagó ese fallo exacto.
3. **La semántica de Ctrl-C llega sola.** Una señal al grupo de procesos es lo
   que `atriz.py` captura para recorrer `cerrar()`.

════════════════════════════════════════════════════════════════════════════
🔴 Y EL GRUPO DE PROCESOS: `os.setsid()`, NUNCA `pkill -f`
════════════════════════════════════════════════════════════════════════════
El PID se conoce **por haberlo engendrado**. Este proyecto tiene documentado que
`pkill -f` mata la propia terminal que lo ejecuta —pasó dos veces— y que el truco
del corchete tampoco basta. Aquí no aparece en ninguna parte: se señala al grupo
con `os.killpg`, que solo alcanza a lo que este agente creó.
"""

from __future__ import annotations

import errno
import fcntl
import os
import pty
import signal
import struct
import sys
import termios
from dataclasses import dataclass

#: Las señales que se pueden mandar, por nombre. Ver `SENALES` del núcleo.
_POR_NOMBRE = {
    'SIGINT': signal.SIGINT,
    'SIGQUIT': signal.SIGQUIT,
    'SIGTERM': signal.SIGTERM,
    'SIGHUP': signal.SIGHUP,
    'SIGKILL': signal.SIGKILL,
}


@dataclass
class Ejecucion:
    """Lo que hace falta para hablar con el hijo y para pararlo."""

    pid: int
    pgid: int
    maestro: int


def lanzar(argv: list[str], entorno: dict, cwd: str,
           columnas: int = 100, filas: int = 30) -> Ejecucion:
    """Arranca `argv` en su propio PTY y su propia sesión.

    🔴 `pty.fork()` y no `subprocess`: hace falta que el hijo tenga el esclavo
       como **terminal de control**, que es lo que hace que `input()` funcione y
       que un Ctrl-C llegue al grupo entero.
    """
    pid, maestro = pty.fork()
    if pid == 0:
        # ── HIJO ────────────────────────────────────────────────────────────
        # Sesión propia: así el grupo de procesos es SUYO y señalarlo no puede
        # alcanzar al agente ni a nada de systemd.
        try:
            os.setsid()
        except OSError:
            pass  # `pty.fork` ya la creó en algunos sistemas
        try:
            os.chdir(cwd)
        except OSError:
            pass
        # 🔴 `nice`: el driver corre un lazo de 0,05 s contra un watchdog de
        #    0,3 s. El programa del alumno NO puede competir con él.
        try:
            os.nice(5)
        except OSError:
            pass
        try:
            os.execvpe(argv[0], argv, entorno)
        except Exception as e:                                   # noqa: BLE001
            # Este print sale por el PTY, así que lo ve el alumno. Un fallo de
            # arranque mudo sería indistinguible de un programa que no imprime.
            sys.stderr.write(f'no se pudo arrancar: {e}\n')
            sys.stderr.flush()
        os._exit(127)

    # ── PADRE ───────────────────────────────────────────────────────────────
    os.set_blocking(maestro, False)
    redimensionar(maestro, columnas, filas)
    # 🔴🔴 `pgid = pid`, SIN preguntarle al kernel. CONFIRMADO POR EFECTO en la
    #    Pi (2026-08-14, evidencia 117): `os.getpgid(pid)` aquí corre en CARRERA
    #    con el `setsid()` del hijo. Si gana el padre, devuelve el grupo DEL
    #    PROPIO AGENTE — y el peldaño de SIGKILL de la parada se suicida con
    #    todo su grupo. Medido: 2 de 4 tandas de la suite murieron enteras por
    #    SIGKILL; con esta línea, 6 de 6 limpias.
    #
    #    Tras `setsid()` el pgid del hijo ES su pid, por definición — y
    #    `pty.fork()` garantiza ese `setsid` en el hijo (es lo que hace al
    #    esclavo su terminal de control). No hay nada que preguntar; preguntar
    #    era la ventana.
    pgid = pid
    return Ejecucion(pid=pid, pgid=pgid, maestro=maestro)


def redimensionar(maestro: int, columnas: int, filas: int) -> None:
    """`TIOCSWINSZ`. Sin esto, un `traceback` largo sale partido donde no toca."""
    try:
        fcntl.ioctl(maestro, termios.TIOCSWINSZ,
                    struct.pack('HHHH', filas, columnas, 0, 0))
    except OSError:
        pass


def leer(maestro: int, cuanto: int = 8192) -> bytes | None:
    """Lo que haya. `b''` si no hay nada ahora; `None` si el hijo cerró.

    🔴 LOS TRES CASOS SON DISTINTOS y confundirlos cuesta caro:
       · `b''` de `EAGAIN` es «todavía no», y hay que seguir leyendo.
       · `EIO` al cerrar el esclavo es **el final**, y en Linux es la forma
         NORMAL de enterarse de que el programa acabó — no un error.
       · Un `OSError` de verdad hay que dejarlo salir.
    """
    try:
        datos = os.read(maestro, cuanto)
    except BlockingIOError:
        return b''
    except OSError as e:
        if e.errno == errno.EIO:
            return None
        if e.errno == errno.EAGAIN:
            return b''
        raise
    # `b''` de una lectura sin excepción también es fin de fichero.
    return datos if datos != b'' else None


def escribir(maestro: int, texto: str) -> None:
    """Lo que el alumno contesta a un `input()`."""
    datos = texto.encode('utf8')
    while datos:
        try:
            n = os.write(maestro, datos)
        except BlockingIOError:
            return
        except OSError:
            return
        datos = datos[n:]


def senalar(pgid: int, nombre: str) -> bool:
    """Manda una señal AL GRUPO. `False` si ya no hay nadie a quien mandarla.

    ⚠️ Al grupo y no al PID: `atriz.py` puede haber creado hijos —el ejecutor de
       `rclpy` es hilos, pero un alumno puede lanzar procesos— y señalar solo al
       padre dejaría huérfanos conduciendo.
    """
    senal = _POR_NOMBRE.get(nombre)
    if senal is None:
        return False
    # 🔴 El cinturón del arreglo de `lanzar()`: `os.killpg(0, s)` señala al
    #    grupo DEL LLAMANTE y `killpg(1, s)` a init. Un pgid <= 1 jamás puede
    #    ser un hijo nuestro: la respuesta es negarse, no preguntar al kernel.
    if pgid <= 1:
        return False
    try:
        os.killpg(pgid, senal)
        return True
    except ProcessLookupError:
        return False
    except OSError:
        return False


def vive(pgid: int) -> bool:
    """¿Queda alguien del grupo? La señal 0 no manda nada: solo pregunta.

    ⚠️ Con `pgid <= 1` la pregunta misma miente: `killpg(0, 0)` responde por el
       grupo DEL LLAMANTE — o sea «sí» siempre — y la pantalla enseñaría
       «corriendo» para la eternidad. Un pgid así no es de ningún hijo nuestro.
    """
    if pgid <= 1:
        return False
    try:
        os.killpg(pgid, 0)
        return True
    except (ProcessLookupError, OSError):
        return False


def cosechar(pid: int) -> tuple[int | None, str | None]:
    """`(codigo, senal)` si ya terminó; `(None, None)` si sigue vivo.

    🔴 Sin esto el hijo se queda ZOMBI y `vive()` seguiría diciendo que sí para
       siempre: la pantalla enseñaría «corriendo» sobre un programa acabado, que
       es justo la familia de fallo que este proyecto persigue.
    """
    try:
        hijo, estado = os.waitpid(pid, os.WNOHANG)
    except ChildProcessError:
        return None, None
    if hijo == 0:
        return None, None
    if os.WIFSIGNALED(estado):
        s = os.WTERMSIG(estado)
        nombre = next((n for n, v in _POR_NOMBRE.items() if v == s), f'señal {s}')
        return None, nombre
    return os.WEXITSTATUS(estado), None


def copiar_biblioteca(origen_dir: str, destino_dir: str) -> bool:
    """Copia `atriz.py` a la carpeta de la sesión.

    🔴 ES LO QUE SUSTITUYE AL «PYTHONPATH EN SOLO LECTURA» DEL PLAN, que no se
       puede hacer: el agente corre como `sphero` y el directorio de prácticas es
       de `sphero` — mismo usuario, mismo derecho de escritura.

    Copiarla en cada lanzamiento consigue el objetivo declarado —que un alumno no
    rompa la biblioteca para el siguiente— y es más fuerte, porque se regenera.

    ⚠️ Lo que NO cierra: que el guion escriba en el directorio real con `open()`.
       Eso solo lo cierra un montaje de solo lectura, y va en endurecimiento.
    """
    origen = os.path.join(origen_dir, 'atriz.py')
    if not os.path.isfile(origen):
        return False
    try:
        with open(origen, 'rb') as f:
            datos = f.read()
        with open(os.path.join(destino_dir, 'atriz.py'), 'wb') as f:
            f.write(datos)
        return True
    except OSError:
        return False


def hay_scan(dominio: str | None = None) -> bool | None:
    """⚠️ NO IMPLEMENTADO AQUI, y es deliberado.

    La comprobación de efecto —¿seguía el barrido?, ¿se movió `/odom`?— habla con
    rosbridge, no con el PTY. Vive en el servidor (`agente_sesion.py`), que ya
    tiene un bucle de tornado con el que hacerlo sin arrancar `rclpy` ni pagar el
    arranque de la CLI de `ros2`.

    Se deja esta firma escrita para que nadie la implemente aquí por descuido.
    """
    raise NotImplementedError('la comprobación de efecto vive en agente_sesion.py')
