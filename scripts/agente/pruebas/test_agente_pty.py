"""EL PTY, medido — y AQUI SE CIERRAN DOS DE LOS TRES REQUISITOS DEL TALLER.

🔴 SIN ROBOT Y SIN ROS. Solo hace falta un Linux: la Pi, WSL, o un contenedor.

Los tres requisitos que la pantalla del taller llevaba escritos como encargo
—«PTY, no tubería», «stdin bidireccional», «señales y PID a la vista»— venían
cada uno con la medición que los obliga. Los dos primeros se pueden convertir en
evidencia AQUI, sin tocar un RVR, y eso es lo que hace este fichero.

🔴 Y CADA UNO LLEVA SU CONTROL CONTRA UNA TUBERIA. Sin el control, «funciona con
   PTY» no distingue «el PTY lo arregla» de «esto funcionaba de todas formas», y
   entonces la prueba no prueba el requisito: prueba que Python imprime.

⚠️ Se salta en Windows, y eso NO es «pasa»: `pty` no existe ahí. Cuando estas
   pruebas salen como `skipped`, los requisitos siguen SIN medir.
"""

import os
import subprocess
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

pytestmark = pytest.mark.skipif(
    os.name != 'posix',
    reason='`pty` es de POSIX: en Windows estos requisitos quedan SIN medir',
)

if os.name == 'posix':
    from agente_pty import (
        cosechar, escribir, lanzar, leer, redimensionar, senalar, vive,
    )
    from agente_nucleo import entorno_de_ejecucion


#: Imita la cadencia de `05_sensor_color.py`: una fila cada 0,5 s.
GUION_FILAS = (
    'import time\n'
    'for i in range(6):\n'
    "    print(f'R={120+i} G={98+i} B={77+i}')\n"
    '    time.sleep(0.5)\n'
)

#: Imita los `input()` de `04_giro_preciso.py`.
GUION_PREGUNTA = (
    "print('¿cuántos grados ha girado?', end=' ')\n"
    'g = input()\n'
    "print(f'anotado: {g}')\n"
)

#: Imita `99_test_ctrl_c.py`: captura la señal y limpia antes de salir.
GUION_SENAL = (
    'import signal, sys, time\n'
    'def cerrar(s, f):\n'
    "    print('CERRANDO LIMPIO')\n"
    '    sys.stdout.flush()\n'
    '    sys.exit(0)\n'
    'signal.signal(signal.SIGINT, cerrar)\n'
    "print('listo')\n"
    'sys.stdout.flush()\n'
    'time.sleep(30)\n'
)


@pytest.fixture
def taller(tmp_path):
    """Una carpeta de sesión, como la que el agente crea en /run/atriz/<sid>."""
    return tmp_path


def escribir_guion(carpeta, texto, nombre='prog.py'):
    ruta = carpeta / nombre
    ruta.write_text(texto, encoding='utf8')
    return str(ruta)


def leer_hasta(ej, segundos, hasta=None):
    """Acumula lo que salga durante `segundos`, o hasta ver `hasta`."""
    fin = time.time() + segundos
    trozos = []
    while time.time() < fin:
        d = leer(ej.maestro)
        if d is None:
            break
        if d:
            trozos.append(d.decode('utf8', 'replace'))
            if hasta is not None and hasta in ''.join(trozos):
                break
        else:
            time.sleep(0.02)
    return ''.join(trozos)


def entorno(carpeta):
    return entorno_de_ejecucion(dict(os.environ), str(carpeta))


# ═══════════════════════════════════════════════════════════════════════════
# REQUISITO 1 · PTY, no tubería
# ═══════════════════════════════════════════════════════════════════════════
def test_con_pty_las_filas_llegan_SEGUN_SE_IMPRIMEN(taller):
    """La cadencia de `05_sensor_color.py`, en vivo.

    Contra una tubería esto saldría a bloques y la pantalla se quedaría
    congelada CON EL ROBOT EN MARCHA, que es el requisito entero.
    """
    prog = escribir_guion(taller, GUION_FILAS)
    ej = lanzar([sys.executable, prog], entorno(taller), str(taller))
    try:
        # A 1,2 s tienen que haber salido ya dos o tres filas, no cero.
        pronto = leer_hasta(ej, 1.2)
        assert 'R=120' in pronto, f'a 1,2 s no había salido ni la primera fila: {pronto!r}'
        assert 'R=121' in pronto, 'la segunda fila tampoco: parece salida a bloques'
    finally:
        senalar(ej.pgid, 'SIGKILL')


def test_EL_CONTROL_con_tuberia_la_misma_salida_llega_a_BLOQUES(taller):
    """🔴 SIN ESTA PRUEBA, LA DE ARRIBA NO PRUEBA EL REQUISITO.

    Se corre el MISMO guion contra una tubería y sin `PYTHONUNBUFFERED`: si aquí
    también llegara en vivo, el PTY no estaría arreglando nada y el requisito
    estaría mal justificado.
    """
    prog = escribir_guion(taller, GUION_FILAS)
    env = dict(os.environ)
    env.pop('PYTHONUNBUFFERED', None)
    p = subprocess.Popen([sys.executable, prog], stdout=subprocess.PIPE,
                         stderr=subprocess.DEVNULL, env=env, cwd=str(taller))
    try:
        os.set_blocking(p.stdout.fileno(), False)
        fin = time.time() + 1.2
        visto = b''
        while time.time() < fin:
            try:
                d = p.stdout.read(4096)
                if d:
                    visto += d
            except BlockingIOError:
                pass
            time.sleep(0.02)
        # A 1,2 s, con tubería, NO ha salido nada: Python la trata como bloque.
        assert b'R=120' not in visto, (
            'la tubería entregó en vivo: entonces el PTY no es lo que arregla '
            'esto, y el requisito 1 está mal justificado'
        )
    finally:
        p.kill()
        p.wait(timeout=5)


# ═══════════════════════════════════════════════════════════════════════════
# REQUISITO 2 · stdin bidireccional
# ═══════════════════════════════════════════════════════════════════════════
def test_input_recibe_lo_que_se_escribe_y_el_programa_sigue(taller):
    """Los cuatro `input()` de `04_giro_preciso.py` y el de la 99."""
    prog = escribir_guion(taller, GUION_PREGUNTA)
    ej = lanzar([sys.executable, prog], entorno(taller), str(taller))
    try:
        pregunta = leer_hasta(ej, 3, hasta='grados')
        assert 'grados' in pregunta
        escribir(ej.maestro, '90\n')
        respuesta = leer_hasta(ej, 3, hasta='anotado')
        assert 'anotado: 90' in respuesta, respuesta
    finally:
        senalar(ej.pgid, 'SIGKILL')


def test_EL_CONTROL_sin_terminal_el_input_NO_espera_y_el_programa_SE_SALTA_LA_PAUSA(taller):
    """🔴 El fallo exacto que este proyecto ya pagó, reproducido.

    Con stdin cerrado, `input()` no espera: lanza EOFError o devuelve vacío, y el
    programa **se salta la pausa sin avisar**. En la práctica 4 eso significa que
    el robot gira otra vez mientras el alumno todavía está midiendo.
    """
    prog = escribir_guion(taller, GUION_PREGUNTA)
    p = subprocess.run([sys.executable, prog], stdin=subprocess.DEVNULL,
                       capture_output=True, cwd=str(taller), timeout=10)
    # No se quedó esperando: terminó solo, y encima con error.
    assert p.returncode != 0 or b'anotado' not in p.stdout


def test_el_eco_del_PTY_devuelve_lo_tecleado(taller):
    """Y por eso la caja del navegador NO debe escribirlo localmente.

    Es como por SSH: lo que se teclea vuelve por la salida. Pintarlo además en el
    cliente lo duplicaría, y el alumno vería «9090» al escribir «90».
    """
    prog = escribir_guion(taller, GUION_PREGUNTA)
    ej = lanzar([sys.executable, prog], entorno(taller), str(taller))
    try:
        leer_hasta(ej, 3, hasta='grados')
        escribir(ej.maestro, '90\n')
        todo = leer_hasta(ej, 3, hasta='anotado')
        assert todo.count('90') >= 2, f'no hubo eco: {todo!r}'
    finally:
        senalar(ej.pgid, 'SIGKILL')


# ═══════════════════════════════════════════════════════════════════════════
# REQUISITO 3 · señales al GRUPO
# ═══════════════════════════════════════════════════════════════════════════
def test_SIGINT_llega_y_el_programa_puede_limpiar_antes_de_salir(taller):
    """Es el camino que `atriz.py` recorre para parar el robot y apagar el barrido.

    Lo que se comprueba es que la señal LLEGA y que el manejador corre. Que
    además el robot se pare **no se puede medir aquí**: eso exige el RVR.
    """
    prog = escribir_guion(taller, GUION_SENAL)
    ej = lanzar([sys.executable, prog], entorno(taller), str(taller))
    try:
        assert 'listo' in leer_hasta(ej, 3, hasta='listo')
        assert senalar(ej.pgid, 'SIGINT') is True
        assert 'CERRANDO LIMPIO' in leer_hasta(ej, 3, hasta='CERRANDO')
    finally:
        senalar(ej.pgid, 'SIGKILL')


def test_el_PID_es_el_que_engendramos_y_NUNCA_se_busca_por_patron(taller):
    """`pkill -f` no aparece en el agente, y este es el motivo.

    Este proyecto tiene dos incidentes documentados en los que `pkill -f` mató la
    propia terminal que lo ejecutaba. Aquí el PID se conoce por haberlo creado.
    """
    prog = escribir_guion(taller, GUION_SENAL)
    ej = lanzar([sys.executable, prog], entorno(taller), str(taller))
    try:
        assert ej.pid > 0
        assert vive(ej.pgid) is True
    finally:
        senalar(ej.pgid, 'SIGKILL')


def test_cosechar_no_deja_zombis_y_dice_como_termino(taller):
    """Sin cosechar, `vive()` diría que sí para siempre.

    La pantalla enseñaría «corriendo» sobre un programa acabado, que es
    exactamente la familia de fallo que este proyecto persigue.
    """
    prog = escribir_guion(taller, "print('adios')\n")
    ej = lanzar([sys.executable, prog], entorno(taller), str(taller))
    leer_hasta(ej, 3, hasta='adios')
    fin = time.time() + 5
    codigo, senal = None, None
    while time.time() < fin:
        codigo, senal = cosechar(ej.pid)
        if codigo is not None or senal is not None:
            break
        time.sleep(0.05)
    assert codigo == 0, f'código={codigo} señal={senal}'


def test_matar_de_verdad_termina_el_grupo(taller):
    prog = escribir_guion(taller, 'import time\nwhile True: time.sleep(0.1)\n')
    ej = lanzar([sys.executable, prog], entorno(taller), str(taller))
    assert vive(ej.pgid) is True
    senalar(ej.pgid, 'SIGKILL')
    fin = time.time() + 5
    while time.time() < fin and vive(ej.pgid):
        cosechar(ej.pid)
        time.sleep(0.05)
    cosechar(ej.pid)
    assert vive(ej.pgid) is False


# ═══════════════════════════════════════════════════════════════════════════
# El entorno y la biblioteca
# ═══════════════════════════════════════════════════════════════════════════
def test_el_hijo_ve_PYTHONPATH_a_su_carpeta_de_sesion(taller):
    prog = escribir_guion(taller, "import os\nprint('RUTA', os.environ.get('PYTHONPATH'))\n")
    ej = lanzar([sys.executable, prog], entorno(taller), str(taller))
    try:
        assert str(taller) in leer_hasta(ej, 3, hasta='RUTA')
    finally:
        senalar(ej.pgid, 'SIGKILL')


def test_el_hijo_NO_ve_lo_que_el_entorno_descarta(taller):
    """Un secreto del portátil no viaja al programa del alumno."""
    os.environ['SECRETO_DE_PRUEBA'] = 'no deberia estar'
    try:
        prog = escribir_guion(taller, "import os\nprint('X', os.environ.get('SECRETO_DE_PRUEBA'))\n")
        ej = lanzar([sys.executable, prog], entorno(taller), str(taller))
        try:
            assert 'X None' in leer_hasta(ej, 3, hasta='X ')
        finally:
            senalar(ej.pgid, 'SIGKILL')
    finally:
        os.environ.pop('SECRETO_DE_PRUEBA', None)


def test_redimensionar_no_revienta_con_valores_raros(taller):
    prog = escribir_guion(taller, 'import time\ntime.sleep(2)\n')
    ej = lanzar([sys.executable, prog], entorno(taller), str(taller))
    try:
        redimensionar(ej.maestro, 200, 60)
        redimensionar(ej.maestro, 1, 1)
    finally:
        senalar(ej.pgid, 'SIGKILL')


def test_leer_devuelve_None_cuando_el_hijo_cierra(taller):
    """En Linux el fin llega como EIO, y NO es un error: es el final.

    Confundirlo con un fallo dejaría la pantalla diciendo «error de lectura»
    cada vez que un programa termina normalmente.
    """
    prog = escribir_guion(taller, "print('fin')\n")
    ej = lanzar([sys.executable, prog], entorno(taller), str(taller))
    fin = time.time() + 5
    cerrado = False
    while time.time() < fin:
        if leer(ej.maestro) is None:
            cerrado = True
            break
        time.sleep(0.05)
    cosechar(ej.pid)
    assert cerrado
