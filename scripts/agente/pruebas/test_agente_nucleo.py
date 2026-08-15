"""El núcleo del agente de sesión. Sin robot, sin Linux, sin tornado.

🔴 LAS QUE IMPORTAN SON LAS NEGATIVAS.

Un agente que acepta todo pasa sin despeinarse las pruebas positivas: ejecuta,
llega la salida, para. Lo que distingue un agente utilizable de uno peligroso es
que RECHACE — la op desconocida, el robot ocupado, el programa de otro, el
código de un mega, y el Ctrl-C encubierto en la línea de entrada.

Este proyecto tiene once falsos positivos documentados en su propio verificador,
todos de la misma familia: comprobaciones que solo miraban el camino bueno.
"""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from agente_nucleo import (  # noqa: E402
    ESPERA_TRAS_SIGINT_S, SENALES, TOPE_CODIGO_BYTES, TOPE_PARED_MAX_S,
    TOPE_PARED_MIN_S, TOPE_PARED_S, Decodificador, Limitador, Ranura,
    entorno_de_ejecucion, huella_de, interpretar, nombre_seguro,
    plan_de_parada, recortar_tope,
)


@pytest.fixture
def ranura():
    return Ranura()


def exec_de(codigo='print("hola")', **extra):
    return {'op': 'atriz_exec', 'codigo': codigo, **extra}


# ═══════════════════════════════════════════════════════════════════════════
# El camino bueno, primero: si esto no pasara, los rechazos de abajo podrían
# ser «rechaza siempre».
# ═══════════════════════════════════════════════════════════════════════════
def test_un_exec_normal_se_acepta(ranura):
    o = interpretar(exec_de(), ranura, 'ana')
    assert o.ok
    assert o.datos['codigo'] == 'print("hola")'
    assert o.datos['tope_pared_s'] == TOPE_PARED_S
    assert o.datos['tope_recortado'] is False
    assert len(o.datos['huella']) == 12


def test_la_ranura_libre_se_toma_y_se_suelta(ranura):
    assert ranura.libre()
    oc = ranura.tomar('ana', 's1', 100.0)
    assert not ranura.libre()
    assert oc.sujeto == 'ana'
    ranura.soltar()
    assert ranura.libre()


# ═══════════════════════════════════════════════════════════════════════════
# 🔴 LOS RECHAZOS
# ═══════════════════════════════════════════════════════════════════════════
def test_una_op_desconocida_se_rechaza_nombrandola(ranura):
    o = interpretar({'op': 'atriz_borrar_todo'}, ranura, 'ana')
    assert not o.ok
    assert o.codigo == 'OP_DESCONOCIDA'
    # El motivo dice CUÁL, o quien lo lea no sabe si escribió mal el nombre.
    assert 'atriz_borrar_todo' in o.motivo


def test_basura_no_revienta_el_agente(ranura):
    """Nada de lo que llegue por el cable puede tumbar la ejecución de alguien."""
    for malo in [None, 42, 'hola', [], {}, {'op': ''}, {'op': 7}]:
        o = interpretar(malo, ranura, 'ana')
        assert not o.ok, malo


def test_con_la_ranura_tomada_se_rechaza_Y_SE_DICE_QUIEN(ranura):
    """El requisito de concurrencia entero.

    Un «ocupado» a secas deja al segundo alumno mirando sin saber a quién buscar.
    Con dos robots por mesa, el nombre es la diferencia entre esperar y preguntar.
    """
    ranura.tomar('ana', 's1', 100.0)
    o = interpretar(exec_de(), ranura, 'luis')
    assert not o.ok
    assert o.codigo == 'OCUPADO'
    assert 'ana' in o.motivo


def test_el_MISMO_sujeto_no_se_bloquea_a_si_mismo(ranura):
    """Una recarga o una segunda pestaña NO es «ocupado».

    Se compara por sujeto y no por conexión a propósito: comparar por socket
    convertiría un F5 en un bloqueo de diez minutos contra el propio robot.
    """
    ranura.tomar('ana', 's1', 100.0)
    assert ranura.es_de('ana')
    assert not ranura.es_de('luis')
    # Y las ops de control sí las acepta de su dueña.
    assert interpretar({'op': 'atriz_parar'}, ranura, 'ana').ok


def test_las_ops_de_control_de_OTRO_se_rechazan(ranura):
    """No se para el programa de otro, ni se le escribe en la entrada."""
    ranura.tomar('ana', 's1', 100.0)
    for op in ['atriz_parar', 'atriz_stdin', 'atriz_signal', 'atriz_tamano']:
        o = interpretar({'op': op, 'texto': 'x', 'senal': 'SIGINT',
                         'columnas': 80, 'filas': 24}, ranura, 'luis')
        assert not o.ok, op
        assert o.codigo == 'NO_ES_TUYO', op
        assert 'ana' in o.motivo


def test_sin_nada_corriendo_las_ops_de_control_se_rechazan(ranura):
    for op in ['atriz_parar', 'atriz_stdin', 'atriz_signal']:
        o = interpretar({'op': op, 'texto': 'x', 'senal': 'SIGINT'}, ranura, 'ana')
        assert not o.ok, op
        assert o.codigo == 'NADA_CORRIENDO', op


def test_codigo_vacio_o_enorme_se_rechaza(ranura):
    assert interpretar(exec_de(''), ranura, 'ana').codigo == 'CODIGO_VACIO'
    assert interpretar(exec_de('   \n  '), ranura, 'ana').codigo == 'CODIGO_VACIO'
    enorme = 'x' * (TOPE_CODIGO_BYTES + 1)
    assert interpretar(exec_de(enorme), ranura, 'ana').codigo == 'CODIGO_ENORME'


def test_los_bytes_de_control_NO_pasan_por_la_entrada(ranura):
    """Un \\x03 en stdin sería un Ctrl-C encubierto.

    Las señales tienen su propia op para que la pantalla pueda decir cuál se
    mandó y cuándo — que es exactamente lo que la práctica 99 estudia. Colarlas
    por la entrada las volvería invisibles.
    """
    ranura.tomar('ana', 's1', 100.0)
    for malo in ['\x03', 'hola\x03', '\x1b[A', '\x04']:
        o = interpretar({'op': 'atriz_stdin', 'texto': malo}, ranura, 'ana')
        assert not o.ok, repr(malo)
        assert o.codigo == 'CONTROL_EN_ENTRADA'
    # Pero el salto de línea y el tabulador SÍ: son entrada normal.
    for bueno in ['45.0\n', 'hola\tmundo', '']:
        assert interpretar({'op': 'atriz_stdin', 'texto': bueno}, ranura, 'ana').ok, repr(bueno)


def test_una_senal_fuera_de_la_lista_se_rechaza(ranura):
    ranura.tomar('ana', 's1', 100.0)
    o = interpretar({'op': 'atriz_signal', 'senal': 'SIGSTOP'}, ranura, 'ana')
    assert not o.ok
    assert o.codigo == 'SENAL_DESCONOCIDA'


def test_las_CINCO_senales_de_la_practica_99_estan(ranura):
    """SIGINT repetido, SIGQUIT, SIGTERM, SIGHUP y el kill -9 del ejercicio 5.

    Recortar la lista a «las seguras» dejaría la práctica sin su instrumento: son
    su objeto de estudio, no un menú de experto.
    """
    ranura.tomar('ana', 's1', 100.0)
    for s in SENALES:
        assert interpretar({'op': 'atriz_signal', 'senal': s}, ranura, 'ana').ok, s
    assert set(SENALES) == {'SIGINT', 'SIGQUIT', 'SIGTERM', 'SIGHUP', 'SIGKILL'}


# ═══════════════════════════════════════════════════════════════════════════
# Nombres de fichero: no se compone una ruta con texto del cliente
# ═══════════════════════════════════════════════════════════════════════════
def test_nombre_seguro_acepta_las_practicas_de_verdad():
    for bueno in ['01_avanzar.py', '99_test_ctrl_c.py', 'seguidor_linea_pid_demo.py',
                  '90_template.py', 'mi_programa.py']:
        assert nombre_seguro(bueno) == bueno, bueno


def test_nombre_seguro_rechaza_todo_lo_que_sea_una_ruta():
    for malo in ['../atriz.py', '/etc/passwd', 'a/b.py', 'a\\b.py', '..', '.py',
                 'sin_extension', 'x.py\x00', 'x.txt', '', None, 7, 'x' * 200 + '.py']:
        assert nombre_seguro(malo) is None, repr(malo)


def test_leer_un_fichero_con_nombre_malo_se_rechaza(ranura):
    o = interpretar({'op': 'atriz_leer', 'fichero': '../../etc/passwd'}, ranura, 'ana')
    assert not o.ok
    assert o.codigo == 'NOMBRE_MALO'


# ═══════════════════════════════════════════════════════════════════════════
# El tope de pared
# ═══════════════════════════════════════════════════════════════════════════
def test_recortar_el_tope_lo_DICE(ranura):
    """Recortar en silencio es mentir: el alumno pide media hora, se le dan diez
    minutos, y cuando su práctica muere no tiene forma de saber por qué."""
    tope, recortado = recortar_tope(99999)
    assert tope == TOPE_PARED_MAX_S
    assert recortado is True
    tope, recortado = recortar_tope(1)
    assert tope == TOPE_PARED_MIN_S
    assert recortado is True
    # Un valor razonable no se toca, y no se anuncia un recorte que no hubo.
    assert recortar_tope(300) == (300, False)


def test_un_tope_absurdo_cae_al_de_por_defecto_sin_lanzar():
    for malo in [None, 'diez', [], True]:
        assert recortar_tope(malo) == (TOPE_PARED_S, False), repr(malo)


def test_el_tope_por_defecto_aguanta_las_practicas_de_bucle_infinito():
    """Ocho de las quince prácticas son `while True` legítimos.

    El diseño original decía «dos». Contadas hoy en scripts/estudiantes/ son ocho
    (05, 11, las cinco de IR y el seguidor). Un tope corto las mataría a mitad de
    clase, y el alumno culparía a la plataforma en vez de aprender.
    """
    assert TOPE_PARED_S >= 600
    assert TOPE_PARED_MAX_S >= 1800


# ═══════════════════════════════════════════════════════════════════════════
# La salida: recortar SIN CALLAR
# ═══════════════════════════════════════════════════════════════════════════
def test_el_limitador_deja_pasar_lo_normal():
    lim = Limitador()
    assert lim.admitir('hola\n') == 'hola\n'
    assert lim.lineas_descartadas == 0


def test_al_agotarse_cuenta_lo_descartado_y_no_lo_calla():
    lim = Limitador(tope_total=20, tope_linea=1000)
    lim.admitir('x' * 30 + '\n')
    assert lim.agotado
    lim.admitir('a\nb\nc\n')
    assert lim.lineas_descartadas >= 3
    assert lim.bytes_descartados > 0


def test_una_linea_larguisima_se_RECORTA_y_no_se_tira():
    """Casi siempre es un `print` de una lista enorme, y su principio dice qué es."""
    lim = Limitador(tope_total=10_000, tope_linea=50)
    salida = lim.admitir('y' * 500 + '\n')
    assert 'recortada' in salida
    assert salida.startswith('y' * 50)
    assert lim.bytes_descartados > 0


# ═══════════════════════════════════════════════════════════════════════════
# 🔴 UTF-8 partido entre dos lecturas del PTY
# ═══════════════════════════════════════════════════════════════════════════
def test_una_eñe_partida_entre_dos_lecturas_no_se_corrompe():
    """El PTY se lee por bloques. Decodificar cada trozo por su cuenta daría un
    carácter de reemplazo EN MITAD DE UNA PALABRA, y el alumno vería su salida
    corrompida sin que nada esté roto."""
    d = Decodificador()
    crudo = 'niño\n'.encode('utf8')
    # Se parte justo dentro de la `ñ` (dos bytes).
    primera = d.alimentar(crudo[:3])
    segunda = d.alimentar(crudo[3:])
    assert primera + segunda == 'niño\n'
    assert '�' not in primera + segunda


def test_el_decodificador_no_se_traga_lo_que_ya_esta_completo():
    d = Decodificador()
    assert d.alimentar(b'hola') == 'hola'
    assert d.alimentar(b' mundo\n') == ' mundo\n'


# ═══════════════════════════════════════════════════════════════════════════
# 🔴 LOS CUATRO PELDAÑOS
# ═══════════════════════════════════════════════════════════════════════════
def test_el_primer_peldano_es_SIGINT_y_no_es_cortesia():
    """Es el camino que `atriz.py` captura: recorre `cerrar()`, para el robot y
    apaga el barrido si lo encendió él. Empezar por SIGKILL dejaría el robot en
    marcha y el X2 girando a 11,8 Hz — por dieciséis, hasta que alguien lo note."""
    assert plan_de_parada((), 100.0, 100.0) == 'SIGINT'


def test_los_cuatro_peldanos_en_orden_y_con_sus_esperas():
    t0 = 100.0
    assert plan_de_parada(('SIGINT',), t0, t0 + 1) == 'ESPERAR'
    assert plan_de_parada(('SIGINT',), t0, t0 + ESPERA_TRAS_SIGINT_S) == 'SIGTERM'
    assert plan_de_parada(('SIGINT', 'SIGTERM'), t0, t0 + 11) == 'ESPERAR'
    assert plan_de_parada(('SIGINT', 'SIGTERM'), t0, t0 + 30) == 'SIGKILL'
    assert plan_de_parada(('SIGINT', 'SIGTERM', 'SIGKILL'), t0, t0 + 60) == 'HECHO'


def test_la_espera_tras_SIGINT_no_corta_lo_que_la_practica_99_mide():
    """`atriz.py` puede estar dentro de un `avanzar()` con su propio plazo, y la
    99 mide exactamente lo que el robot recorre DESPUÉS del Ctrl-C. Cortar antes
    de que ese camino termine borraría lo que se quiere enseñar."""
    assert ESPERA_TRAS_SIGINT_S >= 10.0


# ═══════════════════════════════════════════════════════════════════════════
# El entorno del hijo
# ═══════════════════════════════════════════════════════════════════════════
def test_el_entorno_conserva_lo_que_atriz_py_necesita_para_importar():
    """Sin AMENT_PREFIX_PATH ni LD_LIBRARY_PATH, `import rclpy` falla y con él
    `atriz.py` entero. Y sin ROS_DOMAIN_ID el guion hablaría con el dominio 0,
    que es el de nadie: el robot no se enteraría de nada."""
    padre = {
        'PATH': '/usr/bin', 'HOME': '/home/sphero', 'LD_LIBRARY_PATH': '/opt/ros/jazzy/lib',
        'AMENT_PREFIX_PATH': '/opt/ros/jazzy', 'ROS_DOMAIN_ID': '7',
        'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp', 'CMAKE_PREFIX_PATH': '/opt/ros/jazzy',
        'SECRETO_DEL_PORTATIL': 'no deberia viajar', 'GITHUB_TOKEN': 'tampoco',
    }
    hijo = entorno_de_ejecucion(padre, '/run/atriz/s1')
    for necesaria in ['PATH', 'HOME', 'LD_LIBRARY_PATH', 'AMENT_PREFIX_PATH',
                      'ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION', 'CMAKE_PREFIX_PATH']:
        assert hijo[necesaria] == padre[necesaria], necesaria


def test_el_entorno_DESCARTA_lo_que_no_reconoce():
    hijo = entorno_de_ejecucion({'PATH': '/usr/bin', 'GITHUB_TOKEN': 'ghp_loquesea'},
                                '/run/atriz/s1')
    assert 'GITHUB_TOKEN' not in hijo


def test_PYTHONPATH_apunta_a_la_sesion_y_no_al_directorio_de_practicas():
    """El plan decía «PYTHONPATH en solo lectura». No se puede: el agente corre
    como sphero y ese directorio es de sphero. Se copia `atriz.py` a la carpeta de
    la sesión en cada lanzamiento, que es más fuerte: se regenera cada vez."""
    hijo = entorno_de_ejecucion({'PATH': '/usr/bin'}, '/run/atriz/s1')
    assert hijo['PYTHONPATH'] == '/run/atriz/s1'


def test_PYTHONUNBUFFERED_va_puesto():
    """Sin esto `print()` sale a bloques AUNQUE haya PTY, y la pantalla se
    congelaría con el robot en marcha — que es el requisito 1 entero."""
    assert entorno_de_ejecucion({}, '/run/atriz/s1')['PYTHONUNBUFFERED'] == '1'


def test_la_huella_cambia_con_el_codigo():
    """Sirve para que la pantalla diga «has cambiado el texto desde que lanzaste»
    en vez de dejar creer que lo que corre es lo que se ve."""
    assert huella_de('print(1)') != huella_de('print(2)')
    assert huella_de('print(1)') == huella_de('print(1)')


def test_entorno_conserva_el_pythonpath_de_ros_DETRAS_de_la_sesion():
    """🔴 CAZADO EN VIVO (2026-08-14, práctica 05 por el agente): el entorno
    pisaba PYTHONPATH entero con la carpeta de sesión y el guion del alumno
    moría en `import rclpy` — porque es PYTHONPATH (no AMENT_PREFIX_PATH) quien
    hace visible /opt/ros/.../site-packages. La carpeta de sesión va PRIMERO
    (la copia de atriz.py debe ganar), y el resto del PYTHONPATH del agente,
    detrás."""
    import os
    #: ⚠️ EL SEPARADOR SE ARMA CON `os.pathsep`, NO SE ESCRIBE A MANO.
    #:
    #: Esta prueba llegó del robot con un `:` fijo dentro del valor, y **pasaba
    #: en la Pi y fallaba en el PC**: allí `os.pathsep` es `;`, la cadena no se
    #: partía, y el `in partes` daba falso **sobre un código correcto** — el
    #: código sí usa `os.pathsep`.
    #:
    #: 🔴 Importa más de lo que parece: el núcleo está separado del PTY
    #:    justamente para poder correr donde no hay robot. Una prueba suya que
    #:    solo pasa en Linux devuelve el fichero a depender de la Pi, que es lo
    #:    que la separación existe para evitar.
    ros = '/opt/ros/jazzy/lib/python3.12/site-packages'
    padre = {'PYTHONPATH': os.pathsep.join([ros, '/otro'])}
    hijo = entorno_de_ejecucion(padre, '/run/atriz/abc')
    partes = hijo['PYTHONPATH'].split(os.pathsep)
    assert partes[0] == '/run/atriz/abc', 'la sesión tiene que ir PRIMERO'
    assert ros in partes
    assert '/otro' in partes


def test_entorno_sin_pythonpath_del_padre_queda_solo_la_sesion():
    hijo = entorno_de_ejecucion({}, '/run/atriz/abc')
    assert hijo['PYTHONPATH'] == '/run/atriz/abc'
