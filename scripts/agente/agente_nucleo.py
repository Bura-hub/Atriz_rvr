#!/usr/bin/env python3
"""EL NÚCLEO DEL AGENTE DE SESIÓN. PURO: sin tornado, sin `pty`, sin `fork`.

════════════════════════════════════════════════════════════════════════════
POR QUÉ ESTÁ SEPARADO DEL SERVICIO
════════════════════════════════════════════════════════════════════════════
Es el mismo patrón que `atriz_testigo.py`, y paga por el mismo motivo: **esto se
puede probar desde el PC, y el servicio no**. `tornado` solo está en la Pi, `pty`
no existe en Windows y `fork` tampoco.

Y no es una comodidad de desarrollo. **Los caminos que importan son los
NEGATIVOS** —una op desconocida, la ranura ocupada, no eres el dueño, código de
un mega, bytes de control en la entrada— y todos viven aquí, donde se pueden
recorrer sin un robot delante.

════════════════════════════════════════════════════════════════════════════
🔴 LO QUE ESTE AGENTE **NO** ES
════════════════════════════════════════════════════════════════════════════
**No es una frontera de seguridad.** El programa del alumno corre como `sphero`,
igual que el driver: puede escribir donde `sphero` escriba, abrir sus propios
sockets, e `import rclpy` para publicar en `/cmd_vel` **saltándose la capa de
seguridad**. Con `rclpy` nativo alcanza `raw_motors`, `move_timed` y
`set_ir_mode('following')`, que es justo lo que la lista blanca de rosbridge
cierra para el navegador.

Lo que este agente protege son los **errores honestos**: el guion que se queda
colgado, el que llena la pantalla, el que se olvida de apagar el barrido. Contra
alguien hostil no protege, y eso va escrito aquí y en la pantalla.
"""

from __future__ import annotations

import hashlib
import re
from dataclasses import dataclass, field

#: Estados de la ranura de ejecución. Uno por robot, no uno por conexión.
LIBRE = 'LIBRE'
ARRANCANDO = 'ARRANCANDO'
CORRIENDO = 'CORRIENDO'
PARANDO = 'PARANDO'
TERMINADO = 'TERMINADO'

#: Las señales que el alumno puede mandar. 🔴 Las cinco están porque las cinco
#: son el OBJETO DE ESTUDIO de la práctica 99: SIGINT repetido, SIGQUIT, SIGTERM,
#: SIGHUP, y su ejercicio 5 pide `kill -9` desde otra terminal. Recortar la lista
#: a «las seguras» dejaría la práctica sin instrumento.
SENALES = ('SIGINT', 'SIGQUIT', 'SIGTERM', 'SIGHUP', 'SIGKILL')

#: 64 KiB. Las prácticas del curso son de ~30 líneas; esto son ~1500. No es un
#: límite de seguridad —quien quiera hacer daño lo hace en tres— es lo que impide
#: que un pegado accidental de un fichero enorme viaje entero por el WiFi.
TOPE_CODIGO_BYTES = 64 * 1024

#: 🔴 EL TOPE DE PARED, Y NO PUEDE SER CORTO.
#:
#: El diseño original decía «dos prácticas son `while True` legítimos». Contadas
#: hoy en `scripts/estudiantes/`: **OCHO de quince** (05, 11, las cinco de IR y el
#: seguidor). Un tope que mate la práctica 22 a mitad de clase es peor que no
#: tenerlo: el alumno no aprende nada y culpa a la plataforma.
#:
#: → 10 minutos por defecto, prorrogable desde la pantalla, y **con cuenta atrás
#:   visible**. Nunca en silencio.
TOPE_PARED_S = 600
TOPE_PARED_MIN_S = 30
TOPE_PARED_MAX_S = 1800

#: Tope de salida. `11_sensor_avanzado.py` imprime en bucle: sin tope, una tarde
#: olvidada llena la memoria del navegador. Se recorta y **se dice cuánto**.
TOPE_SALIDA_BYTES = 2 * 1024 * 1024
TOPE_LINEA_BYTES = 4 * 1024

#: Nombre de fichero aceptable. Sin rutas, sin `..`, y acabado en `.py`.
_NOMBRE = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}\.py$')


# ═══════════════════════════════════════════════════════════════════════════
# La ranura: quién tiene el robot
# ═══════════════════════════════════════════════════════════════════════════
@dataclass
class Ocupacion:
    """Quién está ejecutando y desde cuándo. Nunca un booleano.

    🔴 Un booleano obligaría a la pantalla a decir «ocupado» y nada más, y el
       segundo alumno se quedaría mirando sin saber a quién buscar. Con dos
       robots por mesa, «lo tiene ana desde hace 3 min» es la diferencia entre
       esperar y ir a preguntar.
    """

    sujeto: str
    sid: str
    desde_s: float
    estado: str = ARRANCANDO
    pid: int | None = None
    nombre: str = ''
    huella: str = ''


class Ranura:
    """UNA ejecución por robot, y el porqué no es una política: es física.

    `atriz.py` crea `Node('atriz_alumno')` con **nombre fijo**. Dos programas a la
    vez son dos nodos homónimos en el mismo dominio DDS, y el resultado no es un
    error legible: son dos guiones peleándose por el mismo robot, que desde fuera
    se ve como «el robot no obedece».
    """

    def __init__(self) -> None:
        self._quien: Ocupacion | None = None

    @property
    def ocupacion(self) -> Ocupacion | None:
        return self._quien

    def libre(self) -> bool:
        return self._quien is None

    def tomar(self, sujeto: str, sid: str, ahora_s: float) -> Ocupacion:
        if self._quien is not None:
            raise RuntimeError('la ranura ya está tomada; se comprueba con `puede_tomar` antes')
        self._quien = Ocupacion(sujeto=sujeto, sid=sid, desde_s=ahora_s)
        return self._quien

    def soltar(self) -> None:
        self._quien = None

    def es_de(self, sujeto: str) -> bool:
        """¿Es de este sujeto la ejecución en marcha?

        ⚠️ Se compara por SUJETO y no por conexión, a propósito: la misma persona
           con dos pestañas, o que recarga, **no está ocupando su propio robot**.
           Comparar por socket convertiría una recarga en un bloqueo de 10 min.
        """
        return self._quien is not None and self._quien.sujeto == sujeto


# ═══════════════════════════════════════════════════════════════════════════
# Lo que se acepta del cliente
# ═══════════════════════════════════════════════════════════════════════════
@dataclass
class Orden:
    """Lo que el núcleo decide hacer con un mensaje. Nunca lanza hacia arriba."""

    op: str
    ok: bool
    codigo: str = ''
    motivo: str = ''
    datos: dict = field(default_factory=dict)


def _rechazo(op: str, codigo: str, motivo: str) -> Orden:
    return Orden(op=op, ok=False, codigo=codigo, motivo=motivo)


def nombre_seguro(nombre: object) -> str | None:
    """El nombre de un fichero de prácticas, o `None`.

    🔴 NO se compone una ruta con texto del cliente en ninguna parte del agente.
       Este nombre solo sirve para BUSCARLO en la lista que el propio agente
       produjo; nunca para `join()`. Es la diferencia entre validar y confiar.
    """
    if not isinstance(nombre, str):
        return None
    if not _NOMBRE.match(nombre):
        return None
    # Cinturón y tirantes: el patrón ya lo impide, pero `..` merece una negativa
    # explícita para que se vea en la lectura y en las pruebas.
    if '..' in nombre or '/' in nombre or '\\' in nombre:
        return None
    return nombre


def huella_de(codigo: str) -> str:
    """Doce hexadecimales de sha256.

    Sirve para que la pantalla pueda decir **«has cambiado el texto desde que
    lanzaste»** en vez de dejar creer que lo que corre es lo que se ve. No es
    seguridad: es honestidad sobre lo que se está ejecutando.
    """
    return hashlib.sha256(codigo.encode('utf8')).hexdigest()[:12]


def recortar_tope(pedido: object) -> tuple[int, bool]:
    """Devuelve `(tope, se_recorto)`.

    🔴 Devuelve DOS cosas porque recortar en silencio es mentir: el alumno pide
       media hora, se le dan diez minutos, y cuando su práctica muere a los diez
       no tiene forma de saber por qué. El agente lo dice y la pantalla lo pinta.
    """
    if not isinstance(pedido, (int, float)) or isinstance(pedido, bool):
        return TOPE_PARED_S, False
    entero = int(pedido)
    recortado = max(TOPE_PARED_MIN_S, min(TOPE_PARED_MAX_S, entero))
    return recortado, recortado != entero


def interpretar(mensaje: object, ranura: Ranura, sujeto: str) -> Orden:
    """El único sitio por el que entra lo que manda el cliente.

    Devuelve una `Orden`, nunca lanza: un mensaje raro es un camino NORMAL —viene
    de fuera— y no una excepción. Con `throw` bastaría con olvidar un `try` para
    que un JSON tonto tumbara el agente y con él la ejecución de alguien.
    """
    if not isinstance(mensaje, dict):
        return _rechazo('?', 'MAL_FORMADO', 'el mensaje no es un objeto')

    op = mensaje.get('op')
    if not isinstance(op, str) or op == '':
        return _rechazo('?', 'MAL_FORMADO', 'el mensaje no dice qué op es')

    if op == 'atriz_adjuntar' or op == 'atriz_listar':
        return Orden(op=op, ok=True)

    if op == 'atriz_leer':
        nombre = nombre_seguro(mensaje.get('fichero'))
        if nombre is None:
            return _rechazo(op, 'NOMBRE_MALO',
                            'ese nombre de fichero no es de una práctica de este robot')
        return Orden(op=op, ok=True, datos={'fichero': nombre})

    if op == 'atriz_exec':
        codigo = mensaje.get('codigo')
        if not isinstance(codigo, str):
            return _rechazo(op, 'MAL_FORMADO', 'falta el código a ejecutar')
        if codigo.strip() == '':
            return _rechazo(op, 'CODIGO_VACIO', 'no hay nada que ejecutar')
        if len(codigo.encode('utf8')) > TOPE_CODIGO_BYTES:
            return _rechazo(op, 'CODIGO_ENORME',
                            f'el código pasa de {TOPE_CODIGO_BYTES // 1024} KiB')
        if not ranura.libre():
            o = ranura.ocupacion
            assert o is not None
            # 🔴 El motivo lleva QUIÉN y DESDE CUÁNDO. Ver `Ocupacion`.
            return _rechazo(op, 'OCUPADO', f'lo tiene {o.sujeto}')
        tope, recortado = recortar_tope(mensaje.get('tope_pared_s'))
        nombre = nombre_seguro(mensaje.get('nombre')) or 'mi_programa.py'
        return Orden(op=op, ok=True, datos={
            'codigo': codigo, 'nombre': nombre, 'tope_pared_s': tope,
            'tope_recortado': recortado, 'huella': huella_de(codigo),
        })

    if op in ('atriz_stdin', 'atriz_signal', 'atriz_parar', 'atriz_tamano'):
        if ranura.libre():
            return _rechazo(op, 'NADA_CORRIENDO', 'no hay ningún programa corriendo')
        if not ranura.es_de(sujeto):
            o = ranura.ocupacion
            assert o is not None
            return _rechazo(op, 'NO_ES_TUYO', f'el programa que corre es de {o.sujeto}')

        if op == 'atriz_stdin':
            texto = mensaje.get('texto')
            if not isinstance(texto, str):
                return _rechazo(op, 'MAL_FORMADO', 'falta el texto')
            # 🔴 Los bytes de control NO viajan por aquí. Un `\x03` metido en la
            #    entrada sería un Ctrl-C encubierto: las señales tienen su propia
            #    op para que la pantalla pueda decir cuál mandó y cuándo, que es
            #    lo que la práctica 99 estudia. Se permite `\n` y `\t`.
            if any(ord(c) < 32 and c not in '\n\t' for c in texto):
                return _rechazo(op, 'CONTROL_EN_ENTRADA',
                                'la entrada no admite caracteres de control: las señales van aparte')
            return Orden(op=op, ok=True, datos={'texto': texto})

        if op == 'atriz_signal':
            senal = mensaje.get('senal')
            if senal not in SENALES:
                return _rechazo(op, 'SENAL_DESCONOCIDA',
                                f'esa señal no está entre {", ".join(SENALES)}')
            return Orden(op=op, ok=True, datos={'senal': senal})

        if op == 'atriz_tamano':
            cols, filas = mensaje.get('columnas'), mensaje.get('filas')
            if not (isinstance(cols, int) and isinstance(filas, int)):
                return _rechazo(op, 'MAL_FORMADO', 'faltan columnas y filas')
            if not (2 <= cols <= 500 and 2 <= filas <= 200):
                return _rechazo(op, 'TAMANO_ABSURDO', 'ese tamaño de terminal no es creíble')
            return Orden(op=op, ok=True, datos={'columnas': cols, 'filas': filas})

        return Orden(op=op, ok=True)

    return _rechazo(op, 'OP_DESCONOCIDA', f'no sé qué es «{op}»')


# ═══════════════════════════════════════════════════════════════════════════
# El tope de salida
# ═══════════════════════════════════════════════════════════════════════════
class Limitador:
    """Recorta la salida y **cuenta lo recortado**.

    🔴 NUNCA EN SILENCIO. Un programa que imprime en bucle y una pantalla que
       deja de actualizarse se ven igual desde fuera, y este proyecto tiene medido
       lo que cuesta esa confusión. Con el contador, «faltan 4210 líneas» es un
       dato; sin él, es un fallo de la web.
    """

    def __init__(self, tope_total: int = TOPE_SALIDA_BYTES,
                 tope_linea: int = TOPE_LINEA_BYTES) -> None:
        self.tope_total = tope_total
        self.tope_linea = tope_linea
        self.emitidos = 0
        self.lineas_descartadas = 0
        self.bytes_descartados = 0

    @property
    def agotado(self) -> bool:
        return self.emitidos >= self.tope_total

    def admitir(self, trozo: str) -> str:
        """Devuelve lo que hay que emitir; el resto queda contado."""
        if trozo == '':
            return ''
        if self.agotado:
            self.bytes_descartados += len(trozo.encode('utf8'))
            self.lineas_descartadas += trozo.count('\n')
            return ''

        salida = []
        for linea in trozo.splitlines(keepends=True):
            cuerpo = linea.rstrip('\n')
            if len(cuerpo.encode('utf8')) > self.tope_linea:
                # Una línea larguísima se recorta, no se tira: casi siempre es un
                # `print` de una lista enorme, y su principio dice qué es.
                cortada = cuerpo.encode('utf8')[:self.tope_linea].decode('utf8', 'ignore')
                self.bytes_descartados += len(cuerpo.encode('utf8')) - self.tope_linea
                linea = cortada + ' …[línea recortada]' + ('\n' if linea.endswith('\n') else '')
            salida.append(linea)

        texto = ''.join(salida)
        cabe = self.tope_total - self.emitidos
        crudo = texto.encode('utf8')
        if len(crudo) > cabe:
            texto = crudo[:cabe].decode('utf8', 'ignore')
            sobra = crudo[cabe:]
            self.bytes_descartados += len(sobra)
            self.lineas_descartadas += sobra.count(b'\n')
        self.emitidos += len(texto.encode('utf8'))
        return texto


class Decodificador:
    """UTF-8 a través de lecturas troceadas.

    🔴 El PTY se lee por bloques, así que una `ñ` puede quedar partida entre dos
       lecturas. Decodificar cada trozo por su cuenta produciría un carácter de
       reemplazo **en mitad de una palabra**, y el alumno vería su propia salida
       corrompida sin que nada esté roto.
    """

    def __init__(self) -> None:
        self._resto = b''

    def alimentar(self, crudo: bytes) -> str:
        datos = self._resto + crudo
        # Retroceder hasta el principio del último carácter incompleto.
        corte = len(datos)
        for atras in range(1, min(4, len(datos)) + 1):
            b = datos[-atras]
            if b < 0x80 or b >= 0xC0:            # inicio de carácter o ASCII
                largo = 1 if b < 0x80 else (2 if b >= 0xC0 and b < 0xE0 else (3 if b < 0xF0 else 4))
                if atras < largo:
                    corte = len(datos) - atras
                break
        self._resto = datos[corte:]
        return datos[:corte].decode('utf8', 'replace')


# ═══════════════════════════════════════════════════════════════════════════
# 🔴 LA PARADA, EN CUATRO PELDAÑOS
# ═══════════════════════════════════════════════════════════════════════════
#: Lo que se espera tras cada peldaño, en segundos.
ESPERA_TRAS_SIGINT_S = 10.0
ESPERA_TRAS_SIGTERM_S = 5.0


def plan_de_parada(enviadas: tuple[str, ...], desde_s: float, ahora_s: float) -> str:
    """Qué toca hacer ahora. `enviadas` es lo ya mandado, en orden.

    🔴 SIGINT PRIMERO, Y NO ES CORTESÍA. Es el camino que `atriz.py` ya captura:
       recorre `cerrar()`, que para el robot y apaga el barrido si lo encendió él.
       Empezar por SIGKILL dejaría el robot en marcha y el X2 girando a 11,8 Hz —
       por dieciséis robots, hasta que alguien lo notara.

    ⚠️ Los diez segundos del primer peldaño no son un número redondo elegido a
       ojo: `atriz.py` puede estar dentro de un `avanzar()` con su propio plazo, y
       la práctica 99 mide exactamente lo que el robot recorre después del Ctrl-C.
       Cortar antes de que termine ese camino borraría lo que se quiere enseñar.
    """
    espera = ahora_s - desde_s
    if 'SIGINT' not in enviadas:
        return 'SIGINT'
    if espera < ESPERA_TRAS_SIGINT_S:
        return 'ESPERAR'
    if 'SIGTERM' not in enviadas:
        return 'SIGTERM'
    if espera < ESPERA_TRAS_SIGINT_S + ESPERA_TRAS_SIGTERM_S:
        return 'ESPERAR'
    if 'SIGKILL' not in enviadas:
        return 'SIGKILL'
    return 'HECHO'


# ═══════════════════════════════════════════════════════════════════════════
# El entorno del hijo
# ═══════════════════════════════════════════════════════════════════════════
#: Lo que se conserva del entorno del agente. Todo lo demás se descarta.
#:
#: 🔴 Las de ROS hacen falta de verdad: sin `AMENT_PREFIX_PATH` ni
#:    `LD_LIBRARY_PATH`, `import rclpy` falla y con él `atriz.py` entero. Y
#:    `ROS_DOMAIN_ID` es lo que mete al guion del alumno en el grafo de SU robot:
#:    sin él hablaría con el dominio 0, que es el de nadie.
_CONSERVAR = (
    'PATH', 'HOME', 'USER', 'LOGNAME', 'LANG', 'SHELL', 'PWD',
    'LD_LIBRARY_PATH', 'PYTHONHOME',
)
_CONSERVAR_PREFIJO = ('ROS_', 'RMW_', 'AMENT_', 'COLCON_', 'CMAKE_PREFIX_PATH', 'LC_')


def entorno_de_ejecucion(padre: dict, dir_sesion: str) -> dict:
    """El entorno del programa del alumno.

    🔴 `PYTHONPATH` apunta a la CARPETA DE LA SESIÓN, donde el agente deja una
       copia de `atriz.py`, y **no** al directorio de prácticas.

       El plan decía «`PYTHONPATH` en solo lectura». No se puede: el agente corre
       como `sphero` y ese directorio es de `sphero` — mismo usuario, mismo
       derecho de escritura. Copiar la biblioteca en cada lanzamiento consigue el
       objetivo declarado —que un alumno no rompa `atriz.py` para el siguiente—
       y es más fuerte: se regenera cada vez.

    ⚠️ Lo que NO cierra: que el guion escriba en el directorio real con `open()`.
       Eso solo lo cierra un montaje de solo lectura, y va en endurecimiento.
    """
    hijo = {k: v for k, v in padre.items()
            if k in _CONSERVAR or k.startswith(_CONSERVAR_PREFIJO)}
    # 🔴 CAZADO EN VIVO el 2026-08-14 (evidencia 117): aquí se PISABA el
    #    PYTHONPATH entero y la práctica 05 moría en `import rclpy` a los 0 s —
    #    porque es PYTHONPATH quien hace visible /opt/ros/.../site-packages
    #    (AMENT_PREFIX_PATH no importa módulos; el comentario de arriba lo
    #    afirmaba y era falso). La carpeta de sesión va PRIMERO —la copia de
    #    atriz.py debe ganar a cualquier otra— y el PYTHONPATH del agente,
    #    detrás. Ninguna prueba pura podía verlo: lo vio la práctica real.
    import os as _os
    heredado = padre.get('PYTHONPATH', '')
    hijo['PYTHONPATH'] = (dir_sesion + _os.pathsep + heredado) if heredado \
        else dir_sesion
    # Sin esto `print()` sale a bloques aunque haya PTY, y la pantalla se
    # congelaría con el robot en marcha — que es el requisito 1 entero.
    hijo['PYTHONUNBUFFERED'] = '1'
    # `dumb` evita que una biblioteca decida pintar colores o mover el cursor: lo
    # que llega al navegador es texto, no una pantalla de terminal.
    hijo['TERM'] = 'dumb'
    return hijo
