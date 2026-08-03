#!/usr/bin/env python3
"""La biblioteca del laboratorio Atriz — lo que usan las prácticas del curso.

    from atriz import Robot

    with Robot() as robot:
        robot.avanzar(0.20, 3)      # m/s durante segundos
        robot.girar(90)             # grados; positivo = a la izquierda

No hace falta instalar nada: este fichero vive junto a los scripts.

═══════════════════════════════════════════════════════════════════════════════
POR QUÉ EXISTE, EN UNA LÍNEA
═══════════════════════════════════════════════════════════════════════════════
Un programa escrito contra `rclpy` a pelo tiene que acertar, cada vez y sin
ayuda, en siete cosas que este laboratorio ha aprendido a base de fallos. Aquí
se aciertan una vez, y el alumno escribe robótica.

Están documentadas una a una en `03_operacion/API_LABORATORIO.md`.
"""
import math

# ═══════════════════════════════════════════════════════════════════════════
# CONSTANTES — cada una tiene una medida detrás. No se cambian sin otra.
# ═══════════════════════════════════════════════════════════════════════════

# 🔴 EL TOPIC. `/cmd_vel` es la SALIDA del collision_monitor: publicar ahí
#    FUNCIONA y salta la capa de seguridad entera, sin un solo aviso. Es el
#    agujero más silencioso del sistema, y los diez scripts de ROS 1 lo hacían.
TOPIC_MANDO = '/cmd_vel_raw'

VEL_MAX = 0.40        # m/s — meseta REAL medida: 0.401 comandando 0.40 (2026-07-31)
VEL_GIRO_MAX = 2.0    # rad/s — 99-102 % del comandado en las cuatro medidas
TIEMPO_MAX = 10.0     # s por llamada — decisión de diseño, no una medida
GRADOS_MAX = 720.0    # ° por llamada — ídem

# 🔴 El watchdog del driver corta a los 0.3 s sin `cmd_vel`. Un `sleep(3)` entre
#    dos publicaciones deja al robot PARADO casi todo el tiempo, y el alumno ve
#    un robot que «no obedece». Hay que republicar más rápido que eso.
RITMO_HZ = 10.0


class ErrorAtriz(Exception):
    """Algo del laboratorio no está como debería. El mensaje dice qué hacer."""


# ═══════════════════════════════════════════════════════════════════════════
# FUNCIONES PURAS — sin ROS, sin robot. Tienen tests en atriz_migracion.
# ═══════════════════════════════════════════════════════════════════════════

def limitar(valor, tope, nombre, unidad):
    """Recorta `valor` a ±`tope`. Devuelve (valor, aviso o None).

    Recorta en vez de lanzar, y AVISA en vez de recortar en silencio: un
    programa que se muere a mitad deja el robot conduciendo, y uno que recorta
    calladito enseña al alumno que su número se aplicó.
    """
    if abs(valor) <= tope:
        return valor, None
    recortado = math.copysign(tope, valor)
    return recortado, (
        f'AVISO: {nombre} {valor:g} {unidad} pasa del limite del laboratorio '
        f'({tope:g} {unidad}); se usa {recortado:g}.')


def normalizar(rad):
    """Lleva un ángulo al intervalo (−π, π]."""
    angulo = math.fmod(rad, 2.0 * math.pi)
    if angulo > math.pi:
        angulo -= 2.0 * math.pi
    elif angulo <= -math.pi:
        angulo += 2.0 * math.pi
    return angulo


def acumular(yaw_anterior, yaw_actual, acumulado):
    """Suma el INCREMENTO de rumbo, normalizado. Nunca el yaw absoluto.

    🔴 `atan2` devuelve −π..π, así que una vuelta entera leída en absoluto
       vuelve al punto de partida y se lee como 0°. Acumular el incremento
       normalizado es lo que hace que 360° sean 360°.
    """
    return acumulado + normalizar(yaw_actual - yaw_anterior)


def yaw_de_cuaternion(x, y, z, w):
    """El rumbo (giro alrededor de Z) de un cuaternión de ROS, en radianes."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def alcanzado(acumulado, objetivo_rad):
    """¿Se llegó al objetivo? Con signo: girar(−90) termina en −π/2.

    Comparar valores absolutos daría por buena una vuelta en el sentido
    contrario, que es exactamente el fallo que no se vería en un pasillo.
    """
    if objetivo_rad >= 0.0:
        return acumulado >= objetivo_rad
    return acumulado <= objetivo_rad


def velocidad_giro(restante_rad):
    """Rad/s para lo que queda de giro: rápido lejos, lento cerca.

    Es la rampa que hace que el lazo cerrado no se pase de largo. El signo lo
    pone quien llama, no esta función.
    """
    restante = abs(restante_rad)
    if restante > math.radians(30.0):
        return 0.80
    if restante > math.radians(8.0):
        return 0.40
    return 0.20
