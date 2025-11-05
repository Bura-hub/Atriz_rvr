#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Seguidor de línea (desde cero) con PID discreto

Incluye:
- Cálculo de error respecto a la línea (negro vs blanco)
- Integral discreta (método del trapecio) con anti-windup
- Derivada discreta (diferencia finita) con límites de dt

Tópicos usados:
- Sub:  /color  (atriz_rvr_msgs/Color, campo rgb_color[3] entero 0-255)
- Pub:  /cmd_vel (geometry_msgs/Twist)
"""

import os
import time
import math
import signal
from collections import deque

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, TriggerResponse
from atriz_rvr_msgs.msg import Color
import json
import os


class DiscretePID:
    """PID discreto con trapezoidal integral, diferencia finita y anti-windup."""

    def __init__(self, kp=0.5, ki=0.0, kd=0.3,
                 integral_min=-1.0, integral_max=1.0,
                 dt_min=0.001, dt_max=0.05):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.integral = 0.0
        self.integral_min = integral_min
        self.integral_max = integral_max

        self.prev_error = 0.0
        self.prev_time = time.time()

        self.dt_min = dt_min
        self.dt_max = dt_max

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = time.time()

    def compute(self, error: float) -> float:
        now = time.time()
        dt = now - self.prev_time

        # Normalizar dt
        if dt <= 0.0 or dt < self.dt_min:
            dt = self.dt_min
        elif dt > self.dt_max:
            # Si el bucle estuvo detenido, no acumular integral/derivada excesivas
            self.integral = 0.0
            self.prev_error = error
            dt = self.dt_max

        # Proporcional
        P = self.kp * error

        # Integral (trapecio)
        if self.ki != 0.0:
            error_avg = 0.5 * (error + self.prev_error)
            self.integral += error_avg * dt
            self.integral = max(self.integral_min, min(self.integral, self.integral_max))
            I = self.ki * self.integral
        else:
            I = 0.0

        # Derivada (diferencia finita)
        D = 0.0
        if self.kd != 0.0:
            derivative = (error - self.prev_error) / dt
            D = self.kd * derivative

        # Actualizar estados
        self.prev_error = error
        self.prev_time = now

        return P + I + D


class LineFollowerSimple:
    """Seguidor de línea con un solo sensor de color.

    Estrategia robusta con un solo sensor (sin matriz):
    - Usa histéresis (dos umbrales) para clasificar BLACK/WHITE con menos ruido.
    - Sigue el BORDE (edge following) alternando pequeños giros: si ve negro, gira hacia blanco;
      si ve blanco, gira hacia negro. Esto mantiene al robot "pegado" al borde.
    - Ajusta velocidad y giro en pasos cortos y a alta frecuencia para reacciones rápidas.
    - Si se pierde (blanco sostenido), hace búsqueda en sitio hacia el último borde y revierte si no lo encuentra.
    """

    def __init__(self):
        rospy.init_node('seguidor_linea_pid_demo', anonymous=False)

        # Parámetros configurables (rosparam o defaults)
        self.vel_max = float(rospy.get_param('~velocidad_maxima', 0.08))  # m/s
        self.vel_min = float(rospy.get_param('~velocidad_minima', 0.02))  # m/s
        self.vel_ang_max = float(rospy.get_param('~velocidad_angular_max', 1.0))  # rad/s

        self.kp = float(rospy.get_param('~kp', 0.5))
        self.ki = float(rospy.get_param('~ki', 0.0))
        self.kd = float(rospy.get_param('~kd', 0.3))

        # Umbrales: se pueden ajustar vía rosparam
        # Histéresis: negro < t_negro; blanco > t_blanco; entre ambos = transición
        self.umbral_negro = int(rospy.get_param('~umbral_negro', 60))
        self.umbral_blanco = int(rospy.get_param('~umbral_blanco', 200))
        # Margen adicional para histéresis dinámica
        self.hysteresis_margin = float(rospy.get_param('~hysteresis_margin', 5.0))

        # Archivo de calibración (persistencia)
        self.script_dir = os.path.dirname(os.path.abspath(__file__))
        self.calib_path = os.path.join(self.script_dir, 'calibracion_colores.json')
        self._cargar_calibracion_desde_json()

        # Buffer simple para filtrar intensidad
        self.buffer_int = deque(maxlen=int(rospy.get_param('~tamano_buffer', 3)))

        # PID
        self.pid = DiscretePID(kp=self.kp, ki=self.ki, kd=self.kd,
                               integral_min=-1.0, integral_max=1.0,
                               dt_min=0.001, dt_max=0.1)

        # IO ROS
        self.pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/color', Color, self.cb_color)
        rospy.Subscriber('/emergency_stop', Bool, self.cb_emergency)

        # Servicios de calibración
        self.srv_cal_negro = rospy.Service('~calibrar_negro', Trigger, self._srv_calibrar_negro)
        self.srv_cal_blanco = rospy.Service('~calibrar_blanco', Trigger, self._srv_calibrar_blanco)

        self.emergency = False
        self.last_on_line_time = time.time()
        self.last_edge_dir = 1  # +1 significa "borde a la derecha", -1 "borde a la izquierda"
        self.search_started_time = None
        self.search_inverted = False
        self.rate = rospy.Rate(80)

        # Señales
        signal.signal(signal.SIGINT, self._signal)
        signal.signal(signal.SIGTERM, self._signal)

        self.last_error = 0.0

        # Menú interactivo (opcional)
        self.use_menu = bool(rospy.get_param('~menu', True))
        if self.use_menu:
            self._menu_interactivo()

        # Compensación por dos orugas y sensor adelantado respecto al centro:
        # En pérdida (WHITE) retroceder ligeramente y girar contrario, luego escanear al lado opuesto.
        self.sensor_offset_m = float(rospy.get_param('~sensor_offset_m', 0.05))
        self.track_width_m = float(rospy.get_param('~track_width_m', 0.16))
        # Duraciones más largas para alternado más amplio entre puntos
        self.recovery_reverse_time = float(rospy.get_param('~recovery_reverse_time', 0.25))
        self.recovery_scan_time = float(rospy.get_param('~recovery_scan_time', 0.70))
        self.recovery_linear = float(rospy.get_param('~recovery_linear', -0.03))
        self.recovery_angular = float(rospy.get_param('~recovery_angular', 0.6))
        self.recovery_started = None
        self.recovery_phase = 0  # 0 idle, 1 reverse+counter, 2 scan opposite

    def _signal(self, *_):
        self.stop_robot()
        rospy.signal_shutdown('Signal received')

    def cb_emergency(self, msg: Bool):
        self.emergency = bool(msg.data)
        if self.emergency:
            self.stop_robot()

    def cb_color(self, msg: Color):
        try:
            r = int(msg.rgb_color[0])
            g = int(msg.rgb_color[1])
            b = int(msg.rgb_color[2])
            intensidad = (r + g + b) / 3.0
            self.buffer_int.append(intensidad)
        except Exception:
            pass

    def classify_color(self):
        """Clasifica color actual como 'BLACK', 'WHITE' o 'MID'."""
        if len(self.buffer_int) == 0:
            return 'MID', None
        intensidad = sum(self.buffer_int) / len(self.buffer_int)
        t_black = self.umbral_negro + self.hysteresis_margin
        t_white = self.umbral_blanco - self.hysteresis_margin
        if intensidad <= t_black:
            return 'BLACK', intensidad
        if intensidad >= t_white:
            return 'WHITE', intensidad
        return 'MID', intensidad

    # ---------------------- Calibración ----------------------
    def _tomar_mediciones(self, duracion_s=2.0):
        """Promedia intensidad durante 'duracion_s'."""
        t0 = rospy.Time.now().to_sec()
        vals = []
        while (rospy.Time.now().to_sec() - t0) < duracion_s and not rospy.is_shutdown():
            if len(self.buffer_int) > 0:
                vals.append(sum(self.buffer_int) / len(self.buffer_int))
            rospy.sleep(0.05)
        if len(vals) == 0:
            return None
        return sum(vals) / len(vals)

    def _srv_calibrar_negro(self, _req):
        prom = self._tomar_mediciones(rospy.get_param('~cal_duracion', 2.0))
        if prom is None:
            return TriggerResponse(success=False, message='Sin lecturas para calibrar NEGRO')
        # Establecer umbral de negro con margen
        self.umbral_negro = int(prom)
        self._guardar_calibracion_a_json()
        msg = f'NEGRO calibrado: {prom:.1f} → umbral_negro={self.umbral_negro}'
        rospy.loginfo(msg)
        return TriggerResponse(success=True, message=msg)

    def _srv_calibrar_blanco(self, _req):
        prom = self._tomar_mediciones(rospy.get_param('~cal_duracion', 2.0))
        if prom is None:
            return TriggerResponse(success=False, message='Sin lecturas para calibrar BLANCO')
        # Establecer umbral de blanco con margen
        self.umbral_blanco = int(prom)
        self._guardar_calibracion_a_json()
        msg = f'BLANCO calibrado: {prom:.1f} → umbral_blanco={self.umbral_blanco}'
        rospy.loginfo(msg)
        return TriggerResponse(success=True, message=msg)

    def _guardar_calibracion_a_json(self):
        try:
            data = {
                'umbral_negro': int(self.umbral_negro),
                'umbral_blanco': int(self.umbral_blanco),
                'hysteresis_margin': float(self.hysteresis_margin)
            }
            with open(self.calib_path, 'w') as f:
                json.dump(data, f, indent=2)
            rospy.loginfo(f'Calibración guardada en {self.calib_path}')
        except Exception as e:
            rospy.logwarn(f'No se pudo guardar calibración: {e}')

    def _cargar_calibracion_desde_json(self):
        try:
            if os.path.exists(self.calib_path):
                with open(self.calib_path, 'r') as f:
                    data = json.load(f)
                # Solo aplicar si existen
                if 'umbral_negro' in data:
                    self.umbral_negro = int(data['umbral_negro'])
                if 'umbral_blanco' in data:
                    self.umbral_blanco = int(data['umbral_blanco'])
                if 'hysteresis_margin' in data:
                    self.hysteresis_margin = float(data['hysteresis_margin'])
                rospy.loginfo(f'Calibración cargada de {self.calib_path}: negro={self.umbral_negro}, blanco={self.umbral_blanco}, h={self.hysteresis_margin}')
        except Exception as e:
            rospy.logwarn(f'No se pudo cargar calibración: {e}')

    def stop_robot(self):
        cmd = Twist()
        self.pub_cmd.publish(cmd)

    def _menu_interactivo(self):
        """Menú de inicio: calibración de colores y ejecución."""
        rospy.loginfo("\n================ MENÚ SEGUIDOR DE LÍNEA ================")
        rospy.loginfo("1) Calibrar NEGRO")
        rospy.loginfo("2) Calibrar BLANCO")
        rospy.loginfo("3) Ejecutar seguidor")
        rospy.loginfo("4) Salir")
        rospy.loginfo("========================================================\n")

        while not rospy.is_shutdown():
            try:
                opcion = input("Seleccione una opción [1-4]: ").strip()
            except Exception:
                opcion = '3'  # fallback a ejecutar si no hay stdin

            if opcion == '1':
                rospy.loginfo("Coloque el robot sobre la LÍNEA NEGRA. Midiendo...")
                resp = self._srv_calibrar_negro(None)
                rospy.loginfo(resp.message)
            elif opcion == '2':
                rospy.loginfo("Coloque el robot sobre el FONDO BLANCO. Midiendo...")
                resp = self._srv_calibrar_blanco(None)
                rospy.loginfo(resp.message)
            elif opcion == '3':
                rospy.loginfo("Iniciando ejecución del seguidor...")
                break
            elif opcion == '4':
                rospy.loginfo("Saliendo por solicitud del usuario.")
                rospy.signal_shutdown('Salida desde menú')
                break
            else:
                rospy.logwarn("Opción inválida. Intente de nuevo.")

    def run(self):
        rospy.loginfo('Seguidor PID discreto iniciado')
        while not rospy.is_shutdown():
            if self.emergency:
                self.stop_robot()
                self.rate.sleep()
                continue

            state, intensidad = self.classify_color()

            cmd = Twist()

            if state == 'BLACK' or state == 'MID':
                # Estamos sobre línea o cerca del borde: avanzar y girar suavemente hacia el blanco
                self.last_on_line_time = time.time()
                self.search_started_time = None
                self.search_inverted = False

                # Giro corto hacia el blanco: signo opuesto al borde que seguimos
                # Si el borde está a la derecha (+1), giramos +omega (derecha) hasta ver blanco y viceversa
                omega = 0.4  # giro corto y rápido
                cmd.angular.z = omega * self.last_edge_dir

                # Velocidad lineal corta (ráfagas) para no salirnos
                cmd.linear.x = self.vel_max

                rospy.loginfo_throttle(1.0, f"FOLLOW-EDGE | state={state} | v={cmd.linear.x:.2f} | w={cmd.angular.z:.2f}")

                # Si cruzamos a blanco pronto, el siguiente ciclo invertirá el giro
            else:  # 'WHITE' → fuera: recuperación en dos fases por sensor adelantado
                now = time.time()
                if self.recovery_started is None:
                    self.recovery_started = now
                    self.recovery_phase = 1
                    rospy.logwarn("WHITE | RECOVERY F1: retroceso + giro contrario")

                elapsed = now - self.recovery_started

                if self.recovery_phase == 1:
                    # Retroceder y girar contrario al lado del borde seguido
                    cmd.linear.x = self.recovery_linear
                    cmd.angular.z = -self.recovery_angular * self.last_edge_dir
                    if elapsed >= self.recovery_reverse_time:
                        # Pasar a fase de escaneo en sitio hacia el lado opuesto
                        self.recovery_phase = 2
                        self.recovery_started = now
                        self.last_edge_dir *= -1  # invertir hipótesis de borde
                        rospy.logwarn("WHITE | RECOVERY F2: escaneo en sitio al lado opuesto")

                elif self.recovery_phase == 2:
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.5 * self.last_edge_dir
                    if elapsed >= self.recovery_scan_time:
                        # Alternar de nuevo si no se encontró
                        self.recovery_phase = 1
                        self.recovery_started = now
                        rospy.logwarn("WHITE | RECOVERY loop: alternando F1↔F2")

            self.pub_cmd.publish(cmd)
            self.rate.sleep()


def main():
    try:
        node = LineFollowerSimple()
        node.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()