#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Nodo de evitación de obstáculos autónoma para Sphero RVR con LiDAR.
Usa /scan para elegir la dirección más libre y publica /cmd_vel.
Respeta la parada de emergencia (no publica si está activa).

Uso:
  rosrun atriz_rvr_driver obstacle_avoidance.py
  roslaunch atriz_rvr_driver rvr_with_lidar_autonomous.launch
"""

import rospy
import math
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

# Parámetros por defecto (se pueden sobreescribir con ~param en el launch)
DEFAULT_SAFE_DISTANCE = 0.20      # m - por debajo: parar o girar (más bajo = se acerca más)
DEFAULT_WARNING_DISTANCE = 0.45  # m - por debajo: reducir velocidad y girar (menos oscilación)
DEFAULT_MAX_LINEAR = 0.25        # m/s
DEFAULT_MAX_ANGULAR = 0.6        # rad/s (algo menor para movimientos más suaves)
DEFAULT_FRONT_HALF_ANGLE_DEG = 30   # grados a cada lado para "frente"
DEFAULT_NUM_SECTORS = 16            # sectores en 360° para usar todo el mapa del LiDAR
DEFAULT_HYSTERESIS_MARGIN = 0.12    # m - solo cambiar lado de giro si el otro está esta cantidad más libre
DEFAULT_RECOVERY_BLOCKED_TIME = 2.0   # s - si el frente lleva bloqueado más tiempo, recuperación (marcha atrás)
DEFAULT_RECOVERY_REVERSE_TIME = 0.7   # s - duración de la marcha atrás en recuperación
DEFAULT_RECOVERY_REVERSE_SPEED = -0.12  # m/s - velocidad al retroceder
# 15 Hz > frecuencia típica de /scan (YDLidar X2 ~8–12 Hz) y > 1/cmd_vel_timeout del driver (~3.3 Hz)
DEFAULT_CONTROL_RATE = 15
# 180: el 0° del LiDAR apunta atrás del RVR; así "frente" en el código = delante real del robot
DEFAULT_ANGLE_OFFSET_DEG = 180.0
# range_min y range_max se leen del mensaje /scan (driver X2: 0.1 m, 8.0 m); no se hardcodean aquí


def _normalize_angle(angle_rad):
    """Devuelve el ángulo en [-pi, pi] (0 = delante, positivo = izq, convención ROS)."""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def _valid_range(r, range_min, range_max, use_range_min=True):
    """True si r es una lectura válida. Para evitación no usamos range_min como cota inferior:
    un obstáculo muy cerca (p. ej. 0.05 m) debe contar, no ignorarse y devolver 10 m."""
    if r != r or r <= 0:  # nan o no positivo
        return False
    if r == float('inf'):
        return False
    if use_range_min and range_min is not None and r < range_min:
        return False
    if range_max is not None and r > range_max:
        return False
    return True


def get_sector_min(ranges, angle_min_rad, angle_increment, start_deg, end_deg,
                   range_min=None, range_max=None, angle_offset_rad=0.0):
    """Distancia mínima en un sector [start_deg, end_deg] (0 = delante, + = izq).
    No filtramos por range_min para que obstáculos muy cercanos (< range_min) sí cuenten.
    """
    start_rad = math.radians(start_deg)
    end_rad = math.radians(end_deg)
    n = len(ranges)
    if n == 0:
        return float('inf')
    min_val = float('inf')
    for i in range(n):
        angle_raw = angle_min_rad + i * angle_increment
        angle = _normalize_angle(angle_raw + angle_offset_rad)
        in_sector = (start_rad <= angle <= end_rad if start_rad <= end_rad else
                     angle >= start_rad or angle <= end_rad)
        if not in_sector:
            continue
        r = ranges[i]
        if not _valid_range(r, range_min, range_max, use_range_min=False):
            continue
        if r < min_val:
            min_val = r
    # 10.0 m = "sin obstáculo" (por encima de range_max X2 = 8.0)
    return min_val if min_val != float('inf') else 10.0


def get_front_sector_min(msg, angle_offset_rad=0.0):
    """Mínima distancia en el sector frontal (±FRONT_HALF_ANGLE_DEG).
    Usa range_min/range_max del mensaje y normaliza ángulos a [-pi, pi].
    """
    a_min = msg.angle_min
    inc = msg.angle_increment
    half = math.radians(DEFAULT_FRONT_HALF_ANGLE_DEG)
    range_min = getattr(msg, 'range_min', None)
    range_max = getattr(msg, 'range_max', None)
    n = len(msg.ranges)
    min_val = float('inf')
    for i in range(n):
        angle_raw = a_min + i * inc
        angle = _normalize_angle(angle_raw + angle_offset_rad)
        if -half <= angle <= half:
            r = msg.ranges[i]
            if not _valid_range(r, range_min, range_max, use_range_min=False):
                continue
            if r < min_val:
                min_val = r
    return min_val if min_val != float('inf') else 10.0


def get_left_right_mins(msg, angle_offset_rad=0.0):
    """(min_left, min_right) en marco robot. LiDAR: 40–90° = izq real, -90–-40° = der real."""
    range_min = getattr(msg, 'range_min', None)
    range_max = getattr(msg, 'range_max', None)
    lidar_sector_40_90 = get_sector_min(
        msg.ranges, msg.angle_min, msg.angle_increment,
        40, 90, range_min, range_max, angle_offset_rad
    )
    lidar_sector_neg90_neg40 = get_sector_min(
        msg.ranges, msg.angle_min, msg.angle_increment,
        -90, -40, range_min, range_max, angle_offset_rad
    )
    robot_left_min = lidar_sector_40_90
    robot_right_min = lidar_sector_neg90_neg40
    return robot_left_min, robot_right_min


def get_full_sector_mins(msg, num_sectors, angle_offset_rad=0.0):
    """Usa todo el mapa del LiDAR: divide 360° en num_sectors y devuelve (angle_center_rad, min_dist) por sector.
    Ángulos en marco robot: 0 = delante, + = izq. Sectores centrados en -180+step/2, -180+3*step/2, ...
    """
    range_min = getattr(msg, 'range_min', None)
    range_max = getattr(msg, 'range_max', None)
    step_deg = 360.0 / num_sectors
    result = []
    for k in range(num_sectors):
        # Centro del sector k: -180 + (k+0.5)*step_deg, luego en [-180, 180)
        center_deg = -180.0 + (k + 0.5) * step_deg
        start_deg = center_deg - step_deg / 2
        end_deg = center_deg + step_deg / 2
        min_d = get_sector_min(
            msg.ranges, msg.angle_min, msg.angle_increment,
            start_deg, end_deg, range_min, range_max, angle_offset_rad
        )
        result.append((math.radians(center_deg), min_d))
    return result


def best_direction_rad(sector_mins, front_half_deg, search_half_deg, prefer_forward_above):
    """Usa el mapa completo: front_dist = mínimo en ±front_half_deg. Si front_dist >= prefer_forward_above
    sigue recto. Si no, elige el sector con mayor min_dist en ±search_half_deg (p. ej. ±90°)."""
    front_half_rad = math.radians(front_half_deg)
    search_half_rad = math.radians(search_half_deg)
    best_angle = 0.0
    best_dist = 0.0
    front_dist = 10.0
    for angle_rad, d in sector_mins:
        if abs(angle_rad) <= front_half_rad and d < front_dist:
            front_dist = d
        if -search_half_rad <= angle_rad <= search_half_rad and d > best_dist:
            best_dist = d
            best_angle = angle_rad
    if front_dist >= prefer_forward_above:
        return (0.0, front_dist)
    return (best_angle, best_dist)


def main():
    rospy.init_node('obstacle_avoidance', anonymous=False)

    safe_distance = rospy.get_param('~safe_distance', DEFAULT_SAFE_DISTANCE)
    warning_distance = rospy.get_param('~warning_distance', DEFAULT_WARNING_DISTANCE)
    max_linear = rospy.get_param('~max_linear_speed', DEFAULT_MAX_LINEAR)
    max_angular = rospy.get_param('~max_angular_speed', DEFAULT_MAX_ANGULAR)
    control_rate = rospy.get_param('~control_rate', DEFAULT_CONTROL_RATE)
    angle_offset_deg = rospy.get_param('~angle_offset_deg', DEFAULT_ANGLE_OFFSET_DEG)
    num_sectors = rospy.get_param('~num_sectors', DEFAULT_NUM_SECTORS)
    hysteresis_margin = rospy.get_param('~hysteresis_margin', DEFAULT_HYSTERESIS_MARGIN)
    recovery_blocked_time = rospy.get_param('~recovery_blocked_time', DEFAULT_RECOVERY_BLOCKED_TIME)
    recovery_reverse_time = rospy.get_param('~recovery_reverse_time', DEFAULT_RECOVERY_REVERSE_TIME)
    recovery_reverse_speed = rospy.get_param('~recovery_reverse_speed', DEFAULT_RECOVERY_REVERSE_SPEED)
    angle_offset_rad = math.radians(angle_offset_deg)

    latest_scan = [None]
    emergency_stop_active = [False]
    last_best_angle_rad = [0.0]
    last_best_dist = [10.0]
    blocked_since = [None]
    recovery_until = [0.0]

    def scan_cb(msg):
        latest_scan[0] = msg

    def emergency_cb(_):
        emergency_stop_active[0] = True

    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/scan', LaserScan, scan_cb, queue_size=1)
    rospy.Subscriber('/is_emergency_stop', Empty, emergency_cb, queue_size=1)

    # Comprobar también el parámetro que usa el driver
    def is_emergency():
        try:
            return emergency_stop_active[0] or rospy.get_param('/emergency_stop', False)
        except Exception:
            return emergency_stop_active[0]

    rate = rospy.Rate(control_rate)
    front_half_deg = DEFAULT_FRONT_HALF_ANGLE_DEG
    # Giro proporcional: angular = K * best_angle_rad, limitado a max_angular
    k_steer = max_angular / (math.pi / 2.0) if max_angular > 0 else 0.0
    rospy.loginfo('obstacle_avoidance: activo (safe=%.2f m, warning=%.2f m, sectores=%d, hist=%.2f m)',
                  safe_distance, warning_distance, num_sectors, hysteresis_margin)

    while not rospy.is_shutdown():
        if is_emergency():
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            cmd_vel_pub.publish(cmd)
            rate.sleep()
            continue

        msg = latest_scan[0]
        if msg is None:
            rate.sleep()
            continue

        sector_mins = get_full_sector_mins(msg, num_sectors, angle_offset_rad)
        front = get_front_sector_min(msg, angle_offset_rad)
        left_min, right_min = get_left_right_mins(msg, angle_offset_rad)
        search_half_deg = 90.0
        best_angle_rad, best_dist = best_direction_rad(
            sector_mins, front_half_deg, search_half_deg, prefer_forward_above=warning_distance
        )

        # Histéresis: solo cambiar de lado si el nuevo lado está margin más libre (evita oscilación)
        if (best_angle_rad * last_best_angle_rad[0]) < 0:
            if best_angle_rad > 0:
                if right_min >= left_min + hysteresis_margin:
                    pass
                else:
                    best_angle_rad = last_best_angle_rad[0]
                    best_dist = last_best_dist[0]
            else:
                if left_min >= right_min + hysteresis_margin:
                    pass
                else:
                    best_angle_rad = last_best_angle_rad[0]
                    best_dist = last_best_dist[0]
        last_best_angle_rad[0] = best_angle_rad
        last_best_dist[0] = best_dist
        if front >= warning_distance:
            last_best_angle_rad[0] = 0.0
            last_best_dist[0] = front
            blocked_since[0] = None
        elif front < safe_distance and blocked_since[0] is None:
            blocked_since[0] = time.time()

        now = time.time()
        in_recovery = recovery_until[0] > 0 and now < recovery_until[0]
        if not in_recovery and blocked_since[0] is not None and (now - blocked_since[0]) >= recovery_blocked_time:
            recovery_until[0] = now + recovery_reverse_time
            blocked_since[0] = None
            rospy.loginfo_throttle(1.0, 'obstacle_avoidance: recuperación (marcha atrás %.1f s)', recovery_reverse_time)
        if now >= recovery_until[0]:
            recovery_until[0] = 0.0

        cmd = Twist()
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0

        if in_recovery:
            cmd.linear.x = recovery_reverse_speed
            cmd.angular.z = 0.0
        elif front < safe_distance:
            cmd.linear.x = 0.0
            angular = k_steer * best_angle_rad
            cmd.angular.z = max(-max_angular, min(max_angular, angular))
            rospy.logdebug_throttle(0.5, 'obstacle_avoidance: obstáculo frontal %.2f m, girando', front)
        elif front < warning_distance:
            speed = max_linear * 0.35 * (front - safe_distance) / (warning_distance - safe_distance)
            speed = max(0.04, min(speed, max_linear * 0.35))
            cmd.linear.x = speed
            angular = k_steer * best_angle_rad * 0.7
            cmd.angular.z = max(-max_angular * 0.7, min(max_angular * 0.7, angular))
            rospy.logdebug_throttle(0.5, 'obstacle_avoidance: precaución frontal %.2f m', front)
        else:
            t = (front - warning_distance) / (1.5 - warning_distance)
            t = max(0.0, min(1.0, t))
            cmd.linear.x = 0.12 + (max_linear - 0.12) * t
            cmd.angular.z = 0.0

        cmd_vel_pub.publish(cmd)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
