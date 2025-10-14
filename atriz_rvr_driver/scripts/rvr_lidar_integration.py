#!/usr/bin/env python3
"""
Integración LIDAR X2 con Sphero RVR
====================================

Este script demuestra cómo usar datos del LIDAR para control del Sphero RVR.
Incluye ejemplos de:
- Detección de obstáculos
- Navegación reactiva
- Parada de emergencia por proximidad
- Visualización de zonas de seguridad

Autor: AI Assistant
Fecha: 2025
"""

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import sys

class RVRLidarIntegration:
    """Integración del LIDAR con el Sphero RVR."""
    
    def __init__(self):
        rospy.init_node('rvr_lidar_integration', anonymous=True)
        
        # Parámetros configurables
        self.safe_distance = rospy.get_param('~safe_distance', 0.5)  # metros
        self.warning_distance = rospy.get_param('~warning_distance', 1.0)  # metros
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 0.5)  # m/s
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 1.0)  # rad/s
        
        # Variables de estado
        self.latest_scan = None
        self.obstacle_detected = False
        self.min_distance = float('inf')
        self.min_angle = 0.0
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/sphero_rvr/cmd_vel', Twist, queue_size=1)
        
        # Subscribers
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        # Timers
        self.control_timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)
        
        rospy.loginfo("🚀 RVR-LIDAR Integration initialized")
        rospy.loginfo(f"   Safe distance: {self.safe_distance}m")
        rospy.loginfo(f"   Warning distance: {self.warning_distance}m")
        
    def scan_callback(self, msg):
        """Procesa datos del LIDAR."""
        self.latest_scan = msg
        
        # Encontrar distancia mínima y su ángulo
        ranges = np.array(msg.ranges)
        
        # Filtrar valores inválidos (inf, nan)
        valid_ranges = ranges[np.isfinite(ranges)]
        
        if len(valid_ranges) > 0:
            self.min_distance = np.min(valid_ranges)
            min_index = np.argmin(ranges)
            self.min_angle = msg.angle_min + min_index * msg.angle_increment
            
            # Detectar obstáculos
            if self.min_distance < self.safe_distance:
                if not self.obstacle_detected:
                    rospy.logwarn(f"⚠️  Obstáculo detectado a {self.min_distance:.2f}m")
                self.obstacle_detected = True
            else:
                if self.obstacle_detected:
                    rospy.loginfo("✅ Vía libre")
                self.obstacle_detected = False
    
    def get_frontal_distance(self):
        """Obtiene la distancia mínima en el sector frontal (±30°)."""
        if self.latest_scan is None:
            return float('inf')
        
        ranges = np.array(self.latest_scan.ranges)
        angles = np.linspace(
            self.latest_scan.angle_min, 
            self.latest_scan.angle_max, 
            len(ranges)
        )
        
        # Sector frontal: -30° a +30° (en radianes)
        front_mask = np.abs(angles) < np.radians(30)
        front_ranges = ranges[front_mask]
        
        # Filtrar valores válidos
        valid_front = front_ranges[np.isfinite(front_ranges)]
        
        if len(valid_front) > 0:
            return np.min(valid_front)
        return float('inf')
    
    def get_sector_distance(self, angle_min_deg, angle_max_deg):
        """Obtiene la distancia mínima en un sector angular específico."""
        if self.latest_scan is None:
            return float('inf')
        
        ranges = np.array(self.latest_scan.ranges)
        angles = np.linspace(
            self.latest_scan.angle_min, 
            self.latest_scan.angle_max, 
            len(ranges)
        )
        
        # Convertir a radianes
        angle_min_rad = np.radians(angle_min_deg)
        angle_max_rad = np.radians(angle_max_deg)
        
        # Sector específico
        sector_mask = (angles >= angle_min_rad) & (angles <= angle_max_rad)
        sector_ranges = ranges[sector_mask]
        
        # Filtrar valores válidos
        valid_sector = sector_ranges[np.isfinite(sector_ranges)]
        
        if len(valid_sector) > 0:
            return np.min(valid_sector)
        return float('inf')
    
    def control_loop(self, event):
        """Loop de control reactivo basado en LIDAR."""
        if self.latest_scan is None:
            return
        
        # Obtener distancias en diferentes sectores
        front_dist = self.get_frontal_distance()
        left_dist = self.get_sector_distance(30, 90)
        right_dist = self.get_sector_distance(-90, -30)
        
        # Crear comando de velocidad
        cmd = Twist()
        
        # Lógica de control reactivo simple
        if front_dist < self.safe_distance:
            # PARADA DE EMERGENCIA
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            rospy.logwarn_throttle(1.0, f"🛑 STOP! Obstáculo frontal a {front_dist:.2f}m")
            
        elif front_dist < self.warning_distance:
            # Reducir velocidad y girar hacia el lado más libre
            if left_dist > right_dist:
                cmd.linear.x = 0.1
                cmd.angular.z = 0.5  # Girar a la izquierda
                rospy.loginfo_throttle(1.0, f"⬅️  Girando izquierda (frontal: {front_dist:.2f}m)")
            else:
                cmd.linear.x = 0.1
                cmd.angular.z = -0.5  # Girar a la derecha
                rospy.loginfo_throttle(1.0, f"➡️  Girando derecha (frontal: {front_dist:.2f}m)")
        else:
            # Vía libre - movimiento normal
            cmd.linear.x = 0.3
            cmd.angular.z = 0.0
            rospy.loginfo_throttle(5.0, f"✅ Vía libre (frontal: {front_dist:.2f}m)")
        
        # Publicar comando
        # NOTA: Comentado por seguridad - descomentar cuando esté listo para pruebas
        # self.cmd_vel_pub.publish(cmd)
    
    def emergency_stop(self):
        """Detiene el robot inmediatamente."""
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        rospy.loginfo("🛑 Emergency stop executed")
    
    def print_stats(self):
        """Imprime estadísticas del LIDAR."""
        if self.latest_scan is None:
            return
        
        front = self.get_frontal_distance()
        left = self.get_sector_distance(30, 90)
        right = self.get_sector_distance(-90, -30)
        back = self.get_sector_distance(150, 210)
        
        print("\n" + "="*50)
        print("📊 ESTADÍSTICAS DEL LIDAR")
        print("="*50)
        print(f"   Frontal:  {front:.2f}m")
        print(f"   Izquierda: {left:.2f}m")
        print(f"   Derecha:   {right:.2f}m")
        print(f"   Trasera:   {back:.2f}m")
        print(f"   Mín dist:  {self.min_distance:.2f}m @ {np.degrees(self.min_angle):.1f}°")
        print("="*50 + "\n")
    
    def run(self):
        """Ejecuta el nodo."""
        rate = rospy.Rate(1)  # 1 Hz para stats
        
        try:
            while not rospy.is_shutdown():
                self.print_stats()
                rate.sleep()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down...")
            self.emergency_stop()

def main():
    try:
        node = RVRLidarIntegration()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Error: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()

