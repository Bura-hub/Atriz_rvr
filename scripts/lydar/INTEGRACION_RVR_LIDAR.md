# 🤖 Integración LIDAR X2 con Sphero RVR

## 📋 Contenido

1. [Arquitectura de Integración](#arquitectura-de-integración)
2. [Launch Files Disponibles](#launch-files-disponibles)
3. [Scripts de Integración](#scripts-de-integración)
4. [Uso Básico](#uso-básico)
5. [Ejemplos de Integración](#ejemplos-de-integración)
6. [Desarrollo Avanzado](#desarrollo-avanzado)

---

## 🏗️ Arquitectura de Integración

### Estructura del Workspace

```
~/atriz_git/
├── src/
│   ├── ydlidar_ros_driver/          # Driver del LIDAR (paquete ROS)
│   ├── YDLidar-SDK/                 # SDK del LIDAR
│   └── Atriz_rvr/              # Proyecto Sphero RVR
│       ├── launch/                  # ← Launch files de integración
│       │   ├── lidar_only.launch
│       │   └── rvr_with_lidar.launch
│       ├── scripts/                 # ← Scripts de integración
│       │   └── rvr_lidar_integration.py
│       └── ...
├── devel/
└── build/
```

### ¿Por qué esta estructura?

1. **Paquetes en `/src`**: Los paquetes ROS (`ydlidar_ros_driver`) se instalan en el nivel superior del workspace
2. **Integración en `Atriz_rvr/`**: Los launch files y scripts que **integran** LIDAR+RVR están dentro del proyecto
3. **Ventaja**: Puedes usar el LIDAR desde cualquier proyecto, y tener configuraciones específicas en `Atriz_rvr`

---

## 🚀 Launch Files Disponibles

### 1. LIDAR Solo (desde Atriz_rvr)

**Archivo**: `~/atriz_git/src/Atriz_rvr/launch/lidar_only.launch`

```bash
cd ~/atriz_git
source devel/setup.bash
roslaunch atriz_rvr_driver lidar_only.launch
```

**Características**:
- ✅ Lanza solo el LIDAR X2
- ✅ Publica en `/scan`
- ✅ Crea TF de `base_link` a `laser`
- ✅ Parámetro de puerto configurable

**Con parámetros**:
```bash
roslaunch atriz_rvr_driver lidar_only.launch port:=/dev/ttyUSB0
```

### 2. RVR + LIDAR Integrado

**Archivo**: `~/atriz_git/src/Atriz_rvr/launch/rvr_with_lidar.launch`

```bash
roslaunch atriz_rvr_driver rvr_with_lidar.launch
```

**Características**:
- ✅ Lanza LIDAR X2
- ✅ Lanza Sphero RVR (cuando esté configurado)
- ✅ Configura TF entre RVR y LIDAR
- ✅ Listo para navegación

**Configuración**:
- Edita el launch file para ajustar la posición del LIDAR en el RVR
- Descomenta la sección del RVR cuando esté listo

### 3. LIDAR Directo (desde ydlidar_ros_driver)

**Archivo**: `~/atriz_git/src/ydlidar_ros_driver/launch/x2_custom.launch`

```bash
roslaunch ydlidar_ros_driver x2_custom.launch
```

**Uso**: Para probar el LIDAR sin integración con RVR

---

## 🐍 Scripts de Integración

### Script Principal: `rvr_lidar_integration.py`

**Ubicación**: `~/atriz_git/src/Atriz_rvr/scripts/rvr_lidar_integration.py`

**Funcionalidades**:

1. **Detección de obstáculos**
   - Monitoreo de sectores: frontal, izquierda, derecha
   - Distancias de seguridad configurables

2. **Control reactivo**
   - Parada automática ante obstáculos cercanos
   - Navegación evasiva
   - Velocidad adaptativa

3. **Visualización**
   - Estadísticas en tiempo real
   - Indicadores de estado

**Ejecución**:
```bash
# 1. Lanzar LIDAR
roslaunch atriz_rvr_driver lidar_only.launch

# 2. En otra terminal - Ejecutar integración
cd ~/atriz_git
source devel/setup.bash
rosrun atriz_rvr_driver rvr_lidar_integration.py
```

**Parámetros configurables**:
```bash
rosrun atriz_rvr_driver rvr_lidar_integration.py \
  _safe_distance:=0.5 \
  _warning_distance:=1.0 \
  _max_linear_speed:=0.5
```

---

## 💡 Uso Básico

### Opción 1: Solo LIDAR (Visualización)

```bash
# Terminal 1: Lanzar LIDAR
cd ~/atriz_git && source devel/setup.bash
roslaunch atriz_rvr_driver lidar_only.launch

# Terminal 2: Ver datos
rostopic echo /scan

# Terminal 3: RViz
rviz
# Add → LaserScan → Topic: /scan → Fixed Frame: laser
```

### Opción 2: LIDAR + Monitoreo

```bash
# Terminal 1: LIDAR
roslaunch atriz_rvr_driver lidar_only.launch

# Terminal 2: Script de integración (modo monitor)
rosrun atriz_rvr_driver rvr_lidar_integration.py

# Verás estadísticas cada segundo:
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 📊 ESTADÍSTICAS DEL LIDAR
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#    Frontal:  2.45m
#    Izquierda: 3.12m
#    Derecha:   1.87m
#    Trasera:   4.56m
#    Mín dist:  1.23m @ 45.0°
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

### Opción 3: Sistema Completo (RVR + LIDAR)

```bash
# Terminal 1: Sistema completo
roslaunch atriz_rvr_driver rvr_with_lidar.launch

# Terminal 2: Control reactivo (cuando esté listo)
rosrun atriz_rvr_driver rvr_lidar_integration.py
```

---

## 📚 Ejemplos de Integración

### Ejemplo 1: Leer Distancia Frontal

```python
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np

def scan_callback(msg):
    # Convertir a numpy array
    ranges = np.array(msg.ranges)
    angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
    
    # Sector frontal (±30°)
    front_mask = np.abs(angles) < np.radians(30)
    front_ranges = ranges[front_mask]
    
    # Filtrar valores válidos
    valid = front_ranges[np.isfinite(front_ranges)]
    
    if len(valid) > 0:
        min_dist = np.min(valid)
        print(f"Distancia frontal: {min_dist:.2f}m")

rospy.init_node('lidar_reader')
rospy.Subscriber('/scan', LaserScan, scan_callback)
rospy.spin()
```

### Ejemplo 2: Control Básico con LIDAR

```python
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class SimpleAvoidance:
    def __init__(self):
        self.pub = rospy.Publisher('/sphero_rvr/cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.callback)
        
    def callback(self, msg):
        ranges = msg.ranges
        min_dist = min([r for r in ranges if r > 0.1])  # Filtrar ruido
        
        cmd = Twist()
        if min_dist < 0.5:
            # Parar si hay obstáculo cerca
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5  # Girar
        else:
            # Avanzar
            cmd.linear.x = 0.3
            cmd.angular.z = 0.0
        
        self.pub.publish(cmd)

if __name__ == '__main__':
    rospy.init_node('simple_avoidance')
    SimpleAvoidance()
    rospy.spin()
```

### Ejemplo 3: Seguir Pared

```python
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class WallFollower:
    def __init__(self):
        self.pub = rospy.Publisher('/sphero_rvr/cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.callback)
        self.target_distance = 0.5  # 50cm de la pared
        
    def get_right_distance(self, msg):
        """Obtiene distancia al lado derecho (90°)."""
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        
        # Sector derecho: -60° a -120°
        right_mask = (angles > np.radians(-120)) & (angles < np.radians(-60))
        right_ranges = ranges[right_mask]
        valid = right_ranges[np.isfinite(right_ranges)]
        
        return np.mean(valid) if len(valid) > 0 else float('inf')
    
    def callback(self, msg):
        right_dist = self.get_right_distance(msg)
        
        cmd = Twist()
        cmd.linear.x = 0.2
        
        # Control proporcional
        error = right_dist - self.target_distance
        cmd.angular.z = -0.5 * error  # Ajustar giro según error
        
        self.pub.publish(cmd)

if __name__ == '__main__':
    rospy.init_node('wall_follower')
    WallFollower()
    rospy.spin()
```

---

## 🛠️ Desarrollo Avanzado

### 1. Agregar Nuevos Comportamientos

Edita `rvr_lidar_integration.py` para agregar:
- Mapeo SLAM
- Navegación autónoma
- Detección de objetos
- Seguimiento de personas

### 2. Configurar Posición del LIDAR

En `rvr_with_lidar.launch`, ajusta el TF:

```xml
<!-- x, y, z = posición del LIDAR respecto a base_link -->
<node pkg="tf" type="static_transform_publisher" name="base_to_laser_tf"
  args="0.1 0.0 0.15 0.0 0.0 0.0 /base_link /laser 50" />
       ↑   ↑    ↑
     10cm  0   15cm
   adelante    altura
```

### 3. Integrar con ROS Navigation Stack

```bash
# Instalar navigation stack
sudo apt-get install ros-noetic-navigation

# Crear launch file con:
# - map_server
# - amcl
# - move_base
# - LIDAR
# - RVR
```

### 4. Crear Configuración RViz

Guarda configuración en: `~/atriz_git/src/Atriz_rvr/config/rvr_lidar.rviz`

---

## 🔗 Topics ROS Importantes

| Topic | Tipo | Descripción |
|-------|------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | Datos del LIDAR |
| `/sphero_rvr/cmd_vel` | `geometry_msgs/Twist` | Control de velocidad RVR |
| `/tf` | `tf2_msgs/TFMessage` | Transformaciones |

---

## ✅ Checklist de Integración

- [x] LIDAR instalado y funcionando
- [x] Launch files creados en `Atriz_rvr/launch/`
- [x] Script de integración básico creado
- [x] TF entre base_link y laser configurado
- [ ] RVR driver configurado
- [ ] Probar control reactivo
- [ ] Ajustar distancias de seguridad
- [ ] Configurar RViz
- [ ] Crear mapas con SLAM

---

## 📖 Referencias

- [GUIA_COMPLETA_LIDAR.md](../../GUIA_COMPLETA_LIDAR.md) - Guía del LIDAR (raíz del proyecto)
- [YDLIDAR_X2_RESUMEN.md](YDLIDAR_X2_RESUMEN.md) - Resumen del X2
- [ROS LaserScan](http://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html) - Documentación del mensaje
- [ROS Navigation](http://wiki.ros.org/navigation) - Stack de navegación

---

**¡Integración lista para usar!** 🎉

