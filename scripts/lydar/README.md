# 🔍 Configuración del LIDAR YDLidar

## 📖 Documentación Principal

### ⭐ [GUIA_COMPLETA_LIDAR.md](../../GUIA_COMPLETA_LIDAR.md) - **LEE ESTO PRIMERO** (en la raíz del proyecto)

**Guía unificada con TODO lo necesario:**
- ✅ Estado actual y especificaciones
- ✅ Instalación completa paso a paso  
- ✅ Uso del LIDAR y visualización
- ✅ Solución de problemas
- ✅ Configuración avanzada
- ✅ Integración con Sphero RVR

### 🛠️ Herramientas

**[test_lidar.py](test_lidar.py)** - Script de prueba
```bash
python3 test_lidar.py          # Prueba básica
python3 test_lidar.py --scan   # Escanear baudrates
```

**[install_lidar_driver.sh](install_lidar_driver.sh)** - Instalador automático
```bash
./install_lidar_driver.sh
# Instala drivers + SDK automáticamente
```

## ✅ Estado Actual - YDLIDAR X2 Funcionando

Tu LIDAR está **completamente configurado y operativo**:
- ✅ Modelo: **YDLIDAR X2**
- ✅ Puerto: `/dev/ydlidar` o `/dev/ttyUSB0`
- ✅ Baudrate: `115200`
- ✅ Sample Rate: `3K` (3000 muestras/seg)
- ✅ Frecuencia: `8 Hz` (X2 spec)
- ✅ Driver: ydlidar_ros_driver instalado
- ✅ SDK: YDLidar-SDK instalado
- ✅ Workspace: Compilado exitosamente
- ✅ Launch file: `x2_custom.launch`
- ✅ Estado: **✨ FUNCIONANDO PERFECTAMENTE**

## 🚀 Inicio Rápido

### Ejecutar el YDLIDAR X2:

```bash
# 1. Source del workspace
cd /home/sphero/atriz_git && source devel/setup.bash

# 2. Permisos del puerto (si es necesario)
sudo chmod 666 /dev/ttyUSB0

# 3. Ejecutar el LIDAR
roslaunch ydlidar_ros_driver x2_custom.launch

# 4. Visualizar en RViz (en otra terminal)
rviz
# Add → LaserScan → Topic: /scan → Fixed Frame: laser
```

### Verificar funcionamiento:
```bash
# Ver topics
rostopic list

# Ver frecuencia (debe ser ~10 Hz)
rostopic hz /scan

# Ver datos
rostopic echo /scan
```

### Para reinstalar o configurar otro LIDAR:
```bash
./install_lidar_driver.sh
```

---

## 🤖 Integración con Sphero RVR

### Launch Files de Integración

**1. Solo LIDAR**:
```bash
roslaunch atriz_rvr_driver lidar_only.launch
```

**2. RVR + LIDAR (sistema completo)**:
```bash
roslaunch atriz_rvr_driver rvr_with_lidar.launch
```

### Script de Integración

**Monitoreo y control reactivo**:
```bash
# Terminal 1: Lanzar LIDAR
roslaunch atriz_rvr_driver lidar_only.launch

# Terminal 2: Ejecutar integración
rosrun atriz_rvr_driver rvr_lidar_integration.py
```

**Ver guía completa de integración**:
```bash
cat INTEGRACION_RVR_LIDAR.md
```

---

## 📁 Archivos del Directorio

```
lydar/
├── INTEGRACION_RVR_LIDAR.md      🤖 Integración con Sphero RVR
├── README.md                      📖 Este archivo
├── install_lidar_driver.sh        🔧 Instalador automático
└── test_lidar.py                  🧪 Script de prueba

En la raíz del proyecto (Atriz_rvr/):
└── GUIA_COMPLETA_LIDAR.md        ⭐ Guía completa LIDAR (TODO-EN-UNO)

Integración en atriz_rvr_driver/:
├── launch/
│   ├── lidar_only.launch              ⚡ Solo LIDAR
│   ├── rvr_with_lidar.launch          🤖 RVR + LIDAR
│   └── rvr_with_lidar_autonomous.launch  🤖 RVR + LIDAR + evitación
└── scripts/
    ├── rvr_lidar_integration.py  🐍 Script de integración
    └── obstacle_avoidance.py     🤖 Evitación autónoma de obstáculos
```

**Documentación completa** - Todo funcionando correctamente.

---

## 🔗 Enlaces de Referencia

- **Guía Completa**: [GUIA_COMPLETA_LIDAR.md](../../GUIA_COMPLETA_LIDAR.md)
- **YDLidar GitHub**: https://github.com/YDLIDAR/ydlidar_ros_driver
- **ROS LaserScan**: http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html

---

**¿Problemas?** → Consulta la sección "Solución de Problemas" en [GUIA_COMPLETA_LIDAR.md](../../GUIA_COMPLETA_LIDAR.md)

