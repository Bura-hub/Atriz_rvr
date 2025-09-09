# Paquetes ROS del Proyecto Atriz RVR

Esta carpeta contiene todos los paquetes ROS del proyecto organizados de manera lógica.

## 📁 Estructura de Paquetes

```
packages/
├── README.md                    # Este archivo
├── atriz_rvr_driver/           # Paquete principal del driver
│   ├── package.xml
│   ├── CMakeLists.txt
│   ├── setup.py
│   ├── scripts/                # Scripts del paquete
│   ├── rvr++/                  # Hardware RVR++
│   └── sphero_sdk/             # SDK de Sphero
├── atriz_rvr_msgs/             # Mensajes personalizados
│   ├── package.xml
│   ├── CMakeLists.txt
│   └── msg/                    # Definiciones de mensajes
└── atriz_rvr_serial/           # Biblioteca de comunicación serial
    ├── package.xml
    ├── CMakeLists.txt
    └── tests/                  # Pruebas de la biblioteca
```

## 🚀 Paquetes Principales

### **atriz_rvr_driver**
- **Función**: Paquete principal del driver del robot
- **Contenido**: Driver principal, SDK de Sphero, hardware RVR++
- **Dependencias**: atriz_rvr_msgs, atriz_rvr_serial
- **Uso**: `roslaunch atriz_rvr_driver atriz_rvr.launch`

### **atriz_rvr_msgs**
- **Función**: Definiciones de mensajes personalizados
- **Contenido**: Mensajes para comunicación con el robot
- **Dependencias**: std_msgs, geometry_msgs, sensor_msgs
- **Uso**: Importado por otros paquetes

### **atriz_rvr_serial**
- **Función**: Biblioteca de comunicación serial
- **Contenido**: Funciones para comunicación serial con el robot
- **Dependencias**: Ninguna
- **Uso**: Usado por el driver principal

## 🔧 Estructura Detallada

### **atriz_rvr_driver/**
```
atriz_rvr_driver/
├── package.xml              # Metadatos del paquete
├── CMakeLists.txt           # Configuración de build
├── setup.py                 # Configuración de Python
├── scripts/                 # Scripts del paquete
│   ├── Atriz_rvr_node.py   # Driver principal
│   ├── emergency_stop.py   # Parada de emergencia
│   └── ...                 # Otros scripts
├── rvr++/                   # Hardware RVR++
│   └── ref/                # Referencias de hardware
└── sphero_sdk/             # SDK de Sphero
    ├── asyncio/            # Cliente asíncrono
    ├── common/             # Funciones comunes
    └── observer/           # Patrón observer
```

### **atriz_rvr_msgs/**
```
atriz_rvr_msgs/
├── package.xml              # Metadatos del paquete
├── CMakeLists.txt           # Configuración de build
└── msg/                     # Definiciones de mensajes
    ├── Color.msg            # Mensaje de color
    ├── DegreesTwist.msg     # Mensaje de velocidad en grados
    └── BatteryState.msg     # Mensaje de estado de batería
```

### **atriz_rvr_serial/**
```
atriz_rvr_serial/
├── package.xml              # Metadatos del paquete
├── CMakeLists.txt           # Configuración de build
└── tests/                   # Pruebas de la biblioteca
    └── proof_of_concepts/   # Pruebas de concepto
```

## 🚀 Uso de los Paquetes

### **1. Construir los Paquetes**
```bash
cd packages
catkin_make
```

### **2. Ejecutar el Driver**
```bash
roslaunch atriz_rvr_driver atriz_rvr.launch
```

### **3. Usar Mensajes Personalizados**
```python
from atriz_rvr_msgs.msg import Color, DegreesTwist, BatteryState
```

### **4. Usar la Biblioteca Serial**
```python
from atriz_rvr_serial import SerialCommunication
```

## 📋 Dependencias

### **atriz_rvr_driver**
- `atriz_rvr_msgs` - Mensajes personalizados
- `atriz_rvr_serial` - Comunicación serial
- `std_msgs` - Mensajes estándar
- `geometry_msgs` - Mensajes geométricos
- `sensor_msgs` - Mensajes de sensores
- `nav_msgs` - Mensajes de navegación

### **atriz_rvr_msgs**
- `std_msgs` - Mensajes estándar
- `geometry_msgs` - Mensajes geométricos
- `sensor_msgs` - Mensajes de sensores

### **atriz_rvr_serial**
- Ninguna dependencia externa

## 🔗 Enlaces Relacionados

- **[Scripts del Proyecto](../scripts/README.md)**
- **[Documentación Principal](../docs/README.md)**
- **[Scripts de Prueba](../testing_scripts/README.md)**

## 📝 Notas

- Los paquetes están organizados por funcionalidad
- Cada paquete tiene sus propias dependencias
- La estructura sigue las convenciones de ROS
- Los paquetes están diseñados para ser independientes pero complementarios
