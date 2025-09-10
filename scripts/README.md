# Scripts del Proyecto Atriz RVR

Esta carpeta contiene todos los scripts del proyecto organizados por funcionalidad.

## 📁 Estructura de Scripts

```
scripts/
├── README.md              # Este archivo
├── core/                  # Scripts principales del sistema
│   ├── Atriz_rvr_node.py  # Driver principal del robot
│   ├── emergency_stop.py  # Sistema de parada de emergencia
│   └── rvr-ros-restarter.py # Reiniciador automático
├── examples/              # Ejemplos de uso
│   ├── degrees_control_example.py
│   ├── example_degrees_control.py
│   ├── random_walking.py
│   └── rvr_joystick_control.py
├── tools/                 # Herramientas de utilidad
│   ├── cmd_vel_rviz.py    # Control desde RViz
│   ├── color_listener.py  # Listener de color
│   └── rvr_tools.py       # Herramientas generales
└── utilities/             # Scripts de utilidad
    └── test_both_topics.py # Prueba de tópicos
```

## 🚀 Scripts Principales (Core)

### **Atriz_rvr_node.py**
- **Función**: Driver principal del robot Sphero RVR
- **Uso**: `python3 Atriz_rvr_node.py`
- **Descripción**: Nodo ROS que controla el robot, maneja sensores y publica datos

### **emergency_stop.py**
- **Función**: Sistema de parada de emergencia
- **Uso**: `python3 emergency_stop.py`
- **Descripción**: Monitorea y maneja paradas de emergencia

### **rvr-ros-restarter.py**
- **Función**: Reiniciador automático del nodo
- **Uso**: `python3 rvr-ros-restarter.py`
- **Descripción**: Reinicia automáticamente el nodo si falla

## 📚 Ejemplos de Uso (Examples)

### **degrees_control_example.py**
- **Función**: Ejemplo de control en grados
- **Uso**: `python3 degrees_control_example.py`
- **Descripción**: Demuestra cómo usar comandos en grados

### **example_degrees_control.py**
- **Función**: Otro ejemplo de control en grados
- **Uso**: `python3 example_degrees_control.py`
- **Descripción**: Ejemplo alternativo de control

### **random_walking.py**
- **Función**: Caminata aleatoria del robot
- **Uso**: `python3 random_walking.py`
- **Descripción**: El robot se mueve aleatoriamente

### **rvr_joystick_control.py**
- **Función**: Control con joystick
- **Uso**: `python3 rvr_joystick_control.py`
- **Descripción**: Controla el robot con un joystick

## 🔧 Herramientas (Tools)

### **cmd_vel_rviz.py**
- **Función**: Control desde RViz
- **Uso**: `python3 cmd_vel_rviz.py`
- **Descripción**: Permite controlar el robot desde RViz

### **color_listener.py**
- **Función**: Listener de datos de color
- **Uso**: `python3 color_listener.py`
- **Descripción**: Escucha y muestra datos del sensor de color

### **rvr_tools.py**
- **Función**: Herramientas generales
- **Uso**: `python3 rvr_tools.py`
- **Descripción**: Utilidades y funciones auxiliares

## 🛠️ Utilidades (Utilities)

### **test_both_topics.py**
- **Función**: Prueba de tópicos
- **Uso**: `python3 test_both_topics.py`
- **Descripción**: Prueba ambos tópicos de comando

## 🚀 Uso Rápido

### **1. Ejecutar el Driver Principal**
```bash
cd atriz_rvr_driver/scripts
python3 Atriz_rvr_node.py
```

### **2. Ejecutar un Ejemplo**
```bash
cd scripts/examples
python3 random_walking.py
```

### **3. Usar una Herramienta**
```bash
cd scripts/tools
python3 color_listener.py
```

## 📋 Requisitos

- ROS Noetic instalado
- Python 3.8+
- Dependencias del proyecto instaladas
- Robot Sphero RVR conectado

## 🔗 Enlaces Relacionados

- **[Documentación Principal](../docs/README.md)**
- **[Guía del Driver](../docs/driver/DRIVER_FUNCTIONALITY_GUIDE.md)**
- **[Scripts de Prueba](../testing_scripts/README.md)**

## 📝 Notas

- Todos los scripts están organizados por funcionalidad
- Los scripts core son esenciales para el funcionamiento
- Los ejemplos muestran diferentes formas de usar el robot
- Las herramientas facilitan el desarrollo y debugging
