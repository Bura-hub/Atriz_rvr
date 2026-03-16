# Scripts de Prueba del Driver Atriz RVR

Esta carpeta contiene todos los scripts de prueba organizados por categorías para facilitar su uso y mantenimiento.

## 📁 Estructura de Carpetas

```
testing_scripts/
├── automated/          # Pruebas automáticas
├── interactive/        # Pruebas interactivas
├── diagnostic/         # Scripts de diagnóstico
├── launch/            # Scripts de lanzamiento
└── README.md          # Este archivo
```

## 🚀 Scripts Disponibles

### **Pruebas Automáticas** (`automated/`)
Scripts que ejecutan todas las pruebas sin intervención del usuario.

#### `test_atriz_rvr_driver.py`
- **Descripción**: Pruebas automáticas completas del driver
- **Uso**: `python3 test_atriz_rvr_driver.py`
- **Funcionalidades**: Todas las pruebas en secuencia automática

#### `run_complete_tests.py`
- **Descripción**: Suite completa con diagnóstico incluido
- **Uso**: `python3 run_complete_tests.py`
- **Funcionalidades**: Diagnóstico + pruebas automáticas + reporte completo

### **Pruebas Interactivas** (`interactive/`)
Scripts que permiten probar funcionalidades individuales.

#### `test_individual_functions.py`
- **Descripción**: Menú interactivo para pruebas individuales
- **Uso**: `python3 test_individual_functions.py`
- **Funcionalidades**: 14 opciones de prueba individual

### **Diagnóstico** (`diagnostic/`)
Scripts para diagnosticar el estado del sistema.

#### `diagnose_system.py`
- **Descripción**: Diagnóstico completo del sistema
- **Uso**: `python3 diagnose_system.py`
- **Funcionalidades**: Verificación de ROS, driver, tópicos, servicios

### **Lanzamiento** (`launch/`)
Scripts para lanzar las pruebas con opciones.

#### `launch_tests.sh`
- **Descripción**: Script de lanzamiento con opciones
- **Uso**: `./launch_tests.sh [opción]`
- **Opciones**:
  - `all`: Ejecutar todas las pruebas automáticamente
  - `individual`: Ejecutar pruebas individuales interactivas
  - `driver`: Solo lanzar el driver (sin pruebas)
  - `cleanup`: Limpiar procesos en ejecución

## 🎯 Guía de Uso Rápido

### **1. Pruebas Rápidas**
```bash
cd testing_scripts/automated
python3 test_atriz_rvr_driver.py
```

### **2. Pruebas Interactivas**
```bash
cd testing_scripts/interactive
python3 test_individual_functions.py
```

### **3. Diagnóstico del Sistema**
```bash
cd testing_scripts/diagnostic
python3 diagnose_system.py
```

### **4. Lanzamiento con Opciones**
```bash
cd testing_scripts/launch
./launch_tests.sh all
```

## 📋 Prerequisitos

1. **ROS ejecutándose**:
   ```bash
   roscore
   ```

2. **Driver Atriz RVR ejecutándose**:
   ```bash
   python3 ../atriz_rvr_driver/scripts/Atriz_rvr_node.py
   ```

3. **Robot Sphero RVR conectado y emparejado**

## 🔧 Configuración

### **Hacer Scripts Ejecutables**
```bash
# Desde la carpeta testing_scripts
chmod +x launch/launch_tests.sh
chmod +x automated/*.py
chmod +x interactive/*.py
chmod +x diagnostic/*.py
```

### **Configurar PATH (Opcional)**
```bash
# Agregar al .bashrc para usar desde cualquier lugar
export ATRIZ_TESTING_PATH="/home/sphero/atriz_git/src/Atriz_rvr/testing_scripts"
```

## 📊 Reportes Generados

Los scripts generan reportes en formato JSON en `/tmp/`:

- `atriz_rvr_test_report.json` - Reporte de pruebas básicas
- `atriz_rvr_diagnostic_report.json` - Reporte de diagnóstico
- `atriz_rvr_complete_test_report.json` - Reporte completo

## 🎮 Ejemplos de Uso

### **Ejemplo 1: Prueba Completa**
```bash
cd testing_scripts/launch
./launch_tests.sh all
```

### **Ejemplo 2: Prueba Individual**
```bash
cd testing_scripts/interactive
python3 test_individual_functions.py
# Seleccionar opción 1 (movimiento hacia adelante)
```

### **Ejemplo 3: Diagnóstico**
```bash
cd testing_scripts/diagnostic
python3 diagnose_system.py
```

## 🚨 Solución de Problemas

### **1. Scripts no ejecutables**
```bash
chmod +x testing_scripts/launch/launch_tests.sh
chmod +x testing_scripts/automated/*.py
chmod +x testing_scripts/interactive/*.py
chmod +x testing_scripts/diagnostic/*.py
```

### **2. Driver no encontrado**
```bash
# Asegúrate de estar en la carpeta correcta
cd /home/sphero/atriz_git/src/Atriz_rvr
```

### **3. ROS no ejecutándose**
```bash
# Iniciar ROS
roscore
```

## 📚 Documentación Adicional

- **README_TESTING.md**: Guía completa de pruebas
- **DRIVER_FUNCTIONALITY_GUIDE.md**: Guía de funcionalidades del driver

## 🔄 Actualizaciones

Para mantener los scripts actualizados:

1. **Verificar dependencias**: Asegúrate de que todas las dependencias estén instaladas
2. **Probar scripts**: Ejecuta las pruebas regularmente
3. **Revisar logs**: Monitorea los logs de ROS para errores
4. **Actualizar documentación**: Mantén la documentación actualizada

## 📞 Soporte

Para problemas o preguntas:
1. Revisar los logs de ROS
2. Ejecutar el diagnóstico del sistema
3. Verificar la documentación
4. Consultar los reportes generados
