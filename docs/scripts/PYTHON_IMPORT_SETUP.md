# Configuración de Importaciones de Python

Este documento explica cómo resolver los problemas de importación del SDK de Sphero en el proyecto Atriz RVR.

## 🔧 Problema

El linter puede mostrar errores como:
- `No se ha podido resolver la importación 'sphero_sdk'`
- `ModuleNotFoundError: No module named 'sphero_sdk'`

## ✅ Solución

### **1. Archivos de Configuración Creados**

Se han creado varios archivos para resolver las importaciones:

#### **sphero_sdk_config.py**
```python
# atriz_rvr_driver/scripts/sphero_sdk_config.py
# Configura automáticamente la ruta del SDK de Sphero
```

#### **pyrightconfig.json**
```json
{
  "extraPaths": [
    "./atriz_rvr_driver/scripts",
    "./atriz_rvr_driver/scripts"
  ]
}
```

#### **.pylintrc**
```ini
[MASTER]
init-hook='import sys; sys.path.append("./atriz_rvr_driver/scripts")'
```

#### **.vscode/settings.json**
```json
{
  "python.analysis.extraPaths": [
    "./atriz_rvr_driver/scripts",
    "./atriz_rvr_driver/scripts"
  ]
}
```

### **2. Uso en el Código**

El archivo `Atriz_rvr_node.py` ahora incluye:

```python
# Configuración automática de rutas
from sphero_sdk_config import setup_sphero_sdk_path
setup_sphero_sdk_path()

# Importaciones del SDK (ahora funcionan)
from sphero_sdk.asyncio.client.toys.sphero_rvr_async import SpheroRvrAsync
```

### **3. Configuración Manual (si es necesario)**

Si aún hay problemas, ejecuta:

```bash
# Configurar PATH de Python
python3 setup_python_path.py

# O agregar manualmente
export PYTHONPATH="${PYTHONPATH}:$(pwd)/atriz_rvr_driver/scripts"
```

## 🚀 Verificación

### **Probar Importaciones**
```bash
python3 -c "
import sys
sys.path.insert(0, './atriz_rvr_driver/scripts')
from sphero_sdk.asyncio.client.toys.sphero_rvr_async import SpheroRvrAsync
print('✅ Importaciones funcionan correctamente')
"
```

### **Probar Script Principal**
```bash
python3 ./atriz_rvr_driver/scripts/Atriz_rvr_node.py --help
```

## 🔍 Solución de Problemas

### **Error: "No module named 'sphero_sdk'"**
1. Verificar que `atriz_rvr_driver/scripts/sphero_sdk/` existe
2. Ejecutar `python3 setup_python_path.py`
3. Verificar que el archivo `sphero_sdk_config.py` está en `atriz_rvr_driver/scripts/`

### **Error del Linter: "No se ha podido resolver la importación"**
1. Reiniciar el editor/IDE
2. Verificar que los archivos de configuración están en su lugar
3. Verificar que `.vscode/settings.json` tiene las rutas correctas

### **Error: "SDK path not found"**
1. Verificar que la estructura de carpetas es correcta
2. Verificar que `atriz_rvr_driver/scripts/` contiene el SDK
3. Verificar permisos de lectura en las carpetas

## 📁 Estructura de Archivos

```
atriz_rvr_driver/scripts/
├── Atriz_rvr_node.py          # Script principal (con configuración)
├── sphero_sdk_config.py       # Configuración del SDK
└── __init__.py                # Paquete Python

atriz_rvr_driver/scripts/
└── sphero_sdk/                # SDK de Sphero
    ├── asyncio/
    ├── common/
    └── ...

Configuración del Linter:
├── pyrightconfig.json         # Configuración de Pyright
├── .pylintrc                  # Configuración de Pylint
└── .vscode/settings.json      # Configuración de VS Code
```

## 🎯 Resultado

Con esta configuración:
- ✅ **Las importaciones funcionan** en tiempo de ejecución
- ✅ **El linter reconoce** las importaciones del SDK
- ✅ **El autocompletado funciona** en el IDE
- ✅ **No hay errores** de resolución de módulos

## 📝 Notas

- Los archivos de configuración son específicos para este proyecto
- No modificar las rutas sin actualizar todos los archivos de configuración
- Reiniciar el IDE después de cambios en la configuración
- Verificar que todos los archivos de configuración están en su lugar
