# Servicio RGBC - Medición de Color Sin LED

## 🎯 Objetivo

Este servicio permite hacer mediciones de color del Sphero RVR **SIN activar el LED del sensor**, utilizando la función `get_rgbc_sensor_values` de la SDK directamente.

## 🔬 Análisis Técnico

### ¿Por qué funciona?

1. **`get_rgbc_sensor_values` es independiente**: No requiere `enable_color_detection`
2. **Acceso directo al hardware**: Bypasea el sistema de streaming de ROS
3. **Sin activación de LED**: El LED solo se activa con `enable_color_detection`

### Arquitectura de la Solución

```
Cliente ROS → Servicio RGBC → SDK get_rgbc_sensor_values → Hardware
     ↓              ↓                    ↓                    ↓
  Solicitud    Procesamiento      Comando 0x23         Datos Raw
```

## 📁 Archivos Creados

- `rgbc_sensor_service.py` - Servicio ROS principal
- `test_rgbc_service.py` - Cliente de prueba completo
- `test_rgbc_simple.py` - Cliente de prueba simple
- `launch_rgbc_service.sh` - Script de lanzamiento
- `GetRGBCSensorValues.srv` - Definición del servicio

## 🚀 Uso

### 1. Lanzar el Servicio

```bash
# Opción 1: Script de lanzamiento
./scripts/launch_rgbc_service.sh

# Opción 2: Directamente
python3 scripts/rgbc_sensor_service.py
```

### 2. Probar el Servicio

```bash
# Prueba simple
python3 scripts/test_rgbc_simple.py

# Prueba completa
python3 scripts/examples/test_rgbc_service.py

# Prueba continua
python3 scripts/examples/test_rgbc_service.py continuous

# Suite de pruebas
python3 scripts/examples/test_rgbc_service.py test
```

### 3. Usar desde Código

```python
#!/usr/bin/env python3

import rospy
from atriz_rvr_msgs.srv import GetRGBCSensorValues

def get_color_without_led():
    """Obtiene datos de color sin activar el LED."""
    rospy.wait_for_service('/get_rgbc_sensor_values')
    rgbc_service = rospy.ServiceProxy('/get_rgbc_sensor_values', GetRGBCSensorValues)
    
    response = rgbc_service()
    
    if response.success:
        print(f"Red: {response.red_channel_value}")
        print(f"Green: {response.green_channel_value}")
        print(f"Blue: {response.blue_channel_value}")
        print(f"Clear: {response.clear_channel_value}")
        return (response.red_channel_value, 
                response.green_channel_value, 
                response.blue_channel_value, 
                response.clear_channel_value)
    else:
        print(f"Error: {response.message}")
        return None
```

## 🔧 Configuración

### Requisitos

- ROS Noetic
- Driver del RVR ejecutándose
- Conexión serial al RVR (`/dev/ttyS0`)

### Parámetros del Servicio

- **Puerto**: `/dev/ttyS0` (configurable en el código)
- **Baudrate**: 115200
- **Timeout**: 5 segundos por solicitud

## 📊 Comparación de Métodos

| Método | LED Activado | Datos | Latencia | Uso |
|--------|--------------|-------|----------|-----|
| `/enable_color` | ✅ Sí | Procesados | Baja | Streaming |
| `get_rgbc_sensor_values` | ❌ No | Raw | Media | Puntual |

## ⚠️ Limitaciones

1. **Datos Raw**: Los valores son raw del sensor, no procesados
2. **Sin Procesamiento**: No hay detección de color automática
3. **Conexión Directa**: Requiere conexión serial al RVR
4. **Sin Streaming**: Solo mediciones puntuales

## 🎉 Ventajas

1. **Sin LED**: No activa el LED del sensor
2. **Datos Raw**: Acceso completo a los datos del sensor
3. **Control Total**: Control completo sobre cuándo medir
4. **Eficiencia**: Solo mide cuando se solicita

## 🔍 Verificación

Para verificar que el LED no se activa:

1. **Observación Visual**: El LED del sensor no debe encenderse
2. **Logs del Driver**: No debe aparecer activación del LED
3. **Comparación**: Comparar con `/enable_color` (que sí activa el LED)

## 🛠️ Desarrollo

### Modificar el Servicio

```python
# En rgbc_sensor_service.py
async def initialize_connection(self):
    # Cambiar puerto
    self.dal = SerialAsyncDal(loop, port_id='/dev/ttyUSB0', baud=115200)
    
    # Cambiar timeout
    result = await self.rvr.get_rgbc_sensor_values(timeout=10.0)
```

### Agregar Funcionalidades

- Procesamiento de datos raw
- Filtrado de ruido
- Calibración del sensor
- Múltiples mediciones

## 📈 Rendimiento

- **Latencia**: ~100-500ms por medición
- **Throughput**: 1-2 mediciones por segundo
- **Memoria**: Mínima (solo datos de una medición)
- **CPU**: Bajo (solo durante medición)

## 🐛 Solución de Problemas

### Error: "RVR no conectado"
- Verificar que el driver esté ejecutándose
- Verificar conexión serial
- Verificar permisos del puerto

### Error: "No se recibieron datos"
- Verificar que el RVR esté despierto
- Verificar conexión
- Aumentar timeout

### Error: "Servicio no disponible"
- Verificar que el servicio esté ejecutándose
- Verificar que ROS esté ejecutándose
- Verificar nombres de servicios

## 🎯 Conclusión

Esta solución demuestra que **SÍ ES POSIBLE** hacer mediciones de color sin activar el LED del sensor, utilizando `get_rgbc_sensor_values` directamente desde la SDK. La implementación como servicio ROS permite un uso fácil y controlado de esta funcionalidad.
