# Documentación del Proyecto Atriz RVR

Esta carpeta contiene toda la documentación del proyecto organizada por categorías.

## 📁 Estructura de Documentación

```
docs/
├── README.md                    # Este archivo (índice principal)
├── driver/                      # Documentación del driver
│   └── DRIVER_FUNCTIONALITY_GUIDE.md
├── testing/                     # Documentación de pruebas
│   └── README_TESTING.md
├── hardware/                    # Documentación de hardware
│   ├── README.md (rvr++)
│   └── README.md (serial_lib)
└── scripts/                     # Documentación de scripts
    ├── GRADOS_CONTROL_README.md
    └── GRADOS_DIRECTOS_README.md
```

## 🚀 Guías Principales

### **Driver y Funcionalidades**
- **[Guía de Funcionalidades del Driver](driver/DRIVER_FUNCTIONALITY_GUIDE.md)**
  - Tópicos publicados y suscritos
  - Servicios disponibles
  - Funciones de movimiento
  - Sistema de seguridad
  - Sensores disponibles
  - Comunicación IR
  - Control de LEDs
  - Parámetros de configuración

### **Pruebas y Testing**
- **[Guía de Pruebas](testing/README_TESTING.md)**
  - Scripts de prueba disponibles
  - Instrucciones de uso
  - Tópicos probados
  - Servicios probados
  - Funcionalidades de movimiento
  - Sistema de seguridad
  - Funcionalidad IR
  - Solución de problemas

## 🔧 Hardware y Scripts

### **Hardware**
- **[RVR++](hardware/README.md)** - Documentación del hardware RVR++
- **[Serial Library](hardware/README.md)** - Biblioteca de comunicación serial

### **Scripts**
- **[Control por Grados](scripts/GRADOS_CONTROL_README.md)** - Control de movimiento en grados
- **[Grados Directos](scripts/GRADOS_DIRECTOS_README.md)** - Control directo en grados

## 🎯 Guías de Inicio Rápido

### **Para Desarrolladores**
1. Lee la [Guía de Funcionalidades del Driver](driver/DRIVER_FUNCTIONALITY_GUIDE.md)
2. Consulta la [Guía de Pruebas](testing/README_TESTING.md)
3. Revisa la documentación de [Scripts](scripts/)

### **Para Usuarios**
1. Consulta la [Guía de Pruebas](testing/README_TESTING.md)
2. Usa los scripts de prueba organizados
3. Revisa la [Guía de Funcionalidades](driver/DRIVER_FUNCTIONALITY_GUIDE.md) para entender las capacidades

### **Para Testing**
1. Ve a la carpeta `testing_scripts/`
2. Lee el README de testing
3. Ejecuta `./run_tests.sh help` para ver opciones

## 📋 Índice de Contenidos

### **Driver Atriz RVR**
- [Tópicos Publicados](driver/DRIVER_FUNCTIONALITY_GUIDE.md#tópicos-publicados)
- [Tópicos Suscritos](driver/DRIVER_FUNCTIONALITY_GUIDE.md#tópicos-suscritos)
- [Servicios](driver/DRIVER_FUNCTIONALITY_GUIDE.md#servicios)
- [Funciones de Movimiento](driver/DRIVER_FUNCTIONALITY_GUIDE.md#funciones-de-movimiento)
- [Sistema de Seguridad](driver/DRIVER_FUNCTIONALITY_GUIDE.md#sistema-de-seguridad)
- [Sensores](driver/DRIVER_FUNCTIONALITY_GUIDE.md#sensores-disponibles)
- [Comunicación IR](driver/DRIVER_FUNCTIONALITY_GUIDE.md#comunicación-ir)
- [Control de LEDs](driver/DRIVER_FUNCTIONALITY_GUIDE.md#control-de-leds)

### **Testing y Pruebas**
- [Scripts de Prueba](testing/README_TESTING.md#archivos-de-prueba)
- [Instrucciones de Uso](testing/README_TESTING.md#instrucciones-de-uso)
- [Tópicos Probados](testing/README_TESTING.md#tópicos-probados)
- [Servicios Probados](testing/README_TESTING.md#servicios-probados)
- [Reinicio de Odometría](testing/README_TESTING.md#reinicio-de-odometría)
- [Solución de Problemas](testing/README_TESTING.md#solución-de-problemas)

## 🔗 Enlaces Útiles

### **Scripts de Prueba**
- **Script Principal**: `./run_tests.sh`
- **Carpeta de Testing**: `testing_scripts/`
- **Pruebas Automáticas**: `testing_scripts/automated/`
- **Pruebas Interactivas**: `testing_scripts/interactive/`
- **Diagnóstico**: `testing_scripts/diagnostic/`

### **Archivos de Configuración**
- **Driver Principal**: `atriz_rvr_driver/scripts/Atriz_rvr_node.py`
- **Scripts de Lanzamiento**: `testing_scripts/launch/`

## 📞 Soporte

Para problemas o preguntas:

1. **Revisa la documentación** en esta carpeta
2. **Ejecuta el diagnóstico**: `./run_tests.sh diagnostic`
3. **Consulta los logs** de ROS
4. **Revisa los reportes** generados en `/tmp/`

## 🔄 Mantenimiento

Para mantener la documentación actualizada:

1. **Actualiza los README** cuando agregues nuevas funcionalidades
2. **Mantén los índices** actualizados
3. **Revisa los enlaces** regularmente
4. **Actualiza los ejemplos** de uso

## 📝 Notas

- Todos los archivos README están organizados por categoría
- Los enlaces son relativos a esta carpeta
- La documentación está diseñada para ser fácil de navegar
- Cada sección tiene su propio README específico
