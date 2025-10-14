# 📐 Referencias Técnicas - Sphero RVR

## 📚 Contenido
1. [Permisos de Ejecución](#permisos-de-ejecución)
2. [Cálculos de Giro](#cálculos-de-giro)
3. [Tablas de Referencia](#tablas-de-referencia)
4. [Fórmulas Matemáticas](#fórmulas-matemáticas)
5. [Conversiones](#conversiones)
6. [Límites y Rangos](#límites-y-rangos)
7. [FAQ Técnico](#faq-técnico)

> **📖 Para conexión WiFi, credenciales y configuración inicial, ver:**  
> **[`00_LEEME_PRIMERO.md`](00_LEEME_PRIMERO.md)**

---

## 🔐 Permisos de Ejecución

### ¿Cuándo son Necesarios?

**Scripts Python (ejecutados con `python3`):**
```bash
# ❌ NO necesitan permisos de ejecución
python3 01_avanzar.py  # ✅ Funciona directamente
python3 script.py      # ✅ Python interpreta el archivo

# Solo necesitarían permisos si quisieras ejecutarlos así:
./01_avanzar.py        # Requiere chmod +x y shebang #!/usr/bin/env python3
```

**Scripts Shell (ejecutados directamente):**
```bash
# ✅ SÍ necesitan permisos de ejecución
./start_ros.sh         # Requiere permisos

# Dar permisos (solo la primera vez):
chmod +x start_ros.sh
```

### Permisos en Git

**Git MANTIENE los permisos de ejecución:**

```bash
# 1. En tu máquina local - dar permisos
chmod +x start_ros.sh

# 2. Verificar permisos (debe mostrar 'x')
ls -la start_ros.sh
# Salida: -rwxrwxr-x  (las 'x' indican ejecución)

# 3. Agregar y subir a Git
git add start_ros.sh
git commit -m "Add execution permissions to start_ros.sh"
git push

# 4. En otra máquina - clonar repo
git clone <repositorio>
cd <repositorio>

# 5. Los permisos YA están establecidos
ls -la start_ros.sh
# Salida: -rwxrwxr-x  ✅ Permisos mantenidos

# ✅ NO necesitas volver a hacer chmod +x
./start_ros.sh  # Ya funciona directamente
```

### Tabla de Permisos

| Tipo de Archivo | Ejecución | Necesita chmod +x | Se mantiene en Git |
|-----------------|-----------|-------------------|-------------------|
| `script.py` ejecutado con `python3 script.py` | ❌ NO | ❌ NO | N/A |
| `script.py` ejecutado con `./script.py` | ✅ SÍ | ✅ SÍ | ✅ SÍ |
| `script.sh` ejecutado con `./script.sh` | ✅ SÍ | ✅ SÍ | ✅ SÍ |
| `script.sh` ejecutado con `bash script.sh` | ❌ NO | ❌ NO | N/A |

### Comandos Útiles

```bash
# Ver permisos de un archivo
ls -la archivo.sh

# Dar permisos de ejecución
chmod +x archivo.sh

# Quitar permisos de ejecución
chmod -x archivo.sh

# Ver permisos de todos los archivos Python
ls -la *.py

# Dar permisos a todos los scripts shell
chmod +x *.sh

# Verificar si un archivo es ejecutable
test -x archivo.sh && echo "Es ejecutable" || echo "No es ejecutable"
```

### Permisos en el Proyecto Actual

**Archivos que YA tienen permisos (no requieren chmod):**
```bash
# Si clonaste este repositorio, estos archivos YA son ejecutables:
./start_ros.sh                    # ✅ Tiene permisos
./atriz_rvr_driver/scripts/*.py   # ✅ Tienen permisos (opcional)
./scripts/estudiantes/*.py         # ✅ Tienen permisos (opcional)
```

**Cómo ejecutar los scripts del proyecto:**
```bash
# Método 1: Con python3 (NO requiere permisos)
python3 01_avanzar.py              # ✅ Recomendado

# Método 2: Directamente (requiere permisos, ya establecidos)
./01_avanzar.py                    # ✅ También funciona
```

### Preguntas Frecuentes

**P: ¿Por qué mis scripts tienen permisos pero los ejecuto con `python3`?**
- Los permisos están establecidos por compatibilidad
- Ambos métodos funcionan: `python3 script.py` o `./script.py`
- Recomendamos `python3` para mayor claridad

**P: ¿Debo hacer `chmod +x` después de clonar el repositorio?**
- **NO** - Git mantiene los permisos de ejecución
- Los archivos ya vienen con permisos si se subieron correctamente

**P: ¿Qué significa `-rwxrwxr-x`?**
```
-rwxrwxr-x
│││││││││└─ Otros pueden ejecutar
││││││││└── Otros pueden leer
│││││││└─── Otros NO pueden escribir
││││││└──── Grupo puede ejecutar
│││││└───── Grupo puede leer
││││└────── Grupo puede escribir
│││└─────── Usuario puede ejecutar
││└──────── Usuario puede leer
│└───────── Usuario puede escribir
└────────── Es un archivo (no directorio)
```

**P: ¿Por qué algunos scripts tienen shebang `#!/usr/bin/env python3`?**
- Permite ejecutar el script directamente: `./script.py`
- Sin shebang, necesitas especificar el intérprete: `python3 script.py`

---

## 🔄 Cálculos de Giro

### Fórmula Fundamental

```
Tiempo = Ángulo_radianes / Velocidad_angular

Donde:
- Tiempo: segundos
- Ángulo: radianes
- Velocidad_angular: rad/s
```

### Ejemplo Paso a Paso

**Quiero girar 90°:**

1. **Convertir a radianes:**
   ```
   90° × (π / 180) = 1.571 rad
   ```

2. **Aplicar fórmula (velocidad = 0.5 rad/s):**
   ```
   Tiempo = 1.571 / 0.5 = 3.14 segundos
   ```

3. **En código:**
   ```python
   girar(segundos=3.14)  # 90 grados
   ```

### ⚠️ Error Común Corregido

**INCORRECTO:**
```python
girar(1.57)  # Esto solo gira 45°, NO 90°
```

**CORRECTO:**
```python
girar(3.14)  # Esto SÍ gira 90°
```

**Explicación:**
- 1.57 es el ángulo en radianes (π/2)
- Pero necesitamos el TIEMPO, no el ángulo
- Tiempo = 1.57 rad / 0.5 rad/s = **3.14 segundos**

---

## 📊 Tablas de Referencia

### Tabla 1: Giros Comunes (velocidad = 0.5 rad/s)

| Ángulo | Radianes | Tiempo | Uso Típico |
|--------|----------|--------|------------|
| 30° | 0.524 | 1.05 s | Giro suave |
| 45° | 0.785 | 1.57 s | Octágono |
| 60° | 1.047 | 2.09 s | Hexágono |
| 72° | 1.257 | 2.51 s | Pentágono |
| **90°** | **1.571** | **3.14 s** | **Cuadrado** ⭐ |
| 108° | 1.885 | 3.77 s | Pentágono interior |
| **120°** | **2.094** | **4.19 s** | **Triángulo** ⭐ |
| 135° | 2.356 | 4.71 s | Octágono exterior |
| 144° | 2.513 | 5.03 s | Estrella 5 puntas |
| **180°** | **3.142** | **6.28 s** | **Media vuelta** |
| 270° | 4.712 | 9.42 s | 3/4 de vuelta |
| **360°** | **6.283** | **12.57 s** | **Vuelta completa** |

### Tabla 2: Velocidades Lineales Seguras

| Velocidad | m/s | Uso | Descripción |
|-----------|-----|-----|-------------|
| Muy lento | 0.1 | Precisión | Maniobras delicadas |
| Lento | 0.2 | Seguidor línea | Velocidad estable |
| Normal | 0.3 | General | **Default recomendado** ⭐ |
| Rápido | 0.5 | Rectas largas | Requiere espacio |
| Muy rápido | 0.8 | Carreras | Solo con experiencia |
| Máximo | 1.0 | Límite | ⚠️ Difícil control |

### Tabla 3: Velocidades Angulares

| Velocidad | rad/s | Uso | Descripción |
|-----------|-------|-----|-------------|
| Muy lento | 0.2 | Ajuste fino | Giros precisos |
| Lento | 0.3 | Maniobras | Giros controlados |
| Normal | 0.5 | General | **Default recomendado** ⭐ |
| Rápido | 0.8 | Giros rápidos | Cambios de dirección |
| Muy rápido | 1.5 | Spin | Giros sobre eje |
| Máximo | 2.0 | Límite | ⚠️ Puede perder orientación |

### Tabla 4: Figuras Geométricas

| Figura | Lados | Ángulo Exterior | Tiempo (0.5 rad/s) | Código |
|--------|-------|-----------------|-------------------|--------|
| Triángulo | 3 | 120° | 4.19 s | `girar(4.19)` |
| Cuadrado | 4 | 90° | 3.14 s | `girar(3.14)` |
| Pentágono | 5 | 72° | 2.51 s | `girar(2.51)` |
| Hexágono | 6 | 60° | 2.09 s | `girar(2.09)` |
| Octágono | 8 | 45° | 1.57 s | `girar(1.57)` |
| Decágono | 10 | 36° | 1.26 s | `girar(1.26)` |

**Fórmula para cualquier polígono regular:**
```
Ángulo_exterior = 360° / número_de_lados
```

### Tabla 5: Valores RGB Típicos

| Color | R | G | B | Notas |
|-------|---|---|---|-------|
| Blanco | 200-255 | 200-255 | 200-255 | Fondo típico |
| Negro | 0-50 | 0-50 | 0-50 | Línea típica |
| Rojo | 200-255 | 0-50 | 0-50 | Obstáculo/Stop |
| Verde | 0-50 | 200-255 | 0-50 | Go/Inicio |
| Azul | 0-50 | 0-50 | 200-255 | Marcador |
| Amarillo | 200-255 | 200-255 | 0-50 | Advertencia |
| Gris | 100-150 | 100-150 | 100-150 | Intermedio |

**Nota:** Estos valores son aproximados. **SIEMPRE calibrar** con tu pista específica.

---

## 🧮 Fórmulas Matemáticas

### Conversión Grados ↔ Radianes

```
Radianes = Grados × (π / 180)
Grados = Radianes × (180 / π)

Donde π ≈ 3.14159
```

**Ejemplos:**
```
45° = 45 × (π/180) = 0.785 rad
90° = 90 × (π/180) = 1.571 rad
180° = 180 × (π/180) = 3.142 rad

1 rad = 1 × (180/π) = 57.3°
π rad = π × (180/π) = 180°
2π rad = 2π × (180/π) = 360°
```

### Cálculo de Tiempo de Giro

```python
import math

def calcular_tiempo_giro(grados, velocidad_angular=0.5):
    """
    Calcula tiempo necesario para girar un ángulo.
    
    Args:
        grados: Ángulo a girar
        velocidad_angular: Velocidad en rad/s
    
    Returns:
        Tiempo en segundos
    """
    radianes = grados * (math.pi / 180)
    tiempo = radianes / velocidad_angular
    return tiempo

# Ejemplos:
tiempo_90 = calcular_tiempo_giro(90)     # 3.14 s
tiempo_180 = calcular_tiempo_giro(180)   # 6.28 s
tiempo_45 = calcular_tiempo_giro(45)     # 1.57 s
```

### Distancia Recorrida

```
Distancia = Velocidad × Tiempo

d = v × t
```

**Ejemplos:**
```
v = 0.3 m/s, t = 2 s  →  d = 0.6 m
v = 0.5 m/s, t = 5 s  →  d = 2.5 m
v = 0.2 m/s, t = 10 s →  d = 2.0 m
```

### Radio de Giro en Curva

```
Radio = Velocidad_lineal / Velocidad_angular

r = v / ω
```

**Ejemplos:**
```
v = 0.3 m/s, ω = 0.5 rad/s  →  r = 0.6 m
v = 0.2 m/s, ω = 1.0 rad/s  →  r = 0.2 m (curva cerrada)
v = 0.5 m/s, ω = 0.3 rad/s  →  r = 1.67 m (curva amplia)
```

---

## 🔄 Conversiones Útiles

### Unidades de Longitud
```
1 metro (m) = 100 centímetros (cm)
1 m = 1000 milímetros (mm)
1 m ≈ 3.28 pies (ft)

Ejemplos:
0.3 m = 30 cm
1.5 m = 150 cm
2.0 m = 2000 mm
```

### Unidades de Ángulo
```
1 vuelta completa = 360° = 2π rad
1 rad = 57.3°
1° = 0.01745 rad
π rad = 180°
π/2 rad = 90°
```

### Unidades de Velocidad Angular
```
1 rad/s = 57.3 °/s
1 rpm (revolución/min) = 0.105 rad/s
1 vuelta/s = 6.28 rad/s (2π rad/s)
```

---

## ⚙️ Límites y Rangos del Robot

### Velocidades Seguras
```
Velocidad Lineal:
├── Mínima práctica: 0.05 m/s
├── Recomendada: 0.2 - 0.4 m/s
├── Máxima segura: 0.8 m/s
└── Límite absoluto: ~1.0 m/s

Velocidad Angular:
├── Mínima práctica: 0.1 rad/s
├── Recomendada: 0.3 - 0.7 rad/s
├── Máxima segura: 1.5 rad/s
└── Límite absoluto: ~2.0 rad/s
```

### Frecuencias de Publicación
```
Mínima: 5 Hz (5 msg/s) - Movimiento errático
Recomendada: 10 Hz (10 msg/s) ⭐
Óptima: 20-30 Hz - Movimiento suave
Máxima: 50 Hz - Sin beneficio extra
```

### Batería
```
Voltaje:
├── Completo: 8.4 V
├── Normal: 7.0 - 8.0 V
├── Bajo: 6.5 - 7.0 V (advertencia)
├── Crítico: < 6.5 V (detener operación)
└── Mínimo: 6.0 V (apagado automático)

Porcentaje:
├── 100%: Totalmente cargado
├── 20-30%: Considerar recarga
├── 10%: Advertencia activa
└── < 10%: Detener robot
```

---

## ❓ FAQ Técnico

### P: ¿Por qué mi robot gira menos de lo esperado?

**R:** Posibles causas:
1. **Superficie resbaladiza:** Ruedas patinan
   - Solución: Usar superficie con tracción
2. **Batería baja:** Velocidad inconsistente
   - Solución: Cargar batería
3. **Peso desequilibrado:** Afecta el giro
   - Solución: Centrar peso del robot
4. **Cálculo incorrecto:** Ver [Cálculos de Giro](#cálculos-de-giro)

### P: ¿Cómo calibrar el sensor de color?

**R:** Proceso:
```python
python3 11_sensor_avanzado.py
# Seleccionar opción 1: Calibración
# Colocar robot sobre cada color cuando se indique
# Anotar valores RGB promedio
# Actualizar umbrales en tu código
```

### P: ¿Qué velocidad usar para seguidor de línea?

**R:** Recomendaciones:
- **Inicio:** 0.15 - 0.20 m/s (lento pero estable)
- **Intermedio:** 0.25 - 0.30 m/s (equilibrio)
- **Avanzado:** 0.35 - 0.45 m/s (rápido, requiere PID)

### P: ¿Cómo hacer giros más precisos?

**R:** Técnicas:
1. **Usar velocidad angular baja:** 0.3 rad/s en vez de 0.5
2. **Función con cálculo exacto:** Usar `04_giro_preciso.py`
3. **Calibrar tiempos:** Experimentar con tu robot específico
4. **Superficie consistente:** Mismo tipo de piso siempre

### P: ¿Cuál es la diferencia entre linear.x y angular.z?

**R:**
```python
cmd.linear.x    # Velocidad de TRASLACIÓN (adelante/atrás)
                # Positivo = adelante, Negativo = atrás
                # Unidad: m/s (metros por segundo)

cmd.angular.z   # Velocidad de ROTACIÓN (giro)
                # Positivo = izquierda, Negativo = derecha
                # Unidad: rad/s (radianes por segundo)
```

### P: ¿Puedo combinar linear.x y angular.z?

**R:** ¡Sí! Eso crea movimiento en **curva**:
```python
cmd.linear.x = 0.3   # Avanza
cmd.angular.z = 0.5  # Y gira (→ curva izquierda)

# Radio de la curva: r = 0.3 / 0.5 = 0.6 metros
```

### P: ¿Qué es el `queue_size` en el Publisher?

**R:**
```python
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
                                         ↑
                # Tamaño de cola de mensajes

queue_size pequeño (1-5):   Mensajes recientes, puede perder datos
queue_size medio (10):      Balance (recomendado) ⭐
queue_size grande (50-100): Todos los mensajes, puede haber delay
```

### P: ¿Cómo detener el robot inmediatamente?

**R:** Dos métodos:

**Método 1: En código (mejor)**
```python
cmd = Twist()  # Velocidades en 0
for _ in range(10):
    pub.publish(cmd)
    rospy.sleep(0.1)
```

**Método 2: Línea de comandos (emergencia)**
```bash
rostopic pub -1 /cmd_vel geometry_msgs/Twist '{}'
```

### P: ¿Cómo detener roscore?

**R:** Varias opciones según la situación:

**Opción 1: Suave (recomendado)**
```bash
pkill -f roscore
```

**Opción 2: Forzado (si no responde)**
```bash
pkill -9 -f roscore
```

**Opción 3: Si está en terminal visible**
```bash
# Presionar Ctrl+C en la terminal donde corre roscore
```

**Opción 4: Limpiar TODO el entorno ROS**
```bash
killall -9 roscore rosmaster rosout
pkill -9 -f ros
```

**Opción 5: Script completo de limpieza**
```bash
#!/bin/bash
echo "🛑 Deteniendo ROS..."
pkill -f roscore
sleep 1
if pgrep -f roscore > /dev/null; then
    pkill -9 -f roscore
fi
echo "✅ ROS detenido"
```

---

## 📝 Notas Importantes

### ⚠️ Advertencias de Seguridad

1. **Siempre** tener espacio libre alrededor del robot
2. **Nunca** exceder velocidades máximas recomendadas
3. **Siempre** implementar manejo de Ctrl+C
4. **Verificar** batería antes de operaciones largas
5. **Detener** robot si comportamiento errático

### ✅ Mejores Prácticas

1. **Calibrar siempre:** Cada pista es diferente
2. **Empezar lento:** Aumentar velocidad gradualmente
3. **Probar incrementalmente:** Cambios pequeños
4. **Documentar valores:** Anotar qué funciona
5. **Usar constantes:** No hardcodear valores mágicos

### 📊 Valores Recomendados

```python
# Constantes útiles
VELOCIDAD_LENTA = 0.2
VELOCIDAD_NORMAL = 0.3
VELOCIDAD_RAPIDA = 0.5

GIRO_LENTO = 0.3
GIRO_NORMAL = 0.5
GIRO_RAPIDO = 0.8

FREQ_CONTROL = 10  # Hz

# Tiempos de giro (velocidad = 0.5 rad/s)
TIEMPO_90_GRADOS = 3.14
TIEMPO_180_GRADOS = 6.28
TIEMPO_45_GRADOS = 1.57
```

---

## 🔗 Referencias Externas

- **ROS Wiki:** http://wiki.ros.org/
- **Geometry Msgs:** http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html
- **Sphero SDK:** https://github.com/sphero-inc/sphero-sdk-raspberrypi-python
- **Python Math:** https://docs.python.org/3/library/math.html

---

**Última actualización:** Octubre 2025  
**Versión:** 2.0  
**Mantenido por:** Equipo de Robótica Móvil

