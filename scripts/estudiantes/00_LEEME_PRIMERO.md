# 🚀 ¡LÉEME PRIMERO!

## Bienvenido a la Carpeta de Estudiantes

Esta carpeta contiene **todo lo necesario** para aprender a programar el robot Sphero RVR.

---

## ⚡ Inicio Ultra-Rápido

### 0️⃣ Conectar a la Red WiFi del Robot
```
SSID: Atriz-server
Contraseña: Atriz@2r
```
**⚠️ PRIMER PASO OBLIGATORIO** - Sin WiFi no hay comunicación con el robot.

### 1️⃣ Conectar al Robot con Visual Studio Code

**Método Recomendado:**
1. Abrir VS Code
2. Click en botón esquina inferior izquierda `><`
3. Seleccionar: `Connect to Host...`
4. Escribir: `sphero@[IP_del_robot]`
5. Sistema Operativo: `Linux`
6. Contraseña: `admin2024`

**Alternativa (Terminal):**
```bash
ssh sphero@[IP_del_robot]`
# Contraseña: admin2024
```

### 2️⃣ Abrir Carpeta del Proyecto (en VS Code)
```
File → Open Folder...
Ruta: /home/sphero/atriz_git/src/ros_sphero_rvr
```

### 3️⃣ Iniciar ROS (Terminal 1)
```bash
roscore
```

### 4️⃣ Iniciar Driver del Robot (Terminal 2)
```bash
cd ~/atriz_git/src/ros_sphero_rvr/atriz_rvr_driver/scripts
python3 Atriz_rvr_node.py
```

### 5️⃣ Ejecutar Primer Script (Terminal 3)
```bash
cd ~/atriz_git/src/ros_sphero_rvr/scripts/estudiantes
python3 01_avanzar.py
```

**Alternativa (inicio automático):**
```bash
# En lugar de pasos 3 y 4, puedes usar:
./start_ros.sh
```

🎉 **¡Listo! Tu robot ya se movió.**

---

## 📚 ¿Qué Hay en Esta Carpeta?

### 🎯 10 Scripts Organizados por Nivel

```
NIVEL 1: Básico (⭐-⭐⭐)
├── 01_avanzar.py         Solo avanzar
├── 02_girar.py           Solo girar  
├── 03_cuadrado.py        Dibujar cuadrado
├── 04_giro_preciso.py    Giros exactos
└── 05_sensor_color.py    Leer colores

NIVEL 2: Intermedio (⭐⭐⭐)
├── 10_movimiento_completo.py   Control total
└── 11_sensor_avanzado.py       Calibración + modos

NIVEL 3: Proyecto Final (⭐⭐⭐⭐)
└── seguidor_linea_profesional.py   🏁 Seguidor de línea con PID

NIVEL 4: Herramientas
├── 90_template.py        Crear tus scripts
└── 99_test_ctrl_c.py     Probar Ctrl+C
```

### 📖 4 Documentos Esenciales

| Archivo | Contenido |
|---------|-----------|
| **00_LEEME_PRIMERO.md** | 🚀 Este archivo - Todo lo que necesitas |
| **GUIA_PASO_A_PASO.md** | 📚 Tutorial detallado de cada script |
| **REFERENCIAS.md** | 📐 Fórmulas, tablas y datos técnicos |
| **SEGUIDOR_LINEA_DOCS.md** | 🏁 Documentación completa del seguidor profesional |

---

## 🎓 Ruta de Aprendizaje

### Día 1: Fundamentos (2 horas)
```bash
# Paso 0: Conectar a WiFi Atriz-server (contraseña: Atriz@2r)
# Paso 1: Conectar por SSH y ejecutar scripts

python3 01_avanzar.py      # 5 min
python3 02_girar.py        # 5 min
python3 03_cuadrado.py     # 10 min
# Experimentos: 1 hora
```

### Día 2: Precisión (2 horas)
```bash
python3 04_giro_preciso.py # 15 min
python3 05_sensor_color.py # 15 min
# Calibración y práctica: 1.5 horas
```

### Día 3: Proyecto Propio (3 horas)
```bash
cp 90_template.py mi_robot.py
nano mi_robot.py
python3 mi_robot.py
```

### Día 4: Avanzado (3 horas)
```bash
python3 10_movimiento_completo.py
python3 11_sensor_avanzado.py
```

### Día 5: Seguidor de Línea Profesional (4 horas)
```bash
# Leer documentación completa
cat SEGUIDOR_LINEA_DOCS.md

# Ejecutar con calibración automática
python3 seguidor_linea_profesional.py

# Ajustar parámetros
nano seguidor_config.json

# Optimizar para competencia
python3 seguidor_linea_profesional.py
```

**📚 Documentación completa:** `SEGUIDOR_LINEA_DOCS.md`

---

## 🔍 ¿Necesitas Ayuda Con...?

### Conectarse al Robot
→ Ver inicio de este archivo (WiFi y credenciales)  
→ Sección "Guía Completa de Visual Studio Code" (abajo)

### Movimiento
→ Scripts 01, 02, 03, 04  
→ `REFERENCIAS.md` (Cálculos de Giro)

### Sensores
→ Scripts 05, 11  
→ `GUIA_PASO_A_PASO.md` (Script 05)

### Crear Tu Propio Script
→ `90_template.py`  
→ `GUIA_PASO_A_PASO.md` (Nivel 3)

### Problemas Técnicos
→ Sección "Solución de Problemas" (abajo)  
→ `REFERENCIAS.md` (FAQ y permisos)

---

## ⚠️ Importante: Sistema de Numeración

Los scripts están numerados para seguirse **EN ORDEN**:

✅ **Correcto:**
```bash
01 → 02 → 03 → 04 → 05 → 10 → 11
```

❌ **Incorrecto:**
```bash
# NO saltar niveles
01 → 10 (muy difícil)
```

**Regla:** Domina un nivel antes de pasar al siguiente.

---

## 🎯 ¿Qué Aprenderás?

### Nivel 1 (Scripts 01-05):
✅ Publicar comandos de velocidad  
✅ Diferencia lineal vs angular  
✅ Combinar movimientos  
✅ Calcular tiempos y ángulos  
✅ Leer sensores  

### Nivel 2 (Scripts 10-11):
✅ Programación con clases  
✅ Funciones reutilizables  
✅ Múltiples modos de operación  
✅ Calibración de sensores  
✅ Máquinas de estado  

### Nivel 3 (Scripts 90-99):
✅ Crear scripts desde cero  
✅ Debugging  
✅ Mejores prácticas ROS  
✅ Documentar código  

---

## 💡 Consejos Rápidos

### ✅ Hacer:
- Leer los comentarios en el código
- Experimentar cambiando valores
- Anotar qué funciona
- Pedir ayuda cuando necesites
- Usar Ctrl+C para detener

### ❌ Evitar:
- Saltar niveles
- Copiar sin entender
- Modificar sin leer
- Ejecutar sin espacio libre
- Ignorar errores

---

## 🔧 Comandos Más Usados

```bash
# Ejecutar script
python3 01_avanzar.py

# Ver tópicos activos
rostopic list

# Ver velocidades en vivo
rostopic echo /cmd_vel

# Copiar y modificar
cp 01_avanzar.py mi_script.py
nano mi_script.py

# Detener robot (emergencia)
rostopic pub -1 /cmd_vel geometry_msgs/Twist '{}'
```

---

## 📊 Estadísticas de la Carpeta

```
📁 Total archivos: 17
   ├── 🐍 Scripts Python: 10
   ├── 📄 Documentación: 6
   └── ⚙️  Configuración: 1 (JSON)

📏 Líneas de código: ~3,700
💾 Tamaño total: 175 KB

⏱️ Tiempo estimado curso: 16 horas
🎯 Nivel final: Seguidor de línea
```

---

## 🚦 Estados del Robot

### 🟢 Robot Listo
```bash
rostopic list  # Muestra /cmd_vel
```

### 🟡 Robot Ocupado
```bash
# Otro script ejecutándose
# Esperar o detener con Ctrl+C
```

### 🔴 Robot No Responde
```bash
# 1. Detener roscore
pkill -f roscore

# 2. Esperar 2 segundos
sleep 2

# 3. Reiniciar
./start_ros.sh
```

### 🛑 Detener roscore
```bash
# Recomendado
pkill -f roscore

# Forzado (si no responde)
pkill -9 -f roscore

# Limpiar TODO
killall -9 roscore rosmaster rosout
```

---

## 💻 Guía Completa de Visual Studio Code

### 🚀 Primera Conexión con VS Code

**Paso 1: Instalar Extensión Remote - SSH**
1. Abrir VS Code
2. Ir a Extensiones: `Ctrl + Shift + X`
3. Buscar: "Remote - SSH" (Microsoft)
4. Click en "Install"

**Paso 2: Conectar al Robot**
1. Click en botón inferior izquierdo: `><`
2. Seleccionar: "Connect to Host..."
3. Escribir: `sphero@[IP_del_robot]`
4. Si pregunta SO: Linux
5. Contraseña: `admin2024`

**Paso 3: Abrir Proyecto**
1. File → Open Folder...
2. Ruta: `/home/sphero/atriz_git/src/ros_sphero_rvr`
3. Click OK

**Paso 4: Abrir Terminal**
- Terminal → New Terminal (o `Ctrl + Shift + ñ`)
- Click en `+` para más terminales

### ⌨️ Atajos de Teclado Útiles

| Acción | Atajo |
|--------|-------|
| Abrir terminal | `Ctrl + Shift + ñ` |
| Cerrar terminal | `Ctrl + D` |
| Buscar archivo | `Ctrl + P` |
| Guardar archivo | `Ctrl + S` |
| Buscar en archivo | `Ctrl + F` |
| Abrir/Cerrar barra lateral | `Ctrl + B` |
| Paleta de comandos | `Ctrl + Shift + P` |

### 🔄 Reconectar

1. Abrir VS Code
2. Click en `><` → Ver historial
3. Seleccionar `sphero@[IP]`
4. Contraseña: `admin2024`

---

## 🆘 Solución de Problemas

### No puedo conectarme al robot

**Verificar WiFi:**
```bash
# 1. Conectado a: Atriz-server (contraseña: Atriz@2r)
# 2. Verificar conectividad
ping -c 3 raspberrypi.local

# 3. Si no responde:
# - ¿Red correcta?
# - ¿Robot encendido?
```

**Problemas con VS Code:**
- ❌ No aparece botón `><` → Instalar "Remote - SSH"
- ❌ Contraseña rechazada → Usar: `admin2024`
- ❌ No conecta → Verificar IP del robot
- ❌ Pide SO → Seleccionar: Linux

**Problemas con SSH:**
- ❌ "Permission denied" → Contraseña: `admin2024`
- ❌ Timeout → Esperar 30s y reintentar
- ❌ Hostname no resuelve → Usar IP: `ssh sphero@192.168.1.X`

### El robot no se mueve

```bash
# 1. Verificar ROS
rostopic list  # Debe aparecer /cmd_vel

# 2. Verificar batería
# LED parpadea rojo = batería baja

# 3. Reiniciar ROS
pkill -f roscore
sleep 2
roscore  # En terminal 1
python3 ./atriz_rvr_driver/scripts/Atriz_rvr_node.py  # En terminal 2
```

### Cómo detener roscore

```bash
# Método recomendado
pkill -f roscore

# Si no responde (forzado)
pkill -9 -f roscore

# Limpiar TODO
killall -9 roscore rosmaster rosout
```

### El sensor no funciona

```bash
# 1. Verificar que esté activo
rostopic echo /color

# 2. Verificar iluminación
# Necesita luz suficiente pero no directa

# 3. Recalibrar
python3 11_sensor_avanzado.py  # Opción 1: Calibración
```

### Ctrl+C no detiene el robot

```bash
# Esperar 1-2 segundos
# Si persiste:
rostopic pub -1 /cmd_vel geometry_msgs/Twist '{}'
```

---

## 🎯 Desafíos Propuestos

### 🥉 Nivel Bronce (Fácil)
- [ ] Hacer que el robot avance 5 metros
- [ ] Dibujar un triángulo
- [ ] Hacer una vuelta completa (360°)
- [ ] Calibrar 3 colores diferentes

### 🥈 Nivel Plata (Medio)
- [ ] Dibujar un hexágono perfecto
- [ ] Crear secuencia que dibuje tu inicial
- [ ] Robot reacciona diferente a cada color
- [ ] Implementar patrón de movimiento aleatorio

### 🥇 Nivel Oro (Difícil)
- [ ] Seguidor de línea básico
- [ ] Robot que evita objetos usando colores
- [ ] Crear laberinto y que el robot lo resuelva
- [ ] Implementar máquina de estados compleja

---

## 📞 Soporte

### Durante Clase:
🙋 Levantar mano para checkpoint  
💬 Compartir con otros grupos  
📝 Documentar problemas

### Documentación:
📚 GUIA_PASO_A_PASO.md - Tutorial detallado  
📐 REFERENCIAS.md - Datos técnicos y fórmulas  
⚡ Todo lo demás está en este archivo  

---

## 🏁 Siguiente Paso

### Si Eres Nuevo:
1. ✅ **Conectar a WiFi:** `Atriz-server` (contraseña: `Atriz@2r`)  
2. 💻 **Conectar VS Code:** Ver sección "Guía Completa de Visual Studio Code"
3. 🚀 **Iniciar ROS:** Terminal 1: `roscore`, Terminal 2: driver
4. 🤖 **Ejecutar:** `python3 01_avanzar.py`  
5. 📚 **Profundizar:** `GUIA_PASO_A_PASO.md`  

### Si Ya Conoces lo Básico:
👉 **Verificar WiFi:** Conectado a `Atriz-server`  
👉 **Continuar:** Script donde te quedaste  
👉 **Consultar:** `REFERENCIAS.md` para fórmulas  
👉 **Desafíos:** Ver sección "Desafíos Propuestos"

---

## 🎉 ¡Empecemos!

1. **Conecta** al robot (WiFi + VS Code) - 10 min
2. **Ejecuta** `01_avanzar.py` - 2 min
3. **Experimenta** modificando valores - 30 min
4. **Aprende** con `GUIA_PASO_A_PASO.md` - 1 hora
5. **Avanza** al siguiente nivel

---

**🤖 ¡Bienvenido al mundo de la robótica móvil!**

---

## 📋 Resumen de Archivos

Esta carpeta contiene:
- **9 scripts Python** organizados por nivel (01-05, 10-11, 90, 99)
- **3 documentos esenciales:**
  - `00_LEEME_PRIMERO.md` - Este archivo (guía completa)
  - `GUIA_PASO_A_PASO.md` - Tutorial detallado de cada script
  - `REFERENCIAS.md` - Fórmulas, cálculos y datos técnicos

*Última actualización: Octubre 2025*  
*Versión: 3.0 - Documentación consolidada*  
*Sistema ultra-simplificado para mejor aprendizaje*

