# 📚 Taller Práctico - Programación del Sphero RVR

## 🎯 Objetivo del Taller

Aprender a programar el robot Sphero RVR de forma **práctica y progresiva**, enfocándose en:
- ✅ **Movimiento:** Traslación y rotación
- ✅ **Sensores:** Incorporar el sensor de color
- ✅ **Experimentación:** Modificar y jugar con el código
- 🎯 **Meta Final:** Prepararse para el seguidor de línea (semana del 27)

---

## 📋 Estructura del Taller

### 📅 **Sesión 1:** Conexión y Primer Movimiento (1-2 horas)
- Conectarse al robot
- Ejecutar primer script (traslación)
- Entender y modificar el código

### 📅 **Sesión 2:** Rotación y Experimentación (1-2 horas)
- Entender rotación del robot
- Crear formas geométricas
- Experimentar con velocidades

### 📅 **Sesión 3:** Sensor de Color (1-2 horas)
- Leer colores
- Reaccionar a diferentes colores
- Preparar para seguidor de línea

### 🎯 **Proyecto Final:** Seguidor de Línea (Semana del 27)
- Integrar movimiento + sensor
- Pruebas en pista

---

## ✋ Metodología

**Para el Instructor:**
- 🛑 **Checkpoints** en cada sección: Verificar que TODOS los grupos completen antes de continuar
- 👀 **Observar** que los estudiantes entienden, no solo copian
- 💬 **Preguntar:** "¿Qué hace esta línea?" antes de avanzar

**Para el Estudiante:**
- 📝 **No copiar y pegar** - Escribir el código manualmente
- 🤔 **Entender** cada línea antes de continuar
- 🧪 **Experimentar** con los valores
- 🙋 **Preguntar** cuando algo no esté claro

---

## 📅 SESIÓN 1: Conexión y Primer Movimiento

### ✋ Checkpoint 1: Conexión (15 min)

> **Instructor:** Verificar que TODOS los grupos completen estos pasos antes de continuar.

**Cada grupo debe:**

1. **Conectarse a WiFi:**
   - Red: `Atriz-server`
   - Contraseña: `Atriz@2r`

2. **Abrir Visual Studio Code y conectar al robot:**
   - Ver [`00_LEEME_PRIMERO.md`](00_LEEME_PRIMERO.md) para instrucciones detalladas

3. **Verificar conexión:**
   ```bash
   # En terminal de VS Code
   pwd
   # Debe mostrar: /home/sphero/atriz_git/src/ros_sphero_rvr
   ```

**✅ Todos conectados → Continuar**

---

### ✋ Checkpoint 2: Iniciar ROS (10 min)

**Terminal 1 - Iniciar roscore:**
```bash
roscore
```
> Dejar esta terminal corriendo

**Terminal 2 - Iniciar driver del robot:**
```bash
cd ~/atriz_git/src/ros_sphero_rvr/atriz_rvr_driver/scripts
python3 Atriz_rvr_node.py
```
> Dejar esta terminal corriendo. Esperar ~30 segundos hasta ver mensajes del robot.

**Terminal 3 - Verificar:**
```bash
cd ~/atriz_git/src/ros_sphero_rvr/scripts/estudiantes
rostopic list
```

**Debe aparecer:**
- `/cmd_vel`
- `/color`
- `/battery_state`

**✅ Todos ven los tópicos → Continuar**

---

### ✋ Checkpoint 3: Primer Script - Movimiento en Traslación (30 min)

> **📝 SCRIPT 1: `01_avanzar.py`** - Hacer que el robot avance

**Paso 1: Ejecutar el script (5 min)**

```bash
python3 01_avanzar.py
```

**¿Qué debe pasar?**
- El robot avanza durante 3 segundos
- Se detiene automáticamente
- El programa termina

**✅ Instructor:** Verificar que todos los robots se movieron

---

**Paso 2: Abrir y entender el código (10 min)**

Abrir el archivo `01_avanzar.py` en VS Code y analizar:

```python
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

# ¿Qué hacen estas líneas?
# - rospy: Biblioteca de ROS para Python
# - Twist: Tipo de mensaje para velocidad (lineal + angular)
```

**💬 Instructor pregunta:** "¿Qué significa `import`?"
- **Respuesta esperada:** "Traer herramientas/bibliotecas para usar en el código"

```python
rospy.init_node('avanzar_simple')
```
**💬 Pregunta:** "¿Para qué sirve esto?"
- **Respuesta:** "Crear un nodo de ROS llamado 'avanzar_simple'"

```python
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
```
**💬 Pregunta:** "¿Qué es un Publisher?"
- **Respuesta:** "Un publicador que envía mensajes al robot"
- **💬 ¿A dónde envía?** "/cmd_vel (comando de velocidad)"
- **💬 ¿Qué tipo de mensaje?** "Twist (velocidad)"

```python
cmd = Twist()
cmd.linear.x = 0.3  # Velocidad hacia adelante (m/s)
```
**💬 Pregunta:** "¿Qué significa 0.3?"
- **Respuesta:** "0.3 metros por segundo hacia adelante"

```python
for _ in range(30):  # 30 iteraciones
    pub.publish(cmd)  # Enviar comando
    rospy.sleep(0.1)  # Esperar 0.1 segundos
```
**💬 Pregunta:** "¿Cuánto tiempo total avanza?"
- **Respuesta:** "30 × 0.1 = 3 segundos"

```python
cmd.linear.x = 0.0  # Detener
pub.publish(cmd)
```
**💬 Pregunta:** "¿Por qué es importante esta línea?"
- **Respuesta:** "Para que el robot se detenga"

**✅ Todos entienden el código → Continuar**

---

**Paso 3: Experimentar y Modificar (15 min)**

**🧪 Experimento 1: Cambiar la velocidad**

Modificar la línea 15:
```python
# Original:
cmd.linear.x = 0.3

# Cambiar a:
cmd.linear.x = 0.5  # Más rápido
```

**Ejecutar:** `python3 01_avanzar.py`

**💬 Pregunta:** "¿Qué cambió?"
- **Respuesta esperada:** "El robot va más rápido"

---

**🧪 Experimento 2: Cambiar el tiempo**

Modificar la línea 18:
```python
# Original:
for _ in range(30):

# Cambiar a:
for _ in range(50):  # Más tiempo
```

**💬 Pregunta:** "¿Cuánto tiempo avanzará ahora?"
- **Respuesta:** "50 × 0.1 = 5 segundos"

**Ejecutar y verificar**

---

**🧪 Experimento 3: Crear tu propia combinación**

**Desafío:** Hacer que el robot avance:
- A 0.2 m/s
- Durante 4 segundos

**💬 Preguntas guía:**
- "¿Qué velocidad usar?" → 0.2
- "¿Cuántas iteraciones?" → 40 (porque 40 × 0.1 = 4 segundos)

**Solución:**
```python
cmd.linear.x = 0.2
for _ in range(40):
    pub.publish(cmd)
    rospy.sleep(0.1)
```

**✅ Checkpoint:** Todos completaron experimentos → Continuar

---

## 📅 SESIÓN 2: Rotación y Experimentación

### ✋ Checkpoint 4: Segundo Script - Rotación (30 min)

> **📝 SCRIPT 2: `02_girar.py`** - Hacer que el robot gire

**Paso 1: Ejecutar el script (5 min)**

```bash
python3 02_girar.py
```

**¿Qué debe pasar?**
- El robot gira en su lugar durante 3 segundos
- Se detiene
- El programa termina

**✅ Instructor:** Verificar que todos los robots giraron

---

**Paso 2: Entender el código (10 min)**

Abrir `02_girar.py`:

```python
cmd = Twist()
cmd.angular.z = 0.5  # Velocidad angular (rad/s)
```

**💬 Diferencia con el script anterior:**
- **01_avanzar.py:** Usaba `cmd.linear.x` (traslación)
- **02_girar.py:** Usa `cmd.angular.z` (rotación)

**💬 Pregunta:** "¿Qué es `angular.z`?"
- **Respuesta:** "Velocidad de rotación alrededor del eje Z (vertical)"
- **💬 ¿Qué significa 0.5?** "0.5 radianes por segundo"

**💬 Pregunta:** "¿Valores positivos y negativos?"
- **Positivo (0.5):** Gira a la izquierda (contra reloj)
- **Negativo (-0.5):** Gira a la derecha (sentido reloj)

---

**Paso 3: Experimentar con rotación (15 min)**

**🧪 Experimento 1: Cambiar dirección**

```python
# Girar a la derecha:
cmd.angular.z = -0.5
```

**Ejecutar y observar**

---

**🧪 Experimento 2: Girar más rápido**

```python
cmd.angular.z = 1.0  # El doble de rápido
```

**💬 Pregunta:** "¿Qué pasa?"
- **Respuesta:** "Gira más rápido"

---

**🧪 Experimento 3: Combinar traslación y rotación**

**Crear nuevo archivo:** `mi_experimento.py`

```python
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

rospy.init_node('experimento')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rospy.sleep(1)

cmd = Twist()
cmd.linear.x = 0.3   # Avanzar
cmd.angular.z = 0.5  # Y girar al mismo tiempo

for _ in range(30):
    pub.publish(cmd)
    rospy.sleep(0.1)

# Detener
cmd.linear.x = 0.0
cmd.angular.z = 0.0
pub.publish(cmd)
```

**💬 Pregunta:** "¿Qué hará el robot?"
- **Respuesta:** "Avanzar mientras gira (círculo/curva)"

**✅ Checkpoint:** Todos experimentaron con rotación → Continuar

---

### ✋ Checkpoint 5: Crear una Figura (20 min)

**🎯 Desafío:** Hacer que el robot dibuje un cuadrado

**Pistas:**
1. Avanzar → Girar 90° → Repetir 4 veces
2. Para girar 90° (π/2 radianes ≈ 1.57 rad):
   - A velocidad 0.5 rad/s
   - Necesitas: 1.57 / 0.5 = 3.14 segundos

**Solución paso a paso:**

```python
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

rospy.init_node('cuadrado')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rospy.sleep(1)

# Repetir 4 veces (4 lados)
for lado in range(4):
    # Avanzar
    cmd = Twist()
    cmd.linear.x = 0.3
    for _ in range(20):  # 2 segundos
        pub.publish(cmd)
        rospy.sleep(0.1)
    
    # Girar 90 grados
    cmd.linear.x = 0.0
    cmd.angular.z = 0.5
    for _ in range(31):  # 3.1 segundos
        pub.publish(cmd)
        rospy.sleep(0.1)

# Detener
cmd = Twist()
pub.publish(cmd)
```

**💬 Preguntas durante el desarrollo:**
- "¿Por qué 4 iteraciones del for?" → 4 lados del cuadrado
- "¿Por qué 31 en el giro?" → 31 × 0.1 = 3.1 segundos ≈ 90°

**✅ Checkpoint:** Todos crearon un cuadrado → Continuar

---

## 📅 SESIÓN 3: Sensor de Color

### ✋ Checkpoint 6: Leer el Sensor de Color (30 min)

> **📝 SCRIPT 3: `05_sensor_color.py`** - Leer colores

**Paso 1: Ejecutar el script (5 min)**

```bash
python3 05_sensor_color.py
```

**¿Qué debe pasar?**
- El robot muestra en pantalla los colores que detecta
- Los valores RGB aparecen constantemente
- Ctrl+C para detener

**✅ Instructor:** Todos ven valores de color

---

**Paso 2: Entender el código del sensor (15 min)**

Abrir `05_sensor_color.py`:

```python
from atriz_rvr_msgs.msg import Color
```
**💬 Pregunta:** "¿Qué es diferente?"
- **Respuesta:** "Importamos un mensaje de color (no Twist)"

```python
def callback_color(msg):
    r = msg.rgb_color[0]  # Rojo
    g = msg.rgb_color[1]  # Verde
    b = msg.rgb_color[2]  # Azul
    print(f"Color: R={r}, G={g}, B={b}")
```
**💬 Pregunta:** "¿Qué es un callback?"
- **Respuesta:** "Una función que se ejecuta cuando llega un mensaje nuevo"

```python
rospy.Subscriber('/color', Color, callback_color)
```
**💬 Pregunta:** "¿Diferencia entre Publisher y Subscriber?"
- **Publisher:** Envía mensajes (comandos al robot)
- **Subscriber:** Recibe mensajes (datos del sensor)

---

**Paso 3: Experimentar con colores (10 min)**

**🧪 Experimento 1: Detectar colores**

Poner diferentes objetos frente al sensor:
- Papel rojo
- Papel azul
- Papel verde
- Papel negro

**💬 Observar:** "¿Qué valores cambian?"

---

**🧪 Experimento 2: Reaccionar a un color**

**Desafío:** Hacer que el robot avance solo cuando ve verde

```python
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from atriz_rvr_msgs.msg import Color

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

def callback_color(msg):
    g = msg.rgb_color[1]  # Verde
    
    cmd = Twist()
    if g > 150:  # Si detecta mucho verde
        cmd.linear.x = 0.3  # Avanzar
        print("¡Verde detectado! Avanzando...")
    else:
        cmd.linear.x = 0.0  # Detenerse
    
    pub.publish(cmd)

rospy.init_node('seguir_verde')
rospy.Subscriber('/color', Color, callback_color)
rospy.spin()
```

**💬 Preguntas:**
- "¿Qué hace `if g > 150`?" → Verifica si hay mucho verde
- "¿Qué hace `rospy.spin()`?" → Mantiene el programa corriendo

**✅ Checkpoint:** Todos hicieron reaccionar el robot a un color

---

## 🎯 Preparación para Seguidor de Línea (Semana del 27)

### Concepto del Seguidor de Línea

**Idea básica:**
1. **Sensor detecta** línea negra/blanca
2. **Robot decide:**
   - Si ve línea → Avanzar
   - Si no ve línea → Girar para buscarla

**Pseudocódigo:**
```
SI color es NEGRO:
    Avanzar
SI NO:
    Girar para buscar la línea
```

**Código ejemplo básico:**

```python
def callback_color(msg):
    r, g, b = msg.rgb_color
    
    cmd = Twist()
    
    # Detectar negro (valores bajos en RGB)
    if r < 50 and g < 50 and b < 50:
        # Línea negra detectada - Avanzar
        cmd.linear.x = 0.2
        cmd.angular.z = 0.0
    else:
        # No hay línea - Buscar girando
        cmd.linear.x = 0.0
        cmd.angular.z = 0.3
    
    pub.publish(cmd)
```

### Tarea para la Semana del 27

**Cada grupo debe:**
1. ✅ Tener el código del seguidor funcionando
2. ✅ Haber probado con líneas en el suelo
3. ✅ Ajustar los valores según su robot
4. ✅ Estar listos para la pista oficial

---

## 📝 Resumen de lo Aprendido

### ✅ Conceptos Clave

**Movimiento:**
- `cmd.linear.x` → Traslación (avanzar/retroceder)
- `cmd.angular.z` → Rotación (girar)
- `Publisher` → Enviar comandos al robot

**Sensores:**
- `Subscriber` → Recibir datos del sensor
- `callback` → Función que se ejecuta con cada dato
- Valores RGB → Detectar colores

**Integración:**
- Combinar sensor + movimiento = Comportamiento reactivo
- Lógica condicional (`if/else`) para decisiones

### 🎯 Siguiente Nivel

**Para profundizar:**
- Ver scripts `10_movimiento_completo.py` y `11_sensor_avanzado.py`
- Experimentar con más formas geométricas
- Crear comportamientos más complejos

---

**🤖 ¡Felicitaciones! Ya sabes programar el Sphero RVR y estás listo para el seguidor de línea! 🎉**
