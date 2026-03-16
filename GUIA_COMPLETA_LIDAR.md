# 🎯 Guía Completa de Configuración del LIDAR - YDLidar

## 📋 Tabla de Contenidos
1. [Estado Actual](#estado-actual)
2. [Especificaciones Técnicas](#especificaciones-técnicas)
3. [Instalación Completada](#instalación-completada)
4. [Uso del LIDAR](#uso-del-lidar)
5. [Solución de Problemas](#solución-de-problemas)
6. [Configuración Avanzada](#configuración-avanzada)
7. [Integración con Sphero RVR](#integración-con-sphero-rvr)

---

## ✅ Estado Actual

### LIDAR Completamente Configurado y Funcionando

```
Modelo:       YDLIDAR X2
Puerto:       /dev/ttyUSB0 o /dev/ydlidar (si configuraste la regla udev)
Chip:         Silicon Labs CP2102 USB to UART Bridge
Baudrate:     115200 bps
Tasa datos:   ~7024 bytes/segundo
Protocolo:    AA 55 (YDLidar)
Sample Rate:  3K (3000 muestras/seg)
Frecuencia:   10 Hz
Tipo:         Triangle (Single Channel)
Driver:       ydlidar_ros_driver ✅ INSTALADO
SDK:          YDLidar-SDK ✅ INSTALADO
Workspace:    ✅ COMPILADO
Estado:       ✅ FUNCIONANDO CORRECTAMENTE
```

### Checklist Completo
- [x] LIDAR conectado y reconocido
- [x] Puerto identificado y configurado
- [x] Baudrate confirmado (115200)
- [x] Protocolo identificado (YDLidar)
- [x] Driver ROS instalado
- [x] SDK compilado e instalado
- [x] Workspace ROS compilado
- [x] Listo para ejecutar

---

## 🔬 Especificaciones Técnicas

### Hardware Detectado
- **Interfaz**: USB a Serial (CP2102)
- **Conexión**: /dev/ttyUSB0
- **Voltaje**: Verificar especificaciones del modelo (típicamente 5V)
- **Comunicación**: UART a 115200 bps

### Protocolo de Comunicación
- **Firma de inicio**: `AA 55` (distintivo de YDLidar)
- **Baudrates compatibles**: 115200, 128000, 230400, 256000, 460800, 921600
- **Baudrate óptimo**: 115200 (verificado con test)

### YDLIDAR X2 - Especificaciones
**Tu modelo detectado**: YDLIDAR X2

- **Alcance**: 8-12m (0.1-12m configurado)
- **Frecuencia de escaneo**: 10 Hz
- **Sample Rate**: 3000 muestras/segundo
- **Resolución angular**: 360° (single channel)
- **Tipo**: Triangle (triangulación)
- **Interfaz**: UART (Serial)
- **Baudrate**: 115200 bps
- **Alimentación**: 5V DC

### Otros Modelos YDLidar
- **X2L**: Similar al X2, versión mejorada
- **X4**: 360°, 10m alcance, 5000 Hz
- **G2/G4**: 360°, 12m/16m alcance, 9000 Hz
- **TG**: Serie triangulación, mayor precisión

---

## 🚀 Instalación Completada

Sigue estos pasos en orden para conectar el YDLIDAR X2 y preparar el sistema.

### Paso 1: Conexión Física y Permisos de Puerto (Udev)

El YDLIDAR se conecta mediante un adaptador USB. Cuando lo conectes a la Raspberry Pi, Linux le asignará un puerto (usualmente `/dev/ttyUSB0`). Para evitar que este puerto cambie si conectas otros dispositivos, conviene fijar un **nombre estático** con una regla udev.

1. **Conecta el YDLIDAR X2** a un puerto USB de la Raspberry Pi.

2. **Comprueba que fue detectado** en una terminal:
   ```bash
   ls -l /dev/ttyUSB*
   ```
   Deberías ver algo como `ttyUSB0` (o `ttyUSB1`, etc.).

3. **Crea una regla udev** para nombre fijo y permisos permanentes:
   ```bash
   sudo nano /etc/udev/rules.d/ydlidar.rules
   ```
   Pega esta línea (crea el enlace simbólico `ydlidar` y da permisos de lectura/escritura):
   ```
   KERNEL=="ttyUSB*", MODE="0666", SYMLINK+="ydlidar"
   ```
   Guarda (`Ctrl+O`, `Enter`) y cierra (`Ctrl+X`).

4. **Recarga las reglas udev**:
   ```bash
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```

A partir de ahora el LiDAR estará disponible de forma estable en **`/dev/ydlidar`** (y seguirá existiendo en `/dev/ttyUSB0` si es el único dispositivo USB-serial).

---

### Paso 2: Instalar el SDK de YDLIDAR (C++)

El driver de ROS para YDLIDAR necesita el **SDK base** instalado en el sistema.

1. **Instala las herramientas de compilación**:
   ```bash
   sudo apt update
   sudo apt install cmake pkg-config build-essential
   ```

2. **Clona, compila e instala el SDK**:
   ```bash
   cd ~
   git clone https://github.com/YDLIDAR/YDLidar-SDK.git
   cd YDLidar-SDK
   mkdir build && cd build
   cmake ..
   make
   sudo make install
   ```
   **Importante**: Sin `sudo make install` y (si aplica) `sudo ldconfig`, el paquete ROS no encontrará el SDK y la compilación fallará.

Después de este paso puedes instalar el **driver ROS** (ydlidar_ros_driver) y compilar tu workspace; el script de instalación automática incluye ambos.

---

### Proceso Realizado (detalle)

A continuación se detalla el flujo completo incluyendo verificación y driver ROS.

#### 1. Detección Inicial ✅
```bash
cd /home/sphero/atriz_git/src/Atriz_rvr/scripts/lydar
python3 test_lidar.py
```
**Resultado**: 
- Puerto `/dev/ttyUSB0` detectado
- Comunicación estable a 115200 bps
- Protocolo `AA 55` identificado (YDLidar)

#### 2. Instalación de Dependencias ✅
```bash
sudo apt-get update
sudo apt-get install -y cmake pkg-config python3-pip
```

#### 3. Clonación de Repositorios ✅
```bash
cd /home/sphero/atriz_git/src
git clone https://github.com/YDLIDAR/ydlidar_ros_driver.git
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
```

#### 4. Compilación del SDK ✅
```bash
cd YDLidar-SDK
mkdir -p build && cd build
cmake ..
make
```
**Nota**: Los warnings durante `make` son normales y no afectan el funcionamiento.

#### 5. Instalación del SDK ✅ (PASO CRÍTICO)
```bash
sudo make install
sudo ldconfig
```
**⚠️ IMPORTANTE**: Sin este paso, catkin_make fallará con "ydlidar_sdk not found"

#### 6. Compilación del Workspace ✅
```bash
cd /home/sphero/atriz_git
catkin_make
source devel/setup.bash
```

### Script de Instalación Automática

Todo el proceso se puede ejecutar automáticamente con:

```bash
cd /home/sphero/atriz_git/src/Atriz_rvr/scripts/lydar
./install_lidar_driver.sh
# Seleccionar opción 2 (YDLidar)
```

El script incluye:
- ✅ Detección automática del workspace
- ✅ Instalación de dependencias
- ✅ Clonación de repositorios
- ✅ Compilación del SDK
- ✅ Instalación del SDK con sudo
- ✅ Configuración de permisos

---

## ⚠️ Nota Importante sobre Warnings Durante la Compilación

Durante la compilación del SDK verás muchos **WARNINGS** (advertencias), **NO ERRORES**:

- 🟡 `warning: format not a string literal` = Solo advertencia de estilo
- 🟡 `warning: ISO C++ forbids converting a string constant` = Solo advertencia de estilo  
- ✅ `[100%] Built target` = **Compilación exitosa**

**El SDK se compila perfectamente.** Los warnings son del código fuente de YDLidar y **no afectan el funcionamiento**.

### 🔑 Punto Crítico: Instalación del SDK

**Problema común**: Error "ydlidar_sdk not found" durante catkin_make

**Causa**: El SDK se compiló pero **no se instaló** en el sistema

**Solución**: 
```bash
cd /home/sphero/atriz_git/src/YDLidar-SDK/build
sudo make install
sudo ldconfig
```

Este paso es **OBLIGATORIO** cuando se instala YDLidar manualmente. El script automático `install_lidar_driver.sh` ya lo incluye.

---

## 🎮 Uso del LIDAR

### Inicio Rápido

#### 1. Verificar Permisos del Puerto
```bash
ls -la /dev/ttyUSB0
# Si no tienes permisos:
sudo chmod 666 /dev/ttyUSB0
```

#### 2. Ejecutar el Driver ROS - YDLIDAR X2

**Opción A: Con integración Sphero RVR (RECOMENDADO)**
```bash
cd /home/sphero/atriz_git
source devel/setup.bash
roslaunch atriz_rvr_driver lidar_only.launch
```

**Opción B: Driver directo - Launch personalizado**
```bash
cd /home/sphero/atriz_git
source devel/setup.bash
roslaunch ydlidar_ros_driver x2_custom.launch
```

**Opción C: Launch file original del X2**
```bash
# Nota: Requiere modificar el puerto en el archivo
roslaunch ydlidar_ros_driver X2.launch
```

**Configuración del X2 que funciona**:
- ✅ Puerto: `/dev/ttyUSB0`
- ✅ Baudrate: `115200`
- ✅ Sample Rate: `3` (3K)
- ✅ Single Channel: `true`
- ✅ Reversion: `false`
- ✅ Support Motor DTR: `true`

#### 3. Verificar que Funciona
```bash
# En otra terminal - Ver topics
rostopic list
# Deberías ver: /scan

# Ver datos en tiempo real
rostopic echo /scan

# Verificar frecuencia de publicación
rostopic hz /scan
# Esperado: 5-10 Hz
```

### Visualización en RViz

#### Método 1: RViz Básico
```bash
# Terminal 1: Driver del LIDAR
roslaunch ydlidar_ros_driver ydlidar.launch serial_port:=/dev/ttyUSB0 serial_baudrate:=115200

# Terminal 2: RViz
rviz
```

**Configuración en RViz**:
1. Click en **"Add"** (abajo izquierda)
2. Selecciona **"LaserScan"**
3. En el panel LaserScan:
   - Topic: `/scan`
   - Size: 0.05
   - Color: 255;0;0 (rojo)
4. En Global Options:
   - Fixed Frame: `laser` o `base_link`

#### Método 2: Launch File con RViz
Crear archivo `view_lidar.launch`:
```xml
<launch>
  <!-- LIDAR Driver -->
  <node name="ydlidar_node" pkg="ydlidar_ros_driver" type="ydlidar_ros_driver_node" output="screen">
    <param name="port" value="/dev/ttyUSB0"/>
    <param name="baudrate" value="115200"/>
    <param name="frame_id" value="laser"/>
    <param name="angle_min" value="-3.14159"/>
    <param name="angle_max" value="3.14159"/>
    <param name="range_min" value="0.1"/>
    <param name="range_max" value="12.0"/>
  </node>
  
  <!-- RViz -->
  <node name="rviz" pkg="rviz" type="rviz" args="-d $(find ydlidar_ros_driver)/launch/ydlidar.rviz" required="true"/>
</launch>
```

---

## 🔧 Solución de Problemas

### Problemas Comunes y Soluciones

#### 1. Error: "Permission denied" en /dev/ttyUSB0

**Síntoma**: No puede abrir el puerto serial

**Solución Temporal**:
```bash
sudo chmod 666 /dev/ttyUSB0
```

**Solución Permanente**:
```bash
# Crear regla udev
sudo nano /etc/udev/rules.d/99-lidar.rules

# Añadir:
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", SYMLINK+="lidar"

# Recargar reglas
sudo udevadm control --reload-rules
sudo udevadm trigger
```

#### 2. Error: "ydlidar_sdk not found" durante catkin_make

**Síntoma**: CMake no encuentra el SDK de YDLidar

**Causa**: El SDK no fue instalado con `sudo make install`

**Solución**:
```bash
cd /home/sphero/atriz_git/src/YDLidar-SDK/build
sudo make install
sudo ldconfig

# Luego recompilar workspace
cd /home/sphero/atriz_git
rm -rf build/ devel/
catkin_make
```

#### 3. LIDAR no gira (motor no funciona)

**Posibles causas**:
- Alimentación insuficiente
- Motor desconectado
- PWM no configurado

**Soluciones**:
1. Verificar alimentación (5V o 12V según modelo)
2. Verificar conexión del motor
3. Revisar parámetros del launch file

#### 4. No se publican datos en /scan

**Verificaciones**:
```bash
# Ver si el nodo está corriendo
rosnode list

# Ver info del nodo
rosnode info /ydlidar_node

# Ver topics publicados
rostopic list

# Verificar errores
rosnode info /ydlidar_node | grep ERROR
```

**Solución**:
```bash
# Reiniciar el driver
rosnode kill /ydlidar_node
roslaunch ydlidar_ros_driver ydlidar.launch serial_port:=/dev/ttyUSB0 serial_baudrate:=115200
```

#### 5. Puerto no encontrado después de reiniciar

**Causa**: El puerto USB cambió (de ttyUSB0 a ttyUSB1)

**Solución**:
```bash
# Ver puertos disponibles
ls -la /dev/ttyUSB*

# O usar dmesg
dmesg | grep ttyUSB

# Actualizar el puerto en el comando
roslaunch ydlidar_ros_driver ydlidar.launch serial_port:=/dev/ttyUSB1 serial_baudrate:=115200
```

#### 6. Warnings durante compilación del SDK

**Síntoma**: Muchos warnings de formato durante `make`

**Respuesta**: ✅ **NORMAL** - No son errores
- Son warnings del código fuente de YDLidar
- No afectan el funcionamiento
- El SDK funciona correctamente

---

## ⚙️ Configuración Avanzada

### Parámetros del Launch File - YDLIDAR X2

**Launch file funcional** (`x2_custom.launch`):

```xml
<launch>
  <node name="ydlidar_lidar_publisher" pkg="ydlidar_ros_driver" type="ydlidar_ros_driver_node" output="screen" respawn="false">
    <!-- YDLIDAR X2 - Configuración verificada -->
    <param name="port"         type="string" value="/dev/ttyUSB0"/>  
    <param name="baudrate"     type="int"    value="115200"/>  
    <param name="frame_id"     type="string" value="laser"/>
    <param name="ignore_array" type="string" value=""/>

    <!-- Tipo de LIDAR -->
    <param name="lidar_type"       type="int" value="1"/>  <!-- 1=TRIANGLE -->
    <param name="device_type"      type="int" value="0"/>  <!-- 0=SERIAL -->
    <param name="sample_rate"      type="int" value="3"/>  <!-- X2 usa 3K -->
    <param name="abnormal_check_count" type="int" value="4"/>  

    <!-- Configuración booleana CRÍTICA para X2 -->
    <param name="resolution_fixed"    type="bool" value="true"/>
    <param name="auto_reconnect"      type="bool" value="true"/>
    <param name="reversion"           type="bool" value="false"/>  <!-- X2: false -->
    <param name="inverted"            type="bool" value="true"/>
    <param name="isSingleChannel"     type="bool" value="true"/>   <!-- X2: true -->
    <param name="intensity"           type="bool" value="false"/>
    <param name="support_motor_dtr"   type="bool" value="true"/>   <!-- X2: true -->
    <param name="invalid_range_is_inf" type="bool" value="false"/>
    <param name="point_cloud_preservative" type="bool" value="false"/>

    <!-- Rangos para X2 -->
    <param name="angle_min"    type="double" value="-180" />
    <param name="angle_max"    type="double" value="180" />
    <param name="range_min"    type="double" value="0.1" />
    <param name="range_max"    type="double" value="12.0" />
    <param name="frequency"    type="double" value="10.0"/>
  </node>
  
  <!-- TF estático -->
  <node pkg="tf" type="static_transform_publisher" name="base_link_to_laser"
    args="0.0 0.0 0.15 0.0 0.0 0.0 /base_link /laser 50" />
</launch>
```

**Parámetros críticos para X2**:
- `sample_rate: 3` (no 5 o 9)
- `isSingleChannel: true` (es single channel)
- `reversion: false` (no invertir)
- `support_motor_dtr: true` (control de motor)

### Configuración de Permisos Permanente

```bash
# Obtener información del dispositivo
udevadm info -a -n /dev/ttyUSB0 | grep '{serial}'

# Crear regla personalizada
sudo nano /etc/udev/rules.d/99-ydlidar.rules
```

Contenido:
```
# YDLidar USB Rules
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout", SYMLINK+="ydlidar"

# Alternativa con número de serie específico (si lo tienes)
# KERNEL=="ttyUSB*", ATTRS{serial}=="0001", MODE:="0666", SYMLINK+="ydlidar_main"
```

Aplicar:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Ahora puedes usar `/dev/ydlidar` en lugar de `/dev/ttyUSB0`

### Filtrado de Datos

Aplicar filtros a los datos del LIDAR:

```xml
<launch>
  <!-- LIDAR -->
  <include file="$(find ydlidar_ros_driver)/launch/ydlidar.launch"/>
  
  <!-- Filtro de rango -->
  <node pkg="laser_filters" type="scan_to_scan_filter_chain" name="laser_filter">
    <rosparam command="load" file="$(find your_package)/config/laser_filter.yaml"/>
  </node>
</launch>
```

`laser_filter.yaml`:
```yaml
scan_filter_chain:
- name: range
  type: laser_filters/LaserScanRangeFilter
  params:
    use_message_range_limits: false
    lower_threshold: 0.15
    upper_threshold: 10.0
    
- name: intensity
  type: laser_filters/LaserScanIntensityFilter
  params:
    lower_threshold: 100
    upper_threshold: 10000
```

---

## 🤖 Integración con Sphero RVR

### Paso 4: Configurar los Parámetros para el Modelo X2

El paquete `ydlidar_ros_driver` trae configuraciones para varios modelos. Para el **X2** hay que asegurarse de usar puerto, baudrate y `frame_id` correctos.

**Opción A – Usar el launch del paquete YDLIDAR (X2.launch)**

Si usas el launch que viene en el repositorio del driver:

```bash
nano ~/atriz_git/src/ydlidar_ros_driver/launch/X2.launch
```

**Opción B – Usar los launch de este proyecto (recomendado)**

En este proyecto el X2 se configura en:

- `atriz_rvr_driver/launch/rvr_with_lidar.launch` (RVR + LIDAR)
- `atriz_rvr_driver/launch/lidar_only.launch` (solo LIDAR)
- O el `x2_custom.launch` que genera `install_lidar_driver.sh` en `ydlidar_ros_driver/launch/`

Verifica que los parámetros clave sean:

- `port`: **`/dev/ydlidar`** (o `/dev/ttyUSB0` si no usas la regla udev)
- `baudrate`: **`115200`**
- `frame_id`: **`laser_frame`** (o `laser`; en este proyecto se usa `laser` por defecto; para RViz con Fixed Frame `odom` puedes usar `laser_frame`)

Ejemplo de bloque XML:

```xml
<param name="port"         type="string" value="/dev/ydlidar"/>  
<param name="baudrate"     type="int"    value="115200"/>  
<param name="frame_id"     type="string" value="laser_frame"/>
```

---

### Paso 5: Fusión de TF (Transformaciones) con el Sphero RVR

El nodo **Atriz_rvr_node** publica el marco **`rvr_base_link`** (y la odometría en **`odom`**). El YDLIDAR publica los datos en su propio frame (p. ej. **`laser`** o **`laser_frame`**). Para que ROS sitúe los obstáculos respecto al robot, hay que publicar la transformación fija desde `rvr_base_link` hasta el frame del LIDAR.

En este proyecto el **archivo launch maestro** que une RVR y LIDAR es:

```text
atriz_rvr_driver/launch/rvr_with_lidar.launch
```

Equivale al “rvr_lidar.launch” que se suele crear a mano: incluye el nodo del RVR, el driver del LIDAR y un `static_transform_publisher` con la posición del LIDAR respecto al RVR.

En ese launch, la etiqueta relevante es algo como:

```xml
<node pkg="tf" type="static_transform_publisher" name="base_link_to_laser"
      args="0 0 0.1 0 0 0 rvr_base_link laser_frame 50" />
```

Los argumentos **`0 0 0.1 0 0 0`** son **X Y Z Yaw Pitch Roll** en metros y radianes. Ahí se asume que el LIDAR está en el centro del RVR y **0,1 m (10 cm) por encima**. Ajusta estos valores a tu montaje real.

**Árbol TF resultante:**

- `odom` → `rvr_base_link` (lo publica el driver del RVR)
- `rvr_base_link` → `laser` / `laser_frame` (static_transform_publisher)

Así, los puntos del LIDAR quedan correctamente referidos al robot y a la odometría.

---

### Paso 6: Ejecución y Verificación

1. **Lanzar el sistema RVR + LIDAR**

   Desde el workspace:

   ```bash
   cd ~/atriz_git
   source devel/setup.bash
   roslaunch atriz_rvr_driver rvr_with_lidar.launch
   ```

   (Si solo quieres probar el LIDAR sin RVR: `roslaunch atriz_rvr_driver lidar_only.launch`.)

2. **Abrir RViz** (en otra terminal o desde un PC con `ROS_MASTER_URI` apuntando a la Raspberry):

   ```bash
   rviz
   ```

3. **Configurar RViz**

   - **Fixed Frame**: `odom` (así ves el robot y el LIDAR en el mismo marco).
   - Añadir **TF**: para ver `rvr_base_link` y el frame del LIDAR (`laser` o `laser_frame`) moverse juntos.
   - Añadir **LaserScan**: tópico **`/scan`**.

Si todo está bien, verás el Sphero (su TF) en el centro moviéndose con la odometría y los puntos del LIDAR (en rojo) detectando paredes y obstáculos alrededor.

¿Quieres seguir con un nodo de **Gmapping (SLAM)** para que el RVR construya mapas 2D con estos datos? Puedes usar la sección [SLAM con gmapping](#slam-con-gmapping) más abajo.

---

### Montaje Físico

1. **Posición**: Montar el LIDAR en la parte superior del RVR
2. **Altura**: Elevar si es necesario para evitar obstáculos del propio robot
3. **Orientación**: Marcar el frente del LIDAR alineado con el frente del robot
4. **Alimentación**: Conectar a la fuente de 5V del RVR o usar batería externa

### Configuración de TF (Transform)

El driver del RVR publica **odom → rvr_base_link**. Para que el LIDAR esté en el mismo árbol, publica **rvr_base_link → laser** (o **laser_frame**):

```xml
<!-- TF estático: rvr_base_link -> laser (X Y Z Yaw Pitch Roll en m y rad) -->
<node pkg="tf" type="static_transform_publisher" name="base_link_to_laser"
  args="0.0 0.0 0.10 0.0 0.0 0.0 rvr_base_link laser 50"/>
```

Ajusta los valores según la posición real del LIDAR sobre el RVR.

### Launch File Completo RVR + LIDAR

El archivo **`atriz_rvr_driver/launch/rvr_with_lidar.launch`** ya hace esto. Equivalente manual:

```xml
<launch>
  <!-- Sphero RVR Driver (publica odom -> rvr_base_link) -->
  <include file="$(find atriz_rvr_driver)/launch/atriz_rvr_node.launch"/>
  
  <!-- LIDAR X2 (parámetros en el launch; frame_id laser o laser_frame) -->
  <include file="$(find ydlidar_ros_driver)/launch/x2_custom.launch"/>
  <!-- O incluir el nodo con params como en rvr_with_lidar.launch -->
  
  <!-- Transform rvr_base_link -> laser (Paso 5) -->
  <node pkg="tf" type="static_transform_publisher" name="base_link_to_laser"
    args="0.0 0.0 0.10 0.0 0.0 0.0 rvr_base_link laser 50"/>
</launch>
```

Para visualizar: `rviz` con Fixed Frame **odom**, display **TF** y **LaserScan** en `/scan`.

### SLAM con gmapping

```bash
# Instalar gmapping
sudo apt-get install ros-noetic-gmapping

# Crear launch file para SLAM
```

`slam.launch`:
```xml
<launch>
  <!-- RVR + LIDAR (usa rvr_with_lidar.launch) -->
  <include file="$(find atriz_rvr_driver)/launch/rvr_with_lidar.launch"/>
  
  <!-- SLAM gmapping -->
  <node pkg="gmapping" type="slam_gmapping" name="slam_gmapping">
    <param name="base_frame" value="rvr_base_link"/>
    <param name="odom_frame" value="odom"/>
    <param name="map_frame" value="map"/>
    
    <param name="map_update_interval" value="2.0"/>
    <param name="maxUrange" value="10.0"/>
    <param name="sigma" value="0.05"/>
    <param name="kernelSize" value="1"/>
    <param name="lstep" value="0.05"/>
    <param name="astep" value="0.05"/>
    <param name="iterations" value="5"/>
    <param name="lsigma" value="0.075"/>
    <param name="ogain" value="3.0"/>
    <param name="lskip" value="0"/>
    
    <param name="minimumScore" value="50"/>
    <param name="srr" value="0.1"/>
    <param name="srt" value="0.2"/>
    <param name="str" value="0.1"/>
    <param name="stt" value="0.2"/>
    
    <param name="linearUpdate" value="0.2"/>
    <param name="angularUpdate" value="0.2"/>
    <param name="temporalUpdate" value="0.5"/>
    
    <param name="xmin" value="-10.0"/>
    <param name="ymin" value="-10.0"/>
    <param name="xmax" value="10.0"/>
    <param name="ymax" value="10.0"/>
    <param name="delta" value="0.05"/>
    
    <param name="particles" value="30"/>
  </node>
</launch>
```

### Navegación Autónoma

```bash
# Instalar navigation stack
sudo apt-get install ros-noetic-navigation
```

---

## 📊 Herramientas de Diagnóstico

### Scripts de Prueba

#### test_lidar.py (Ya incluido)
```bash
# Prueba básica
python3 test_lidar.py

# Escanear baudrates
python3 test_lidar.py --scan

# Probar protocolo específico
python3 test_lidar.py --rplidar
```

#### Ver Información del LIDAR
```bash
# Topics
rostopic list | grep scan

# Información del topic
rostopic info /scan

# Tipo de mensaje
rosmsg show sensor_msgs/LaserScan

# Frecuencia
rostopic hz /scan

# Bandwidth
rostopic bw /scan
```

#### Grabar Datos
```bash
# Grabar datos del LIDAR
rosbag record /scan /tf

# Reproducir
rosbag play archivo.bag
```

---

## 📚 Referencias y Recursos

### Documentación Oficial
- **YDLidar SDK**: https://github.com/YDLIDAR/YDLidar-SDK
- **YDLidar ROS Driver**: https://github.com/YDLIDAR/ydlidar_ros_driver
- **ROS LaserScan**: http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html

### Archivos Locales
- **Script de instalación**: `install_lidar_driver.sh`
- **Script de prueba**: `test_lidar.py`
- **Esta guía**: `GUIA_COMPLETA_LIDAR.md`

### Contacto y Soporte
- Revisa primero la sección de Solución de Problemas
- Consulta los logs: `rosrun rqt_console rqt_console`
- Issues de YDLidar: https://github.com/YDLIDAR/ydlidar_ros_driver/issues

---

## 🎯 Resumen Ejecutivo

### Para Empezar Rápidamente:

```bash
# 1. Source del workspace
cd /home/sphero/atriz_git && source devel/setup.bash

# 2. Dar permisos al puerto (si es necesario)
sudo chmod 666 /dev/ttyUSB0

# 3. Ejecutar LIDAR X2 (con integración RVR)
roslaunch atriz_rvr_driver lidar_only.launch

# O ejecutar driver directo:
# roslaunch ydlidar_ros_driver x2_custom.launch

# 4. En otra terminal - Visualizar
rviz
# Add → LaserScan → Topic: /scan → Fixed Frame: laser
```

### Configuración Confirmada - YDLIDAR X2:
- ✅ Modelo: **YDLIDAR X2**
- ✅ Puerto: `/dev/ttyUSB0`
- ✅ Baudrate: `115200`
- ✅ Sample Rate: `3K`
- ✅ Frecuencia: `10 Hz`
- ✅ Driver: `ydlidar_ros_driver`
- ✅ SDK: Instalado y funcionando
- ✅ Launch file: `x2_custom.launch`
- ✅ Estado: **✨ FUNCIONANDO PERFECTAMENTE**

---

**Última actualización**: Configuración completada y verificada  
**Autor**: Asistente AI  
**Versión**: 1.0

