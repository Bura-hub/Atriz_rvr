# Mapas del aula

## 🔴🔴 LO PRIMERO: UN MAPA QUE NO ES DEL SITIO HACE QUE NAV2 MIENTA

Medido el 2026-08-07 (evidencias 83 y 84). Mismo cuarto, mismo robot, mismo recorrido de 80 cm,
mismos parámetros de AMCL. **Lo único distinto: el mapa.**

```
                          mapa rancio    tanda 1    tanda 2
  error de AMCL              45,0 cm      8,9 cm    15,2 cm
  corrección map → odom       0,424 m     0,028 m    0,021 m
  distancia real al objetivo  41,3 cm      6,1 cm    11,8 cm
  ¿dentro de los 10 cm?        🔴 NO       ✅ SÍ      🔴 NO
  lo que dijo Nav2            ✅ ÉXITO    ✅ ÉXITO   ✅ ÉXITO   ← 🔴 LAS TRES
```

🔴 **El síntoma es que no hay síntoma.** Nav2 declara el objetivo cumplido, `/estado_navegacion`
dice `FUNCIONANDO`, no aparece un error en ningún log, y el robot está **a medio metro** de donde
cree estar. No hay nada en el sistema que lo detecte: **lo destapó una cinta métrica**.

📌 **Consecuencia operativa: mapear es parte de MONTAR EL AULA, no una tarea de una sola vez.**
Si se mueven las mesas, se mapea otra vez. Un mapa de la semana pasada con los muebles cambiados
reproduce el fallo entero.

## 🔑 Y una decisión que hay que tomar ANTES de mapear: ¿con los muebles puestos o no?

No es lo mismo, y **cambia por dónde puede pasar el robot**. Medido el 2026-08-20 en la arena del
laboratorio, con el **mismo hueco físico de 40 cm** y el mismo instrumento (49 muestras del costmap
contando celdas transitables en la fila más estrecha):

```
mapa SIN los objetos (los ve solo la capa de obstaculos)   1 celda estable + 3 parpadeando
mapa CON los objetos (entran en la capa estatica)          0 celdas · 49/49 CERRADAS
```

**Lo que está en el mapa entra ya engordado** (~5 cm por lado), así que el mismo paso «vale» unos
**7 cm menos**. En números para montar un aula:

| | paso mínimo |
|---|---|
| mobiliario **en el mapa** (mapeaste con él puesto) | **~49 cm** |
| mobiliario **añadido después** (una silla que alguien movió) | **~42-45 cm** |

⚠️ Y eso es lo que pide el **planificador**. Encima está lo que pide la **capa de seguridad**, que es
otra cuenta y va aparte: `hueco practicable ≈ 2 × Aproximacion.radius + 2 × deriva` → con este robot
**40 cm es el límite y 45-50 lo cómodo** (medido cruzando: 41 % de lo mandado dentro de la puerta,
11 % al salir). **Manda la más exigente de las dos.**

⚠️ Y **lo que se arregló remapeando NO fue todo**: hubo dos fallos distintos con el mismo síntoma
aparente. El otro —el marco `map → odom` rotando 98°— era la recuperación de «robot secuestrado»
de AMCL, y se cerró apagando `recovery_alpha_slow/fast` (evidencia 82, ver
`config/localizacion_amcl.yaml`). Arreglar uno dejaba el otro en pie.

## Dónde vive cada mapa, que son DOS sitios y no es un descuido

| Sitio | Qué es | Quién lo pone |
|---|---|---|
| **este directorio** (`aula.yaml`) | el mapa **de la flota**, igual en los 16 | va con el paquete: lo reparten `provision.sh` y la imagen dorada |
| **`~/mapas/`** (`ATRIZ_DIR_MAPAS`) | lo que **SLAM produce** en este robot | `atriz-slam.service`, al mapear |

## 🔑 `aula.yaml` DE ESTE REPOSITORIO ES LA ARENA DE ATRIZ — reemplázalo en otra aula

👤 **Decidido el 2026-08-20.** Aquí vive el mapa **de la arena del laboratorio de Atriz**
(~3,95 × 4,00 m, mapeada el 2026-08-19 con la receta de abajo). Está en el repositorio **a
propósito**, y la razón es concreta: los 16 robots comparten **la misma arena**, así que clonar el
mapa es correcto — al revés que con un mapa de casa, que en 15 robots sería de otro sitio.

🔴 **Sin esto, los 15 clones saldrían SIN MAPA.** `fase_6` borra `~/mapas` y vacía `ATRIZ_MAPA` (y
hace bien), y el mapa vivía sólo ahí. Con el fichero en el paquete, el valor **por defecto** del
supervisor y de `atriz-nav.sh` —`~/atriz_ws/src/Atriz_rvr/atriz_rvr_bringup/maps/aula.yaml`— lo
encuentra sin que nadie configure nada.

⚠️ **Y ESTO ES LO QUE HAY QUE HACER EN OTRA AULA, o Nav2 dirá «llegué» a medio metro** (el fallo de
arriba, que no tiene ningún otro síntoma):

1. mapea tu aula con la receta de este documento;
2. **sustituye `aula.yaml` y `aula.pgm`** por los tuyos, o apunta `ATRIZ_MAPA` a los tuyos en
   `/etc/default/atriz`;
3. comprueba por efecto que `/estado_navegacion` dice el **nombre** y la **edad** que esperas.

📌 El `.yaml` referencia su imagen **relativa a su propio directorio**, así que los dos ficheros
viajan juntos y con el mismo nombre base.

Y **quien decide cuál se usa es `ATRIZ_MAPA` de `/etc/default/atriz`**, no la convención de
nombres. Puede apuntar a cualquiera de los dos.

🔴 **La imagen dorada sale SIN mapa y sin `ATRIZ_MAPA`, a propósito**
(`fase_6_preparar_imagen_dorada.sh`): clonar el mapa del robot de referencia repartiría a los 16
un mapa que en 15 de ellos ni siquiera es del mismo sitio, y con el modo de fallo silencioso de
arriba.

El mapa compartido es, además, **el argumento entero para usar AMCL en vez de SLAM** — no la CPU,
que en AMCL es **mayor** (8,8 % contra 4,8 %). Un `map` común es lo que permite que la web diga
«ve a la mesa 3» y signifique lo mismo para los 16 robots.

## Cómo se genera

Desde la web (botón de SLAM) o a mano:

```bash
ros2 launch atriz_rvr_bringup slam.launch.py     # y pasear el robot por el aula
ros2 run nav2_map_server map_saver_cli -f aula --ros-args -p save_map_timeout:=10.0
```

⚠️ `save_map` falla con `result=255` de forma **intermitente** (~1 de cada 3) por una carrera
entre el `map_update_interval` de slam_toolbox y el `save_map_timeout` del map_saver. De ahí el
`-p save_map_timeout:=10.0`.

⚠️ **Girar sobre el eje NO hace crecer el mapa.** El X2 barre los 360°, así que un robot que gira
en el sitio vuelve a ver lo mismo desde el mismo punto: cero información nueva. Hay que
**desplazarlo**, y no bastan 40 cm — `slam_toolbox` cuenta desde el último nodo del grafo.

🔴🔴 **Y EL MORRO TIENE QUE ARRANCAR PARALELO A UNA PARED, O EL MAPA SALE EN ROMBO.** Los ejes del
mapa son **los del robot en su primer barrido**. Medido el 2026-08-19 mapeando la misma arena tres
veces:

```
                 lienzo         ocupadas   desconocido   forma
v1 y v2      8,1x4,4 / 7,1x6,3   540/549    66 / 67 %    ROMBO (morro a ~45° de la pared)
v3           6,0x4,7             511        47 %         ✅ recto (4,1x4,0 al cerrar el perímetro)
```

✅ **La receta que lo cierra, y es de OPERACIÓN** (guion:
`Atriz_migracion_ros2/00_auditoria/evidencia/mediciones_banco/mapear_arena.py`):

1. robot en una **esquina**, a ~55 cm de cada pared, con la pared a su **izquierda** y el morro
   **PARALELO** a ella — se comprueba **con el propio LIDAR**, ajustando una recta a los puntos de
   esa pared, no a ojo;
2. `/set_pos_and_yaw(0,0,0)` y comprobar que `/odom` da **(0,000, 0,000, 0,0°)**;
3. arrancar `atriz-slam` **DESPUÉS**, y comprobar que `map → base_footprint` da también
   **(0,000, 0,000, 0,0°)**;
4. y **sólo entonces** conducir (perímetro primero, para cerrar lazos; luego el interior).

Así **el (0,0) del mapa ES esa esquina**, con testigo — y eso es lo que hace utilizable el mapa
después.

🔴 **Por qué esa convención no es cosmética: una sala CUADRADA no se puede localizar sola.**
Casando `/scan` contra el mapa salieron **tres candidatos empatados** (coste 0,003 · 0,004 · 0,004)
y **conducir 1 m no los desempató**. Con un LIDAR de 360° en una sala simétrica no hay nada que
rompa el empate, y este robot **no tiene rumbo absoluto**. La pose de partida la da una persona:
para eso existe `/initialpose`, y para eso vale la esquina convenida.

⚠️ **Y mientras SLAM esté vivo, que NADIE toque el robot.** Mover el robot a mano con SLAM
corriendo produce el **mismo síntoma que la congelación del `collision_monitor`** —lecturas de
`/scan` idénticas tramo tras tramo y giros de 0,0°—, y costó **dos mapas** el 2026-08-19 antes de
que se preguntara a la persona que estaba al lado.

⚠️ **El `.yaml` no basta: tiene que existir su `.pgm`**, y se resuelve **relativo al directorio
del yaml**. Copiar solo el `.yaml` a otro sitio hace fallar a `map_server` **después** de arrancar
la unidad, lo que consume reintentos igual que no tener mapa.

### 📌 Y un orden que importa, si vas a medir la precisión

Al validar la navegación contra cinta métrica, **coloca y marca el robot ANTES de mapear**:

1. pon el robot donde quieras el origen y **no lo muevas**
2. arranca SLAM **ahí** → el origen del mapa es ese punto
3. pasea, y **devuelve el robot al origen**
4. marca `A` en el suelo (las cuatro esquinas del chasis, y cruza las diagonales)
5. marca `B` a ~1 m, perpendicular a como mira, **sobre la línea que pasa por `A`**

Al revés, el origen del mapa y la marca del suelo son puntos distintos y las medidas no comparan
lo que crees. Costó una tanda entera el 2026-08-07.

🔴 **Y hacen falta DOS distancias, no una.** Medir solo la diagonal `A→P` deja al robot en
cualquier punto de una circunferencia: en la evidencia 83 la odometría y AMCL coincidían en
distancia (2 cm) mientras estaban **a 45 cm la una de la otra**. La segunda distancia (`B→P`) es
la que discrimina. Herramienta:
`atriz_migracion/00_auditoria/evidencia/mediciones_banco/comparar_con_cinta.py`.

## Estado

⏳ **El mapa del aula NO existe todavía** (2026-08-07). Todo lo medido hasta hoy se hizo en casa.
`aula.yaml` se creará el día que se monte el laboratorio, y **es un requisito de puesta en
marcha**, no una mejora.

✅ Lo que sí está verificado, sobre un mapa fresco de un cuarto de 3,80 × 4,20 m: Nav2 acepta el
objetivo, navega, y **para a 6,1 y 11,8 cm de él** en dos tandas (n=2). La tolerancia configurada
son 10 cm, así que **una tanda cayó dentro y la otra fuera**.

🔴 **La cifra honesta que hay que dar es «~10-12 cm», no la tolerancia.** Y sobre todo: **Nav2
declaró `SUCCEEDED` en las tres tandas**, a 6,1, a 11,8 y a 41,3 cm. El desenlace del objetivo
**no informa de la precisión**, y nada que se le prometa a un alumno puede apoyarse en él.
