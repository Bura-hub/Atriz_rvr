# Mapas del aula

`atriz-nav.service` busca aquí **`aula.yaml`**, y **falla alto si no está**: AMCL lo necesita y
`localizacion.launch.py` no tiene valor por defecto para el argumento `mapa`. Arrancar sin él
daría una navegación ciega que parece viva.

El mapa vive **con el paquete** a propósito: así lo reparten `provision.sh` y la imagen dorada, y
los 16 robots comparten el mismo `map`. Ese marco compartido es el argumento entero para usar
AMCL en vez de SLAM — no la CPU, que en AMCL es **mayor** (8.8 % contra 4.8 %).

## Cómo se genera

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

## Estado

⏳ **El mapa del aula NO existe todavía** (2026-08-03). Las medidas de esa fecha se tomaron en
casa, no en el laboratorio.
