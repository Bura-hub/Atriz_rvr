cmake_minimum_required(VERSION 3.8)
project(atriz_rvr_bringup)

# Paquete de solo datos: launch, parámetros y reglas udev. No compila nada.
find_package(ament_cmake REQUIRED)

# Solo directorios que EXISTAN: git no versiona directorios vacíos, y listar uno
# vacío rompería el build en un clon recién hecho.
install(
  DIRECTORY launch config udev
  DESTINATION share/${PROJECT_NAME}
)

# El rosbridge que exige testigo (Fase B de SEGURIDAD_ROSBRIDGE.md). Va a `lib/`
# y no a `share/` porque `Node(executable=...)` lo busca ahí.
#
# 🔴 `atriz_rosbridge.py` con PROGRAMS (queda ejecutable) y `rosbridge_nucleo.py`
#    con FILES (solo se importa). Si el núcleo faltara, el lanzador se NIEGA a
#    arrancar en vez de seguir sin comprobar nada.
install(
  PROGRAMS scripts/atriz_rosbridge.py
  DESTINATION lib/${PROJECT_NAME}
)
install(
  FILES scripts/rosbridge_nucleo.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
