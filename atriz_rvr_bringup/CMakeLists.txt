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

ament_package()
