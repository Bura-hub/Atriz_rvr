cmake_minimum_required(VERSION 3.8)
project(atriz_rvr_description)

# Paquete de solo datos: URDF/xacro, launch y configuración de RViz2. No compila
# nada, solo instala. Por eso es ament_cmake y no ament_python: es lo habitual
# para paquetes de descripción, y así `$(find-pkg-share …)` funciona desde los
# launch de cualquier otro paquete.

find_package(ament_cmake REQUIRED)

# OJO: solo directorios que EXISTAN en el repositorio. Git no versiona
# directorios vacíos, así que listar aquí un `rviz/` vacío haría fallar el build
# en un clon recién hecho — con un error de CMake poco evidente. Se añade cuando
# haya una configuración de RViz2 de verdad (Fase 4, al montar SLAM).
install(
  DIRECTORY urdf launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
