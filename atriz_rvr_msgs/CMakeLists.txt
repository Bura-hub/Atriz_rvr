cmake_minimum_required(VERSION 3.8)
project(atriz_rvr_msgs)

# ─────────────────────────────────────────────────────────────────────────────
# Portado de catkin a ament_cmake + rosidl el 2026-07-30.
#
# Diferencias con la versión de ROS 1, para quien venga del CMakeLists anterior:
#
#   catkin_package() + add_message_files() + add_service_files()
#       -> rosidl_generate_interfaces(), que recibe msg y srv en la MISMA lista
#   generate_messages(DEPENDENCIES ...)
#       -> el argumento DEPENDENCIES de rosidl_generate_interfaces()
#   Las rutas van con prefijo: "msg/Color.msg", no solo "Color.msg"
#
# Y un cambio en las interfaces: los tres .srv que declaraban `Header header`
# ahora dicen `std_msgs/Header header`. En ROS 1 `Header` a secas resolvía; en
# ROS 2 NO.
# ─────────────────────────────────────────────────────────────────────────────

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # ── Mensajes (8) ───────────────────────────────────────────────────────────
  # 📝 El rótulo decía «(6)» con SIETE ficheros en la lista desde antes de este
  #    cambio. Se corrige al añadir el octavo: un contador que no cuadra es lo
  #    que hace que la siguiente persona dude de la lista entera.
  "msg/Color.msg"
  "msg/ControlState.msg"
  "msg/DegreesTwist.msg"
  "msg/Encoder.msg"
  "msg/EstadoNavegacion.msg"
  "msg/EstadoRobot.msg"
  "msg/InfraredMessage.msg"
  "msg/MotorStatus.msg"
  "msg/SystemInfo.msg"

  # ── Servicios (20) ─────────────────────────────────────────────────────────
  "srv/BatteryState.srv"
  "srv/ConfigureStreaming.srv"
  "srv/GetControlState.srv"
  "srv/GetEncoders.srv"
  "srv/GetRGBCSensorValues.srv"
  "srv/GetSystemInfo.srv"
  "srv/MoveTimed.srv"
  "srv/MoveToPosAndYaw.srv"
  "srv/MoveToPose.srv"
  "srv/RawMotors.srv"
  "srv/SendInfraredMessage.srv"
  "srv/SetDriveParameters.srv"
  "srv/SetIREvading.srv"
  "srv/SetIRMode.srv"
  "srv/SetLEDRGB.srv"
  "srv/SetLeds.srv"
  "srv/SetMultipleLEDs.srv"
  "srv/SetPosAndYaw.srv"
  "srv/StartStreaming.srv"
  "srv/TriggerLedEvent.srv"

  DEPENDENCIES std_msgs geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
