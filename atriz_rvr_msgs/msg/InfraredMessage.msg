# Un mensaje IR recibido de OTRO robot.
#
# 🔴 REHECHO EL 2026-08-11, Y ES UN CAMBIO INCOMPATIBLE A PROPÓSITO.
#
#    Antes tenía cinco campos: `code` y las cuatro intensidades
#    (front/left/right/rear_strength). CUATRO DE LOS CINCO ERAN FICCIÓN.
#
#    La notificación del firmware (CID 0x2C) declara UN SOLO campo de salida, y
#    el payload real —medido en rvr-01 el 2026-08-11 con rvr-02 emitiendo— es:
#
#        {'infrared_code': 3}
#
#    Las intensidades son parámetros del ENVÍO (`SendInfraredMessage`), donde sí
#    existen: dicen con cuánta fuerza emite cada uno de los cuatro emisores. En
#    la RECEPCIÓN no llegan, y no hay forma de saberlas.
#
#    Los dos drivers, el de ROS 1 y el de ROS 2, intentaban leerlas igualmente.
#    El de ROS 1 hacía `datos['InfraredMessage']['Code']` contra ese payload:
#    KeyError en la primera línea. Nunca recibió un solo mensaje.
#
#    Se rompe ahora porque es el momento barato: no lo consume nadie. Ni la web
#    —no está en la lista blanca de rosbridge— ni ninguna práctica de alumno.
#
#    Diseño: docs/superpowers/specs/2026-08-11-sistema-ir-robot-a-robot-design.md
#    Evidencia: 00_auditoria/evidencia/99_el_ir_robot_a_robot_...txt

# Cuándo llegó. NO es adorno: la lectura IR del firmware caduca en 1 segundo, así
# que un mensaje sin marca de tiempo no se puede juzgar.
std_msgs/Header header

# El código que emitió el otro robot. Rango válido de emisión: 0-7.
# 📝 El registro de LECTURA del firmware admite 0-15 (referencia_sdk/sensor.md:54)
#    y la documentación no explica la discrepancia. Se publica lo que llegue.
uint8 code
