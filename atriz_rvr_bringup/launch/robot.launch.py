"""Arranca el robot completo: driver del RVR + descripción + LIDAR + seguridad.

    ros2 launch atriz_rvr_bringup robot.launch.py

    # sin LIDAR (para probar solo el RVR):
    ros2 launch atriz_rvr_bringup robot.launch.py lidar:=false

    # sin la capa de seguridad (mediciones de banco que mandan velocidad cruda):
    ros2 launch atriz_rvr_bringup robot.launch.py collision_monitor:=false

Es el punto de entrada único del robot. Los nodos que arranca se reparten el
árbol TF, y hay que entender el reparto porque es donde estaba el bloqueante raíz
del proyecto:

    odom ──(rvr_driver)──► base_footprint ──(robot_state_publisher)──► base_link ──► laser
                                                                                       │
                                                                       /scan ◄──(ydlidar)

  · `rvr_driver`             publica `odom → base_footprint`, /odom, /imu, /color
  · `robot_state_publisher`  publica los transforms FIJOS desde el URDF
                             (`base_footprint → base_link → laser / imu_link / ruedas`)
  · `ydlidar_ros2_driver`    publica /scan con frame_id = `laser`

🔴 **Los nombres tienen que coincidir en TRES sitios.** El `base_frame` del
driver, el link del URDF y el `frame_id` del LIDAR. Si uno se desalinea, el árbol
se parte y **falla en silencio**: `slam_toolbox` se queda esperando un transform
que nunca llega, sin un solo mensaje de error. Es exactamente lo que le pasó al
sistema de ROS 1, donde el driver decía `rvr_base_link` y el LIDAR colgaba de
`base_link`.

🔴 **El RVR se duerme solo si nadie le habla**, y cuando lo hace el nodo se queda
vivo publicando CERO sin dar un error (manual, cap. 9.8). El driver lo previene
hablándole cada 30 s —y publicando `/battery_state` de paso— y, si aun así deja
de llegar telemetría, lo dice y trata de reanudarla. Para reproducir el fallo a
propósito: `keepalive_period:=0.0`.

🔴 **LA CAPA DE SEGURIDAD VIVE AQUÍ, NO EN NAV2**, aunque el ejemplo oficial la
ponga con la navegación. Los estudiantes teleoperan **sin Nav2** —la web hablará
por rosbridge (plan, Fase 5)—, así que con el monitor en `nav2.launch.py` el caso
peligroso de verdad, una persona conduciendo el robot contra una pared desde otro
edificio, no estaría protegido. Detalle y los números en
`config/collision_monitor.yaml`.

    Nav2 (velocity_smoother) ─┐
    web / rosbridge          ─┼─► /cmd_vel_raw ─► collision_monitor ─► /cmd_vel ─► driver
    teleop / scripts         ─┘

⚠️ **Quien quiera mover el robot publica en `/cmd_vel_raw`.** Publicar en
`/cmd_vel` funciona —el driver obedece— pero **salta la seguridad en silencio**.

VERIFICAR SIEMPRE tras arrancar:

    ros2 run tf2_ros tf2_echo odom base_footprint   # ← LA prueba: es lo que pide SLAM
    ros2 run tf2_ros tf2_echo odom laser            # la cadena completa
    ros2 topic hz /scan                    # ~10 Hz (medido libre: 11.48 Hz)
    ros2 topic hz /odom                    # ~16.7 Hz  ← si es 0, ¿se durmió?
    ros2 topic echo /battery_state --once  # llega cada 30 s (es el keepalive)
    ros2 lifecycle get /collision_monitor  # active [3] ← si no, NO FILTRA NADA
    ros2 topic info /cmd_vel --verbose     # UN publicador, y es collision_monitor
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            Shutdown)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    bringup = FindPackageShare('atriz_rvr_bringup')
    descripcion = FindPackageShare('atriz_rvr_description')

    arg_lidar = DeclareLaunchArgument(
        'lidar', default_value='true',
        description='Arrancar el driver del YDLIDAR X2. false para probar solo el RVR.',
    )
    arg_ns = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Namespace del robot. El aislamiento entre robots lo da '
                    'ROS_DOMAIN_ID, no esto: ver ARQUITECTURA.md, Decisión 1.',
    )
    arg_puerto = DeclareLaunchArgument(
        'rvr_port', default_value='/dev/rvr',
        description='Puerto del RVR. /dev/rvr es un symlink de udev a ttyAMA0 (PL011).',
    )
    # 🔴 Estos dos existen porque el RVR SE DUERME SOLO si nadie le habla, y sin
    # ellos el nodo se queda vivo publicando cero sin dar un error (manual 9.8).
    # Se exponen como argumentos sobre todo para poder DESACTIVARLOS a propósito
    # y reproducir el fallo — que es como se mide el timeout real del RVR.
    arg_keepalive = DeclareLaunchArgument(
        'keepalive_period', default_value='30.0',
        description='Cada cuántos segundos se le habla al RVR para que no se '
                    'duerma. 0 lo desactiva: el robot se dormirá y dejará de '
                    'publicar. Publica battery_state en cada latido.',
    )
    arg_silencio = DeclareLaunchArgument(
        'silence_timeout', default_value='3.0',
        description='Segundos sin telemetría del RVR antes de avisar e intentar '
                    'reanudar el streaming. 0 desactiva el detector.',
    )
    # ✅ POR DEFECTO **FALSE** desde el 2026-07-31. El robot publica la
    # orientación PLANA, que es la físicamente correcta.
    #
    # 🔴 Y la razón NO es el efecto en la deriva de SLAM, que se midió y salió
    #    de ~1 cm sin significación (p=0.142, n=6 por rama; manual 9.12d). Se
    #    decidió NO perseguirlo: ~62 corridas y 5 h de robot para un efecto de
    #    1 cm sobre una tolerancia de objetivo de 10.
    #
    #    La razón es que **la inclinación no existe**:
    #      · el suelo está plano — medido con nivel, ≤ 0.40° en cuatro puntos
    #      · el error del acelerómetro es FIJO EN EL MARCO DEL ROBOT: no gira
    #        con él (accel.x −1.091 → −1.158 tras girar 177.8°)
    #      · y |g| sale un 3.8 % corto (9.435 contra 9.807): está descalibrado
    #
    #    Publicar 6.9° de inclinación falsa en `odom -> base_footprint` es
    #    publicar un dato que sabemos incorrecto, independientemente de que se
    #    pueda o no medir su efecto. REP-105 espera ahí la pose del robot.
    #
    # ⚠️ Con `true` se recupera el comportamiento anterior. Hace falta si algún
    #    día este robot trabaja en una superficie inclinada de verdad — pero
    #    entonces habría que calibrar antes el acelerómetro, porque el que hay
    #    no acierta ni el módulo.
    arg_inclinacion = DeclareLaunchArgument(
        'publicar_inclinacion', default_value='false',
        description='Publicar el roll y pitch de la IMU en /odom y TF. Por '
                    'defecto FALSE: la inclinación que reporta el RVR es un '
                    'artefacto de su acelerómetro descalibrado, no una '
                    'inclinación real (manual, cap. 13). Con true se recupera.',
    )

    # 🔴 Por defecto FALSE, y encenderlo tiene coste físico: deja un LED blanco
    # encendido bajo el chasis mientras el driver viva.
    #
    # Sin él, `/color` publica [0, 0, 0] SIEMPRE — el sensor no da nada sin su
    # luz: medido, canal claro 4 apagada contra 741 encendida, 185 veces
    # (manual, cap. 16). El driver lo avisa por el log al arrancar en vez de
    # publicar ceros en silencio, que es lo que hacía antes.
    # 🔴 rosbridge: es POR DONDE HABLA LA WEB (ARQUITECTURA.md, D2). Va aquí y no
    # en una unidad systemd propia porque así hereda el entorno ROS del servicio y
    # el ROS_DOMAIN_ID del robot, que es justo lo que systemd no sabe dar solo.
    #
    # 📝 Por defecto ENCENDIDO: un robot al que la web no puede hablar no sirve
    #    para nada en este laboratorio. Se apaga con `rosbridge:=false` para
    #    depurar en local.
    arg_rosbridge = DeclareLaunchArgument(
        'rosbridge', default_value='true',
        description='Servidor WebSocket para la web, en el puerto 9090.')

    arg_color = DeclareLaunchArgument(
        'color_detection', default_value='false',
        description='Encender el sensor de color. Con false, /color publica '
                    '[0,0,0]. Enciende un LED blanco bajo el chasis y gasta '
                    'batería: por eso no está activo por defecto.',
    )

    # 🔴 ESTOS DOS ESTABAN DECLARADOS EN EL NODO Y NO AQUI, y eso los hacia
    #    INALCANZABLES: `ros2 launch ... color_apagado_max_s:=20.0` se aceptaba
    #    EN SILENCIO y arrancaba con el valor por defecto. Se descubrio al
    #    comprobar el parametro EFECTIVO en vez de fiarse de la linea de
    #    comandos -- pedidos 20.0 y 0.0, el driver tenia 900.0 y 120.0.
    #    Sin esto, ajustarlos en el aula obliga a editar ficheros: exactamente
    #    lo que pedian que no hiciera falta.
    arg_color_inact = DeclareLaunchArgument(
        'color_apagado_inactividad_s', default_value='120.0',
        description='Segundos sin que nadie use el sensor de color antes de '
                    'apagarle la luz sola. Cuenta suscriptores de /color Y '
                    'llamadas a get_rgbc_sensor_values. 0 lo desactiva.',
    )
    arg_color_max = DeclareLaunchArgument(
        'color_apagado_max_s', default_value='900.0',
        description='Tope duro: segundos desde que se encendio la luz del '
                    'sensor, pase lo que pase. Existe porque rosbridge comparte '
                    'UNA suscripcion entre todos sus clientes, asi que una '
                    'pestana olvidada la mantendria encendida. 0 lo desactiva.',
    )

    # 🔴 Por defecto TRUE: la seguridad no se activa, se desactiva a propósito.
    arg_seguridad = DeclareLaunchArgument(
        'collision_monitor', default_value='true',
        description='Capa de seguridad. Con false, /cmd_vel_raw deja de existir y '
                    'hay que publicar en /cmd_vel directamente: el robot NO '
                    'esquiva nada. Solo para mediciones de banco.',
    )

    ns = LaunchConfiguration('namespace')
    seguridad_on = IfCondition(LaunchConfiguration('collision_monitor'))

    # ── El driver del RVR: publica odom -> base_footprint ─────────────────────
    #
    # 🔴 `on_exit=Shutdown()`: SI EL DRIVER MUERE, SE CAE EL LAUNCH ENTERO.
    #
    #    Sin esto, un nodo que muere deja el resto en pie —y `atriz-robot.service`
    #    sigue en `active (running)`, porque su PID principal es el `ros2 launch`,
    #    no el nodo. `Restart=always` NO se entera.
    #
    #    Medido el 2026-08-01: el driver estuvo **cuatro minutos muerto** por un
    #    `SyntaxError` con el servicio en verde. En un laboratorio remoto eso es un
    #    robot inservible sin que nadie lo sepa.
    #
    #    Con esto, la muerte del driver tumba el launch, systemd lo ve y lo
    #    reinicia en ~15 s. Solo se pone en el DRIVER: sin él no hay robot. Si se
    #    cae el LIDAR el robot sigue siendo útil para teleoperar, así que ese no
    #    tumba nada.
    rvr = Node(
        package='atriz_rvr_driver',
        executable='rvr_driver_node',
        name='rvr_driver',
        namespace=ns,
        output='screen',
        on_exit=Shutdown(),
        parameters=[{
            'serial_port': LaunchConfiguration('rvr_port'),
            # 🔴 base_footprint, NO base_link. Un frame solo puede tener UN
            # padre en TF, y el URDF ya publica base_footprint->base_link. Si el
            # driver publicara odom->base_link, base_link tendría dos padres y el
            # árbol se partiría en dos otra vez.
            # Es también lo que slam_toolbox pide en su `base_frame`.
            'base_frame': 'base_footprint',
            'odom_frame': 'odom',
            # 60 ms es el mínimo del firmware del RVR: a 50 ms no arranca, y
            # cuantiza a múltiplos de 20. Medido el 2026-07-29.
            'streaming_interval_ms': 60,
            'keepalive_period': ParameterValue(
                LaunchConfiguration('keepalive_period'), value_type=float),
            'silence_timeout': ParameterValue(
                LaunchConfiguration('silence_timeout'), value_type=float),
            'publicar_inclinacion': ParameterValue(
                LaunchConfiguration('publicar_inclinacion'), value_type=bool),
            'color_detection': ParameterValue(
                LaunchConfiguration('color_detection'), value_type=bool),
            'color_apagado_inactividad_s': ParameterValue(
                LaunchConfiguration('color_apagado_inactividad_s'),
                value_type=float),
            'color_apagado_max_s': ParameterValue(
                LaunchConfiguration('color_apagado_max_s'), value_type=float),
        }],
    )

    # ── La descripción: base_footprint -> base_link -> laser, imu_link, ruedas ─
    desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([descripcion, 'launch', 'description.launch.py'])
        ),
        launch_arguments={'namespace': ns}.items(),
    )

    # ── El LIDAR: publica /scan con frame_id 'laser' ──────────────────────────
    # La configuración vive en config/ydlidar_x2.yaml, con la procedencia de cada
    # valor documentada. Dos parámetros están SIN VERIFICAR y pueden arruinar el
    # mapa en silencio: `inverted` (mapa espejado) y `frequency` (puede que el X2
    # de canal único lo ignore). Léelo antes de mapear.
    lidar = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        namespace=ns,
        output='screen',
        condition=IfCondition(LaunchConfiguration('lidar')),
        parameters=[PathJoinSubstitution([bringup, 'config', 'ydlidar_x2.yaml'])],
    )

    # ── La capa de seguridad: /cmd_vel_raw -> /cmd_vel ────────────────────────
    # Los números de los polígonos y por qué son `approach`/`slowdown` y no
    # `stop`, en config/collision_monitor.yaml.
    #
    # 🔴 `on_exit=Shutdown()`, igual que el driver y por una razón MÁS fuerte:
    #    un robot **sin capa de seguridad que parece sano** es peligroso. Si este
    #    nodo muere, `/cmd_vel_raw` deja de filtrarse y nadie se entera — el
    #    servicio seguiría en `active (running)` porque su PID principal es el
    #    `ros2 launch`.
    #
    #    ⚠️ OJO A LA ASIMETRÍA CON EL LIDAR: si se cae el LIDAR el launch NO se
    #       cae, y no es una contradicción — sin `/scan` el propio
    #       `collision_monitor` bloquea el movimiento, así que el robot queda
    #       seguro. Si se cae el MONITOR, en cambio, el robot queda conduciendo
    #       sin filtro. Son dos situaciones opuestas.
    #
    #    Decisión del usuario, 2026-08-01.
    monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        namespace=ns,
        output='screen',
        condition=seguridad_on,
        on_exit=Shutdown(),
        parameters=[PathJoinSubstitution([bringup, 'config', 'collision_monitor.yaml']),
                    {'use_sim_time': False}],
    )

    # 🔴 collision_monitor ES UN NODO DE CICLO DE VIDA. Sin gestor arranca en
    # `unconfigured`: el proceso vive, `ros2 node list` lo muestra, **no filtra
    # absolutamente nada** y `/cmd_vel_raw` no llega al robot. Es el mismo fallo
    # que costó la Fase 4 con slam_toolbox, y aquí sería peor: el robot parecería
    # protegido y no lo estaría.
    #
    # Gestor propio, no el de Nav2: esto tiene que funcionar en teleoperación,
    # cuando nav2.launch.py ni siquiera está corriendo.
    gestor_seguridad = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_seguridad', namespace=ns, output='screen',
        condition=seguridad_on,
        parameters=[{'use_sim_time': False, 'autostart': True,
                     'node_names': ['collision_monitor']}],
    )

    # ═══════════════════════════════════════════════════════════════════════
    # EL PUENTE CON LA WEB, Y SU LISTA BLANCA
    # ═══════════════════════════════════════════════════════════════════════
    # 🔴🔴 HASTA EL 2026-08-02 ESTO ESTABA ABIERTO DE PAR EN PAR: solo se pasaba
    #    `port`, y eso deja los 18 servicios del driver alcanzables por cualquiera
    #    en la red — incluido `raw_motors`, que **se salta el collision_monitor y
    #    el watchdog y no tiene corte automatico**. Verificado de extremo a
    #    extremo: un navegador de otra subred abrio el WebSocket y encendio los
    #    faros de un robot (evidencia 39).
    #
    # 🔴 Y NO SE PUEDE ARREGLAR CON AUTENTICACION: **rosbridge 2.7.0 en Jazzy no
    #    la tiene**. No es que este sin configurar — no existe: `rosauth` no es
    #    dependencia, no hay parametro `authenticate`, la capacidad
    #    `Authentication` no esta en el protocolo, y `check_origin()` devuelve
    #    `True` incondicionalmente (`websocket_handler.py:233-234`), asi que
    #    tampoco filtra por origen. Lo unico que ofrece son LISTAS BLANCAS.
    #    → La identidad por usuario llega en la Fase B, con un proxy que valida
    #      el JWT delante de un rosbridge atado a 127.0.0.1.
    #      Diseño completo: `atriz_migracion/03_operacion/SEGURIDAD_ROSBRIDGE.md`
    #
    # 📝 SINTAXIS, que no es obvia (`rosbridge_websocket.py:115-126`): son CADENAS
    #    que contienen una lista estilo Python de patrones **fnmatch**, no regex.
    #        ""          -> sin filtro. TODO PERMITIDO. Era lo de antes.
    #        "[]"        -> filtra y no casa nada: TODO DENEGADO.
    #        "['/odom']" -> lista blanca.
    #    Y `services_glob`, si no es None, recibe `/rosapi/*` automaticamente.
    #
    # 🔴 ROSBRIDGE DENIEGA EN SILENCIO: registra un `warn` y hace `return`, SIN
    #    respuesta de error al cliente (`call_service.py:109-113`,
    #    `publish.py:96-99`, `subscribe.py:296-299`). Una lista mal puesta se
    #    manifiesta como «la web no responde», no como un fallo.
    #    → Por eso existe `mediciones_banco/probar_lista_blanca.py`, que comprueba
    #      las DOS direcciones: que lo prohibido se rechaza Y que lo permitido
    #      sigue funcionando. Si tocas estas listas, pasalo.

    #: Lo que la web LEE. Contrato en ARQUITECTURA.md.
    #: 📝 `/estado_robot` (AÑADIDO 2026-08-04, ⚠️ NO VERIFICADO) va SOLO aquí y
    #:    NUNCA en ESCRIBIR: la web lo lee, no lo escribe. Es la señal de vida
    #:    barata —~0,03 kB/s por robot contra los 13,05 de `/odom`— que permite al
    #:    muro del profesor vigilar 16 robots sin pagar 1,7 Mbit/s de telemetría.
    LEER = ['/odom', '/imu', '/scan', '/battery_state', '/motor_status',
            '/encoders', '/color', '/estado_robot', '/estado_navegacion',
            '/map', '/tf', '/tf_static',
            '/collision_monitor_state', '/amcl_pose']

    #: Lo que la web MANDA. 🔴 `cmd_vel_raw`, NUNCA `cmd_vel`: `/cmd_vel` es la
    #: SALIDA del collision_monitor, y publicar ahi funciona y salta la seguridad
    #: entera sin ningun aviso.
    ESCRIBIR = ['/cmd_vel_raw', '/emergency_stop', '/initialpose']

    #: Los servicios que la web necesita. De los 18 del driver, el ciclo normal
    #: usa MUY POCOS.
    #: 🔴 `set_pos_and_yaw` va DENTRO, y costo una revision darse cuenta: es el
    #:    UNICO modo de poner la odometria a cero entre alumnos. `reset_odom` no
    #:    existe; lo que hay es `set_pos_and_yaw(0,0,0)`, que llama a
    #:    `reset_locator_x_and_y()` y pone `_yaw_offset = None`. Solo acepta
    #:    (0,0,0) y rechaza el resto, asi que exponerlo es seguro. Sin el, la web
    #:    no puede resetear entre sesiones.
    #: 📝 `start_scan`/`stop_scan` son del nodo del YDLIDAR, no del driver.
    #: 🔴 `enable_color` + `get_rgbc_sensor_values` son la SESION DE MEDICION de
    #:    color: sin el LED encendido no hay lectura, y el LED solo lo enciende
    #:    `enable_color`. Van juntos o no sirve ninguno de los dos. Anadidos el
    #:    2026-08-06, cuando se midio que el encendido en caliente SI funciona
    #:    (`mediciones_banco/probar_color_stream_caliente.py`: claro 1 -> 1321).
    #: ⚠️ `enable_color(true)` deja un LED blanco encendido bajo el chasis y gasta
    #:    bateria. La web tiene que ofrecer el boton de PARAR igual de visible que
    #:    el de arrancar.
    SERVICIOS = ['/start_scan', '/stop_scan', '/release_emergency_stop',
                 '/set_pos_and_yaw',
                 '/set_led_rgb', '/set_multiple_leds', '/set_leds',
                 '/trigger_led_event',
                 '/enable_color', '/get_rgbc_sensor_values',
                 #: 🔴 Los botones de SLAM y Nav2. Los sirve `supervisor_navegacion`,
                 #:    NO el driver. `success=true` significa PETICION ACEPTADA,
                 #:    jamas «arrancado»: quien dice si funciona es
                 #:    `/estado_navegacion`, que va en LEER.
                 '/pedir_slam', '/pedir_nav']

    def _glob(lista):
        """rosbridge quiere una CADENA con una lista, no una lista."""
        return str(lista)

    puente = Node(
        package='rosbridge_server', executable='rosbridge_websocket',
        name='rosbridge_websocket', namespace=ns, output='screen',
        condition=IfCondition(LaunchConfiguration('rosbridge')),
        parameters=[{
            'port': 9090, 'use_sim_time': False,
            'topics_sub_glob': _glob(LEER),
            'topics_pub_glob': _glob(ESCRIBIR),
            'services_glob': _glob(SERVICIOS),
            'actions_glob': _glob(['/navigate_to_pose']),
            # 🔴 La web NO cambia parametros ROS. "[]" es denegar todo; "" seria
            #    permitir todo, que es justo lo contrario.
            'params_glob': '[]',
        }],
    )

    # 🔴 `rosapi_node` NO se arrancaba, y sin el `ros.getTopics()`,
    #    `getServices()` y `getTopicType()` de roslibjs **se cuelgan sin error**:
    #    ninguna UI puede descubrir topics por introspeccion. El launch oficial de
    #    rosbridge si lo arranca; este no lo hacia.
    #    Se levanta con las mismas listas para que la introspeccion no revele lo
    #    que la lista blanca esconde.
    introspeccion = Node(
        package='rosapi', executable='rosapi_node',
        name='rosapi', namespace=ns, output='screen',
        condition=IfCondition(LaunchConfiguration('rosbridge')),
        parameters=[{
            'topics_glob': _glob(LEER + ESCRIBIR),
            'services_glob': _glob(SERVICIOS),
            'params_glob': '[]',
        }],
    )

    # ── El supervisor de SLAM y Nav2 ─────────────────────────────────────────
    # 🔴 VA APARTE DEL DRIVER, Y NO ES ELEGANCIA: es el unico proceso que llama a
    #    `systemctl start|stop`, o sea el unico que necesita la regla de polkit.
    #    Metido en el driver, esa capacidad se la quedaria el proceso que YA
    #    tiene el puerto serie, los motores y la parada de emergencia.
    #
    # 📝 Se arranca AQUI, con el driver, y no desde las unidades de SLAM/Nav2:
    #    tiene que estar vivo PRECISAMENTE CUANDO ESTAN APAGADAS, para poder
    #    decir «apagado». Un supervisor que solo existe cuando existe lo
    #    supervisado no supervisa nada.
    #
    # ⚠️ Hoy `atriz-slam.service` NO existe y la regla de polkit NO esta puesta.
    #    El nodo lo dice con todas las letras en `/estado_navegacion` en vez de
    #    fingir: estado DESCONOCIDO con el motivo. Verificado el 2026-08-07.
    supervisor = Node(
        package='atriz_rvr_driver', executable='supervisor_navegacion',
        name='supervisor_navegacion', namespace=ns, output='screen',
        parameters=[{
            'unidad_slam': 'atriz-slam.service',
            'unidad_nav': 'atriz-nav.service',
            # El mismo defecto que usa `atriz-nav.sh`, para que los dos miren el
            # mismo sitio. Si no coinciden, el supervisor diria «hay mapa» y la
            # unidad fallaria por no encontrarlo.
            'mapa': PathJoinSubstitution([bringup, 'maps', 'aula.yaml']),
        }],
    )

    return LaunchDescription([
        arg_lidar, arg_ns, arg_puerto, arg_keepalive, arg_silencio, arg_seguridad,
        arg_inclinacion, arg_color, arg_color_inact, arg_color_max, arg_rosbridge,
        rvr, desc, lidar, monitor, gestor_seguridad, supervisor, puente,
        introspeccion,
    ])
