# Parches a código de terceros

`provision.sh` clona el driver del YDLIDAR desde GitHub **y le borra el `.git`**, porque es
código de terceros y no se mezcla con este repositorio. Eso tiene una consecuencia:

🔴 **Un cambio hecho a mano en `~/atriz_ws/src/ydlidar_ros2_driver` se pierde al reflashear, y
`provision.sh` no lo reproduce.** El robot arreglado y un robot recién aprovisionado
divergirían — y la regla del proyecto dice que **gana el script**.

Por eso los parches viven aquí, versionados, y `provision.sh` los aplica tras clonar.

| Parche | Qué arregla |
|---|---|
| `ydlidar-no-inundar-journal.patch` | El nodo emitía `Failed to get scan` **25 veces por segundo** con el barrido apagado — que es el estado normal en reposo. Ver abajo. |

---

## `ydlidar-no-inundar-journal.patch`

**El problema.** `/stop_scan` y `/start_scan` son servicios del propio nodo del YDLIDAR, y
llaman a `laser.turnOff()` / `turnOn()`. Pero **nadie guarda ese estado**: el bucle principal
sigue llamando a `doProcessSimple()` 20 veces por segundo, falla siempre, y emite un
`RCLCPP_ERROR` cada vez.

**Medido en rvr-01 el 2026-08-01**, con el barrido apagado:

```
502 errores en 20 s = 25 por segundo
47 291 de 47 551 líneas del journal del servicio = el 99 %
2.17 millones de mensajes al día por robot · 34 millones entre los 16
```

**Por qué importa, y no es que «moleste»:**

1. **Ahoga cualquier error de verdad.** Los peores fallos de este proyecto están documentados
   como silenciosos, y el journal es donde se buscan.
2. **Desgasta la microSD.** Las tarjetas mueren por escrituras, y es el único almacenamiento
   que tiene el robot.
3. **Sondea el puerto serie 20 veces por segundo para nada.**

**El arreglo** son nueve líneas: una bandera `std::atomic<bool> escaneando`, que los dos
servicios actualizan, y una salida temprana en el bucle. Con el barrido parado **no se toca el
hardware y no se escribe en el log**, pero se sigue atendiendo a ROS — los servicios tienen que
responder para poder volver a encenderlo.

✅ **Verificado en rvr-01** tras aplicarlo: 0 errores en 20 s con el barrido apagado (eran 502),
`atriz-escaneo on` devuelve `/scan` a **12.00 Hz con 250 puntos**, y `atriz-escaneo off` lo para
del todo (0 mensajes) **sin volver a generar ruido**.

📝 **No se manda aguas arriba** de momento: el proyecto usa la rama `humble` de YDLIDAR tal
cual, y mantener un fork abierto cuesta más que un parche de nueve líneas. Si algún día el
upstream lo arregla, este parche fallará al aplicarse y `provision.sh` lo dirá — que es
justamente lo que queremos que pase.
