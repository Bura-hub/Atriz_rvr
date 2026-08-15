# El agente de sesión — lo que ejecuta el código del alumno

Es la mitad que corre **en el robot** del Taller de `atriz-lab`: la pantalla
donde un alumno escribe su programa y lo ejecuta desde el navegador, sin SSH.

> 🔴🔴 **NADA DE ESTE DIRECTORIO SE HA EJECUTADO EN UN ROBOT.** Escrito el
> 2026-08-14 desde el PC. Lo que **sí** está probado es `agente_nucleo.py`
> (31 pruebas, que corren en Windows). El PTY tiene 13 pruebas que **se saltan**
> donde no hay POSIX, y eso **no es que pasen**.

Diseño completo, con lo que se descartó y por qué:
`Atriz_migracion_ros2 → docs/superpowers/specs/2026-08-14-el-taller-terminal-del-alumno-design.md`

---

## Los ficheros, y por qué están separados

| fichero | qué | dónde se prueba |
|---|---|---|
| `agente_nucleo.py` | **Lo que decide**: la ranura, los nombres, el tope, los cuatro peldaños de la parada, el troceado de salida | **En cualquier sitio**, incluido Windows |
| `agente_pty.py` | `pty.fork()`, `setsid`, señales al grupo, cosechar | **Cualquier Linux**, sin RVR |
| `agente_sesion.py` | tornado. El pegamento, deliberadamente delgado | **Solo en el robot** |
| `atriz-agente.service` · `.sh` | La unidad y su envoltorio | Solo en el robot |

**La separación es el patrón de `atriz_testigo.py`**, y paga por lo mismo: los
caminos que importan son los **negativos** —op desconocida, ranura ocupada, no
eres el dueño, Ctrl-C encubierto en la entrada— y todos viven donde se pueden
recorrer sin un robot delante.

---

## Probarlo

```bash
# Todo lo que se puede probar sin robot. En Linux corre TAMBIÉN el PTY.
python3 -m pytest scripts/agente/pruebas/ -q
```

⚠️ En Windows salen **13 `skipped`**: son las del PTY, y con ellas quedan sin
medir los dos requisitos que justifican todo este diseño —que `print()` salga en
vivo y que `input()` espere—. Cada una lleva **su control contra una tubería**:
sin el control, «funciona con PTY» no distinguiría que el PTY lo arregle de que
funcionara igual.

---

## Instalarlo (👤 lleva `sudo`)

```bash
sudo install -m 755 atriz-agente.sh /usr/local/bin/
sudo install -m 644 atriz-agente.service /etc/systemd/system/
sudo install -d -m 755 /etc/atriz
# La clave PÚBLICA del servidor de la web. Se saca en el PC, dentro de atriz-lab:
#   node herramientas/publicar_clave.mjs
sudo tee /etc/atriz/testigo.pub   # y se pega
sudo systemctl daemon-reload && sudo systemctl enable --now atriz-agente
```

### 🔴 Antes: quita `~/.git-credentials` de este robot

El programa del alumno corre **como `sphero`**, así que puede leer lo que
`sphero` lea — y ahí está el PAT de GitHub del proyecto. Los repositorios ya son
públicos: **clonar no lo necesita**. Subir sí, y eso se hace desde el PC.

---

## 🔴 Lo que este agente NO es

**No es una frontera de seguridad.** El alumno corre como `sphero`, igual que el
driver: puede escribir donde `sphero` escriba, abrir sus propios sockets, e
`import rclpy` para alcanzar `raw_motors` y `set_ir_mode('following')` —
**saltándose el `collision_monitor`**, que es justo lo que la lista blanca de
rosbridge cierra para el navegador.

Lo que protege son los **errores honestos**: el guion que se cuelga, el que llena
la pantalla, el que olvida apagar el barrido. Contra alguien hostil no protege, y
eso va escrito también en la pantalla del alumno.

---

## Dos trampas que ya están resueltas aquí, y conviene no deshacer

**`RuntimeDirectoryPreserve=yes` no es opcional.** `atriz-robot.service` declara
el **mismo** `RuntimeDirectory=atriz`, y dentro vive la marca del vigía de DDS que
garantiza «una sola cura por arranque». Sin el `Preserve`, parar el agente borra
`/run/atriz` y esa marca con él: el robot se reiniciaría solo más de una vez por
arranque, en mitad de una clase. **`systemd-analyze verify` no lo ve**, y leer
cualquiera de las dos unidades por separado tampoco.

**`pkill -f` no aparece en ninguna parte.** El PID se conoce por haberlo
engendrado, y se señala al **grupo** con `os.killpg`. Este proyecto tiene dos
incidentes documentados en los que `pkill -f` mató la propia terminal que lo
ejecutaba.
