#!/usr/bin/env bash
# Envoltorio del agente de sesión. Mismo patrón que `atriz-robot.sh`.
#
#   sudo cp atriz-agente.sh /usr/local/bin/ && sudo chmod +x /usr/local/bin/atriz-agente.sh
#
# 🔴🔴 NO ESTÁ PROBADO. Escrito desde el PC el 2026-08-14, sin robot.

set -euo pipefail

# ═══════════════════════════════════════════════════════════════════════════
# 🔴 LOS `setup.bash` DE ROS NO SON COMPATIBLES CON `set -u`
# ═══════════════════════════════════════════════════════════════════════════
# `AMENT_TRACE_SETUP_FILES: unbound variable`, y el mensaje no menciona ROS. Este
# proyecto ya lo pagó dos veces: se arregló en `atriz-robot.sh` y NO en
# `atriz-escaneo.sh`, y en el primer arranque real bajo systemd el barrido del
# LIDAR se quedó encendido. Aquí va desde el primer día.
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
[[ -f "$HOME/atriz_ws/install/setup.bash" ]] && source "$HOME/atriz_ws/install/setup.bash"
# La identidad del robot: ROS_DOMAIN_ID y ATRIZ_ROBOT_ID.
# shellcheck disable=SC1091
[[ -f /etc/profile.d/atriz-robot.sh ]] && source /etc/profile.d/atriz-robot.sh
set -u

# ═══════════════════════════════════════════════════════════════════════════
# El verificador del testigo vive en el OTRO repositorio
# ═══════════════════════════════════════════════════════════════════════════
# Se importa en vez de copiarse: dos copias de un verificador de firmas se
# separan en silencio, y el día que pase el síntoma sería «el terminal no abre en
# ese robot», buscado en el sitio equivocado.
MIGRACION="${ATRIZ_MIGRACION:-$HOME/atriz_migracion}"
if [[ ! -f "$MIGRACION/scripts/atriz_testigo.py" ]]; then
  echo "🔴 no encuentro $MIGRACION/scripts/atriz_testigo.py" >&2
  echo "   El agente lo necesita para verificar los testigos de la web." >&2
  echo "   Clona el repositorio de migración, o pon ATRIZ_MIGRACION." >&2
  exit 1
fi
export PYTHONPATH="$MIGRACION/scripts:${PYTHONPATH:-}"

# ═══════════════════════════════════════════════════════════════════════════
# La identidad del robot
# ═══════════════════════════════════════════════════════════════════════════
# 🔴 Sin número no se arranca. El testigo lleva `rob` dentro y el agente lo
#    compara con el suyo: con el número equivocado rechazaría a TODO EL MUNDO, y
#    con uno inventado dejaría entrar a quien no toca.
ID="${ATRIZ_ROBOT_ID:-}"
if [[ -z "$ID" ]]; then
  echo "🔴 ATRIZ_ROBOT_ID no está puesto." >&2
  echo "   Lo pone /etc/profile.d/atriz-robot.sh, que instala fase_7_systemd.sh --id NN." >&2
  exit 1
fi

CLAVE="${ATRIZ_CLAVE_PUB:-/etc/atriz/testigo.pub}"
if [[ ! -f "$CLAVE" ]]; then
  echo "🔴 no está la clave pública en $CLAVE." >&2
  echo "   Es la mitad pública de la que firma el servidor de la web. Se saca con:" >&2
  echo "     node herramientas/publicar_clave.mjs   (en el PC, dentro de atriz-lab)" >&2
  exit 1
fi

AQUI="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AGENTE="${ATRIZ_AGENTE_PY:-$HOME/atriz_ws/src/Atriz_rvr/scripts/agente/agente_sesion.py}"
[[ -f "$AGENTE" ]] || AGENTE="$AQUI/agente_sesion.py"

# `exec` para que el PID principal de la unidad sea el agente: así el `KillSignal`
# de systemd le llega a él y puede aplicar sus cuatro peldaños al hijo.
exec python3 "$AGENTE" --robot "$((10#$ID))" --clave "$CLAVE"
