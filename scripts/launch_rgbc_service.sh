#!/bin/bash

# Script para lanzar el servicio RGBC que permite mediciones de color sin activar el LED

echo "🎨 Lanzando Servicio RGBC (Color Sin LED)"
echo "========================================"

# Verificar si ROS está ejecutándose
if ! pgrep -x "rosmaster" > /dev/null; then
    echo "❌ ROS Master no está ejecutándose"
    echo "💡 Ejecuta: roscore"
    exit 1
fi

# Verificar si el driver del RVR está ejecutándose
if ! pgrep -f "Atriz_rvr_node.py" > /dev/null; then
    echo "⚠️ Driver del RVR no detectado"
    echo "💡 Asegúrate de que el driver esté ejecutándose"
fi

echo "🚀 Iniciando servicio RGBC..."
echo "📡 Servicio disponible en: /get_rgbc_sensor_values"
echo "🛑 Presiona Ctrl+C para detener"
echo ""

# Lanzar el servicio
python3 /home/sphero/atriz_git/src/ros_sphero_rvr/scripts/rgbc_sensor_service.py
