#!/usr/bin/env python3
"""
Configuración del SDK de Sphero para resolver importaciones.
Este archivo configura las rutas necesarias para que el SDK funcione correctamente.
"""

import sys
import os

def setup_sphero_sdk_path():
    """
    Configura la ruta del SDK de Sphero en el PATH de Python.
    """
    # Obtener el directorio actual del script
    current_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Construir la ruta al SDK de Sphero
    sdk_path = os.path.join(current_dir, 'sphero_sdk')
    sdk_path = os.path.abspath(sdk_path)
    
    # Verificar que la ruta existe
    if not os.path.exists(sdk_path):
        raise ImportError(f"SDK path not found: {sdk_path}")
    
    # Agregar la ruta al PATH de Python si no está ya presente
    if sdk_path not in sys.path:
        sys.path.insert(0, sdk_path)
    
    return sdk_path

# Configurar la ruta automáticamente al importar este módulo
try:
    SDK_PATH = setup_sphero_sdk_path()
except ImportError as e:
    print(f"Warning: Could not setup Sphero SDK path: {e}")
    SDK_PATH = None
