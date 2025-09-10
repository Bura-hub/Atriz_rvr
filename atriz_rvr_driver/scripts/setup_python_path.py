#!/usr/bin/env python3
"""
Script para configurar el PATH de Python para el proyecto Atriz RVR.
Ejecutar este script antes de usar cualquier script del proyecto.
"""

import sys
import os

def setup_python_path():
    """
    Configura el PATH de Python para incluir todas las rutas necesarias.
    """
    # Obtener el directorio raíz del proyecto
    project_root = os.path.dirname(os.path.abspath(__file__))
    
    # Rutas a agregar
    paths_to_add = [
        os.path.join(project_root, 'atriz_rvr_driver', 'scripts'),
        os.path.join(project_root, 'scripts', 'core'),
        os.path.join(project_root, 'scripts', 'examples'),
        os.path.join(project_root, 'scripts', 'tools'),
        os.path.join(project_root, 'scripts', 'utilities'),
        os.path.join(project_root, 'testing_scripts', 'automated'),
        os.path.join(project_root, 'testing_scripts', 'interactive'),
        os.path.join(project_root, 'testing_scripts', 'diagnostic')
    ]
    
    # Agregar rutas al PATH si no están ya presentes
    for path in paths_to_add:
        if os.path.exists(path) and path not in sys.path:
            sys.path.insert(0, path)
            print(f"Added to Python path: {path}")
    
    print(f"Python path configured. Total paths: {len(sys.path)}")
    return True

if __name__ == "__main__":
    setup_python_path()
