#!/usr/bin/env python3

"""
Script de prueba corregido para las funcionalidades del Sphero RVR.
"""

import rospy
import time
from sphero_rvr_msgs.srv import GetEncoders, RawMotors, GetSystemInfo, GetControlState

def test_corrected_functionalities():
    """Prueba las funcionalidades corregidas."""
    rospy.init_node('test_corrected', anonymous=True)
    rospy.loginfo("🚀 Iniciando pruebas corregidas...")
    
    # Esperar a que los servicios estén disponibles
    rospy.loginfo("Esperando servicios...")
    rospy.sleep(3)
    
    # Probar encoders
    try:
        rospy.loginfo("Probando encoders...")
        get_enc = rospy.ServiceProxy('/get_encoders', GetEncoders)
        response = get_enc()
        if response.success:
            rospy.loginfo(f"✓ Encoders: Izq={response.left_wheel_count}, Der={response.right_wheel_count}")
        else:
            rospy.logwarn(f"✗ Encoders: {response.message}")
    except Exception as e:
        rospy.logerr(f"✗ Error encoders: {e}")
    
    # Probar motores raw
    try:
        rospy.loginfo("Probando motores raw...")
        raw_motors = rospy.ServiceProxy('/raw_motors', RawMotors)
        response = raw_motors(left_mode=1, left_speed=50, right_mode=1, right_speed=50)
        if response.success:
            rospy.loginfo("✓ Motores raw: Movimiento iniciado")
            time.sleep(1)
            # Detener
            raw_motors(left_mode=0, left_speed=0, right_mode=0, right_speed=0)
            rospy.loginfo("✓ Motores raw: Detenidos")
        else:
            rospy.logwarn(f"✗ Motores raw: {response.message}")
    except Exception as e:
        rospy.logerr(f"✗ Error motores raw: {e}")
    
    # Probar información del sistema
    try:
        rospy.loginfo("Probando información del sistema...")
        get_sys = rospy.ServiceProxy('/get_system_info', GetSystemInfo)
        response = get_sys()
        if response.success:
            info = response.system_info
            rospy.loginfo(f"✓ Sistema: App {info.app_major}.{info.app_minor}.{info.app_revision}")
            rospy.loginfo(f"✓ Sistema: MAC {info.mac_address}")
        else:
            rospy.logwarn(f"✗ Sistema: {response.message}")
    except Exception as e:
        rospy.logerr(f"✗ Error sistema: {e}")
    
    # Probar estado de control
    try:
        rospy.loginfo("Probando estado de control...")
        get_control = rospy.ServiceProxy('/get_control_state', GetControlState)
        response = get_control()
        if response.success:
            state = response.control_state
            rospy.loginfo(f"✓ Control: ID={state.active_controller_id}, Detenido={state.is_stopped}")
        else:
            rospy.logwarn(f"✗ Control: {response.message}")
    except Exception as e:
        rospy.logerr(f"✗ Error control: {e}")
    
    rospy.loginfo("🎉 Pruebas corregidas completadas!")

if __name__ == '__main__':
    test_corrected_functionalities()
