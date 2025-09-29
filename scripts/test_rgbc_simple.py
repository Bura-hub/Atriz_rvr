#!/usr/bin/env python3

import rospy
from atriz_rvr_msgs.srv import GetRGBCSensorValues

def test_rgbc_service():
    """Prueba simple del servicio RGBC."""
    rospy.init_node('test_rgbc_simple', anonymous=True)
    
    try:
        # Esperar servicio
        rospy.loginfo("🔍 Esperando servicio /get_rgbc_sensor_values...")
        rospy.wait_for_service('/get_rgbc_sensor_values', timeout=10.0)
        
        # Crear proxy del servicio
        rgbc_service = rospy.ServiceProxy('/get_rgbc_sensor_values', GetRGBCSensorValues)
        
        # Hacer solicitud
        rospy.loginfo("📡 Solicitando datos RGBC...")
        response = rgbc_service()
        
        if response.success:
            rospy.loginfo("✅ ¡ÉXITO! Datos obtenidos sin activar LED")
            rospy.loginfo(f"   Red:   {response.red_channel_value}")
            rospy.loginfo(f"   Green: {response.green_channel_value}")
            rospy.loginfo(f"   Blue:  {response.blue_channel_value}")
            rospy.loginfo(f"   Clear: {response.clear_channel_value}")
            rospy.loginfo("🎉 ¡Objetivo cumplido! Medición de color sin LED")
        else:
            rospy.logwarn(f"⚠️ Error: {response.message}")
            
    except rospy.ROSException as e:
        rospy.logerr(f"❌ Error: {e}")
        rospy.logwarn("💡 Asegúrate de que el servicio rgbc_sensor_service.py esté ejecutándose")

if __name__ == '__main__':
    test_rgbc_service()
