#!/usr/bin/env python3

# Ejemplo básico de servicio ROS para Sphero RVR
import rospy
from std_srvs.srv import SetBool, SetBoolResponse

def emergency_stop_callback(req):
    """Callback que maneja las peticiones de parada de emergencia"""
    
    rospy.loginfo("Solicitud de parada de emergencia recibida: %s" % req.data)
    
    if req.data:
        # Activar parada de emergencia
        rospy.logwarn("¡PARADA DE EMERGENCIA ACTIVADA!")
        rospy.logwarn("Deteniendo todos los movimientos del robot")
        
        # Aquí iría la lógica real para detener el robot
        # Por ejemplo: publicar velocidad cero en /cmd_vel
        
        response = SetBoolResponse()
        response.success = True
        response.message = "Parada de emergencia activada correctamente"
        
    else:
        # Desactivar parada de emergencia
        rospy.loginfo("Parada de emergencia desactivada")
        rospy.loginfo("Robot listo para operación normal")
        
        response = SetBoolResponse()
        response.success = True
        response.message = "Parada de emergencia desactivada"
    
    return response

def emergency_stop_service():
    """Nodo que proporciona servicio de parada de emergencia"""
    
    # Inicializar el nodo
    rospy.init_node('emergency_stop_service', anonymous=True)
    
    # Crear servicio
    service = rospy.Service('/emergency_stop', SetBool, emergency_stop_callback)
    
    rospy.loginfo("Servicio de parada de emergencia iniciado")
    rospy.loginfo("Servicio disponible en: /emergency_stop")
    rospy.loginfo("Uso: rosservice call /emergency_stop 'data: true'  # Activar")
    rospy.loginfo("     rosservice call /emergency_stop 'data: false' # Desactivar")
    
    # Mantener el nodo activo
    rospy.spin()

if __name__ == '__main__':
    try:
        emergency_stop_service()
    except rospy.ROSInterruptException:
        rospy.loginfo("Nodo interrumpido por el usuario")
