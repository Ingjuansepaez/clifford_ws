import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from custom_interfaces.srv import TrajectoryPoint
import numpy as np

class TrayectoriaServicio(Node):
    def __init__(self):
        super().__init__("trajectory_service")
        
        # Parámetros de la elipse para las patas delanteras
        self.a_front = -0.020
        self.h_front = 0.010
        self.z0_front = -0.162
        
        # Parámetros de la elipse para las patas traseras
        self.a_back = -0.02
        self.h_back = 0.01
        self.z0_back = -0.162
        
        self.num_puntos = 30
        
        # Generar trayectoria para patas delanteras
        self.t_vals = np.linspace(0, 30, self.num_puntos)
        self.x_trayectoria_front = np.round(self.a_front * (self.t_vals / 15), 3)
        self.z_trayectoria_front = np.round(self.z0_front + self.h_front * np.sin(np.pi * (self.t_vals / 15)), 3)
        
        # Generar trayectoria para patas traseras
        self.x_trayectoria_back = np.round(self.a_back * (self.t_vals / 15), 3)
        self.z_trayectoria_back = np.round(self.z0_back + self.h_back * np.sin(np.pi * (self.t_vals / 15)), 3)
        
        # Crear servicio
        self.srv = self.create_service(TrajectoryPoint, 'get_trajectory_point', self.handle_request)
    
    def handle_request(self, request, response):
        punto = Point()
            
        if request.index < self.num_puntos:
            if request.leg_type == "front":  # Para patas delanteras
                punto.x = self.x_trayectoria_front[request.index]
                punto.z = self.z_trayectoria_front[request.index]
            elif request.leg_type == "back":  # Para patas traseras
                punto.x = self.x_trayectoria_back[request.index]
                punto.z = self.z_trayectoria_back[request.index]
            else:
                self.get_logger().warning("Tipo de pata no válido. Usando trayectoria delantera por defecto.")
                punto.x = self.x_trayectoria_front[request.index]
                punto.z = self.z_trayectoria_front[request.index]
        else:
            self.get_logger().warning("Índice fuera de rango")
        
        punto.y = 0.0 
        response.point = punto
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TrayectoriaServicio()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
