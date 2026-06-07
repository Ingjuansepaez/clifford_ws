import rclpy
import numpy as np
import pandas as pd
import time
import tf2_ros

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from rclpy.node import Node
from clifford.numeric_method import nodeNumericMethod
from custom_interfaces.srv import TrajectoryPoint

class QuadrupedLocomotion(Node):
    def __init__(self):
        super().__init__("quadruped_locomotion")

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.torso_x = 0.0  # Posición del torso en X
        self.torso_z = 0.0  # Elevación en Z
        self.moving_torso = False
        self.apply_torso_offset = True  # 🔥 Esta bandera decide si sumamos torso_x o no
        self.phase = 1

        # Parámetros generales
        self.px = 0.0
        self.py = 0.0
        self.pz = 0.0

        self.q_pata_da = [0.0, 0.0, 0.0, 0.0]
        self.q_pata_ia = [0.0, 0.0, 0.0, 0.0]
        self.q_pata_dt = [0.0, 0.0, 0.0, 0.0]
        self.q_pata_it = [0.0, 0.0, 0.0, 0.0]

        self.joint_names = [
            "hombro_DA_joint", "brazo_DA_joint", "muneca_DA_joint", "end_effector_DA_joint",
            "hombro_IA_joint", "brazo_IA_joint", "muneca_IA_joint", "end_effector_IA_joint",
            "hombro_DT_joint", "brazo_DT_joint", "muneca_DT_joint", "end_effector_DT_joint",
            "hombro_IT_joint", "brazo_IT_joint", "muneca_IT_joint", "end_effector_IT_joint"
        ]

        self.q_torso = {"brazo_DA_joint": 0.0, "brazo_IA_joint": 0.0, "brazo_DT_joint": 0.0, "brazo_IT_joint": 0.0}
        
        self.l_list = [0.052, 0.041, 0.090] #Eslabones para el uso de la geometrica

        self.current_leg = 0 #Numero para indicar la pata a mover
        self.num_points = 10 #Numero de puntos de la elipse
        self.reverse_phase = False #Bandera para la fase de retorno

        # Parámetros de la elipse
        self.a = -0.010
        self.h = 0.010
        self.z0 = -0.162

        # Publisher de /joint_states
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)

        # Método numérico
        self.numeric_method = nodeNumericMethod()

        # Cliente para el servicio de trayectoria
        self.cli = self.create_client(TrajectoryPoint, 'get_trajectory_point')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando al servicio "get_trajectory_point"...')
        self.get_logger().info('Servicio "get_trajectory_point" conectado.')

        # Índice para la trayectoria
        self.index = 0
        # Almacén de datos para exportar a Excel
        self.data_log = []

    def selec_move_leg(self, current_leg):
        leg_type = "front" if current_leg in [2, 3] else "back"
        #self.get_logger().info(f"Solicitando trayectoria para {leg_type} en la pata {current_leg}")
        punto = self.send_request(self.index, leg_type)

        #self.px = punto.x + self.torso_x
        #self.get_logger().info(f"Valor torso_x = {self.torso_x}")
        #self.get_logger().info(f"Punto recibido - X: {punto.x}, Y: {punto.y}, Z: {punto.z}")

        self.px = punto.x
        self.py = punto.y
        self.pz = punto.z

        #self.get_logger().info(f"Valor en la posicion X = {self.px}")

        desired_point = [self.px, self.py, self.pz, 0.0]
        self.q_list = self.numeric_method.numeric_method(desired_point)

        self.get_logger().info(f"VALOR DE LOS ANGULOS = {np.degrees(self.q_list)}")

        self.value_in_joint = [*self.q_pata_da, 
                              *self.q_pata_ia,
                              *self.q_pata_dt,
                              *self.q_pata_it]

        if current_leg == 0:
            #Aqui se debe escoger la pata_IT
            self.q_pata_it = -1*(self.q_list)
            self.move_leg()
        
        elif current_leg == 1:
            #Aqui se debe escoger la pata_DT
            self.q_pata_dt = self.q_list
            self.move_leg()
            
        elif current_leg == 2:
            #Aqui se debe escoger la pata_DA
            self.q_pata_da = self.q_list
            self.move_leg()
            
        elif current_leg == 3:
            #Aqui se debe escoger la pata_IA
            self.q_pata_ia = -1*(self.q_list)
            self.move_leg()
            
        self.log_data(leg_type)

    def move_leg(self):
        joint_states = JointState()
        joint_states.header.stamp = self.get_clock().now().to_msg()
        joint_states.name = self.joint_names
        joint_states.position = self.value_in_joint
        self.publisher_.publish(joint_states)
        self.broadcast_transform()

    def move_torso(self):
       
        avance = 0.001  
        
        if self.torso_x + avance >= 0.02:
            self.torso_x = 0.02
        else:
            self.torso_x += avance

        self.broadcast_transform()

    def broadcast_transform(self):
    
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_footprint"  
        t.child_frame_id = "base_link"  

        # Posición del base_link en el mundo
        t.transform.translation.x = self.torso_x
        t.transform.translation.y = 0.0
        t.transform.translation.z = self.torso_z

        # Sin rotaciones (mantener orientación)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # Publicar en tf
        self.tf_broadcaster.sendTransform(t)

    def log_data(self, leg_type):
        q_list_degrees = np.degrees(self.q_list) 
        self.data_log.append([self.index, leg_type, self.torso_x, self.px, self.py, self.pz, *q_list_degrees])

    def export_to_excel(self):
        df = pd.DataFrame(self.data_log, columns=["Index", "Leg Type", "Torso X", "PX", "PY", "PZ", "Theta1 (°)", "Theta2 (°)", "Theta3 (°)", "Theta4 (°)"])

        df_front = df[df["Leg Type"] == "front"]
        df_back = df[df["Leg Type"] == "back"]

        df_front.to_excel("locomotion_front.xlsx", index=False)
        df_back.to_excel("locomotion_back.xlsx", index=False)

        #self.get_logger().info("📊 Datos exportados a locomotion_front.xlsx y locomotion_back.xlsx")

        
    def send_request(self, index, leg_type):
        req = TrajectoryPoint.Request()
        req.index = index
        req.leg_type = leg_type

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            return future.result().point
        else:
            self.get_logger().error('Error al llamar al servicio')
            return None


    def execute_locomotion(self):
    
        try:
            while rclpy.ok():
                match self.moving_torso:
                    case False:
                        # 🔥 Fase de movimiento de patas
                        if self.index < self.num_points:
                            self.selec_move_leg(self.current_leg)
                            self.index += 1
                        else:
                            # 🔥 Pasar a la siguiente pata
                            self.index = 0  # Reiniciar índice de trayectoria
                            if self.current_leg == 3:
                                self.moving_torso = True  # Cambiar a modo de mover el torso
                                self.apply_torso_offset = True  # 🔥 Activar la suma de torso_x
                                self.current_leg = 0
                            else:
                                self.current_leg += 1

                    case True:
                        # 🔥 Fase de movimiento del torso (una sola vez)
                        self.move_torso()
                        self.moving_torso = False  # Volver al modo de patas
                        self.apply_torso_offset = False  # 🔥 Activar la suma de torso_x
                        self.index = 0  # Reiniciar índice para la nueva trayectoria de patas
                        self.current_leg = 0

                time.sleep(0.01)
        except Exception as e:
            self.get_logger().error(f'Error en la locomoción: {str(e)}')
        
                

def main(args=None):
    rclpy.init(args=args)
    node = QuadrupedLocomotion()
    try:
        node.execute_locomotion()
    except KeyboardInterrupt:
        node.export_to_excel()
        node.get_logger().info('Locomoción interrumpida por el usuario.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()