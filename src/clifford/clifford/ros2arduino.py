import rclpy
import serial 
import math
import time

from std_msgs.msg import String
from sensor_msgs.msg import JointState 
from rclpy.node import Node

class ros2arduino(Node):
    def __init__(self):
        super().__init__("transform_angles_ros2arduino")
        self.position_subscriber = self.create_subscription(
            JointState, 
            '/joint_states', 
            self.joint_states_callback, 
            10
        )

        # Serial port's information
        self.microcontroller_port = "/dev/ttyUSB0"
        self.microcontroller_baudrate = 115200
        self.serial_port = serial.Serial(
            self.microcontroller_port, 
            self.microcontroller_baudrate, 
            timeout=10
        )
        self.mapped_msg = String()

    def map_position_into_bits(self, value, in_min, in_max, out_min, out_max):
        mapped_data_list = (value - in_min) * (out_max - out_min) / ((in_max - in_min) + out_min)
        return round(mapped_data_list)

    def write_into_serial_port(self, mapped_data_list):
        data_out = f"[{','.join(map(str, mapped_data_list))}]"
        self.get_logger().info(f"Enviando: {data_out}")
        
        # Enviar al puerto serial solo si los valores cumplen con las restricciones
        self.serial_port.write(data_out.encode())
    
    def joint_states_callback(self, joint_states_msg):
        print("JointState received!")
        
        def filter_angle(q, valid_angles, tol=0.1):
            return q if any(abs(q - v) <= tol for v in valid_angles) else None
        
        valid_q2_angles = {14.25, 18.15, 20.05}
        valid_q3_angles = {19.79, 18.87, 6.98}
        
        joint_positions = [round(math.degrees(angle), 2) for angle in joint_states_msg.position]
        self.get_logger().info(f"Ángulos en grados: {joint_positions}")
        
        filtered_data = []
        temp_group = []
        
        for i, angle in enumerate(joint_positions):
            if i % 3 == 1:  # q2
                filtered_angle = filter_angle(angle, valid_q2_angles)
                temp_group.append(filtered_angle if filtered_angle is not None else "X")
            elif i % 3 == 2:  # q3
                filtered_angle = filter_angle(angle, valid_q3_angles)
                temp_group.append(filtered_angle if filtered_angle is not None else "X")
            else:  # q1 se mantiene
                temp_group.append(angle)
            
            if len(temp_group) == 3:  # Solo enviar conjuntos completos de q1, q2, q3
                if "X" not in temp_group:
                    filtered_data.extend(temp_group)
                temp_group = []
        
        if filtered_data:  # Solo enviamos si hay datos válidos
            self.write_into_serial_port(filtered_data)
        
        self.get_logger().info(f"Ángulos filtrados enviados: {filtered_data}")


def main(args=None):
    rclpy.init(args=args)
    node = ros2arduino()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
