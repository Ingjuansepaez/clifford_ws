import numpy as np

import rclpy
import math

from rclpy.node import Node

class InverseKinematic(Node):
    def __init__(self):
        super().__init__("cinematica_inversa")

    def compute_inverse_kinematic(self, L_list, desired_point):
        L0 = L_list[0]
        L1 = L_list[1]
        L2 = L_list[2]
        # desired_x = desired_htm[0, 3]
        # print(f"\nDesired X: {desired_x}")
        # desired_y = desired_htm[1, 3]
        # print(f"Desired Y: {desired_y}")
        # desired_z = desired_htm[2, 3]
        # print(f"Desired Z: {desired_z}"
        desired_x = desired_point[0]
        #print(f"\nDesired X: {desired_x}")
        desired_y = desired_point[1]
        #print(f"Desired Y: {desired_y}")
        desired_z = desired_point[2]
        #print(f"Desired Z: {desired_z}"
        q1 = math.atan2(desired_z, desired_y)+math.radians(90)
        #print(f"\nQ1: {math.degrees(q1)} °")
        cos_q1 = math.cos(q1)
        sin_q1 = math.sin(q1)
        cc=math.sqrt(desired_x**2+(math.fabs(desired_y)-math.fabs(L0*sin_q1))**2+(math.fabs(desired_z)-math.fabs(L0*cos_q1))**2)-0.00000001
        #if cos_q1<=1.0e-19:
        #    cos_q1=1.0e-10 
        #print(f"\ncc: {cc}")
        cos_a=(L1**2+cc**2-L2**2)/(2*L1*cc)
        #print(f"\ncos_a: {cos_a} °")
        if cos_a >= 1:
            cos_a = 1
        elif cos_a <= -1:
            cos_a = -1
        A = math.acos(cos_a)
        q2 = math.asin(-desired_x/cc)-A
        cos_q3=(L1**2+L2**2-cc**2)/(2*L1*L2)
        if cos_q3>1:
            cos_q3=1
        elif cos_q3<-1:
            cos_q3=-1
        q3 = -math.acos(cos_q3)+math.radians(180)
        q4 = 0.0
        q_list = np.array([q1, q2, q3, q4]).T

        self.get_logger().info(f"VALOR DE LOS ANGULOS RESULTANTES \n Q1 = {q_list[0]}, \n Q2 = {q_list[1]}, \n Q3 = {q_list[2]}, \n Q4 = {q_list[3]},")
        
        return q_list

def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematic()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()