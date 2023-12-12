#!/usr/bin/env python3
#Librerias de ROS2
from typing import List
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
#Librerias de Python
import math
import time

class AnglePosition(Node):
    def __init__(self, q1, q2, q3):
        super().__init__('inverse_kinematic_publisher')

        self.joint1_names = [
            "front_left_wrist_joint",
            "front_left_leg_joint",
            "front_left_foot_joint",
            "back_left_wrist_joint",
            "back_left_leg_joint",
            "back_left_foot_joint",
            "back_right_wrist_joint",
            "back_right_leg_joint",
            "back_right_foot_joint",
            "front_right_wrist_joint",
            "front_right_leg_joint",
            "front_right_foot_joint"
        ]

        self.q1 = 0.0
        self.q2 = q2
        self.q3 = q3

        self.q1_1 = q1
        self.q2_2 = q2
        self.q3_3 = q3

        q1_1 = self.q1_1
        q2_2 = self.q2_2
        q3_3 = self.q3_3

        self.joint1_values = [
            q1,
            q2, 
            q3,
            q1,
            q2,
            q3,
            q1_1,
            q2_2, 
            q3_3,
            q1_1,
            q2_2, 
            q3_3
        ]

        self.angle_publisher = self.create_publisher(
            JointState, 
            'joint_states', 10
        )

        self.timer = self.create_timer(0.01, 
                                       self.publish_kinematic_joint)
        
    def publish_kinematic_joint(self):

        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint1_names
        joint_state.position = self.joint1_values
        self.angle_publisher.publish(joint_state)

        print("\nFront Right Limb:")
        print(f"front_left_wrist_joint: {math.degrees(self.q1)} °")
        print(f"front_left_leg_joint: {math.degrees(self.q2)} °")
        print(f"front_left_foot_joint: {math.degrees(self.q3)} °") 

class inverseKinematic:
    def __init__(self, x, y, z, l1, l2, l3):
        super().__init__()

        #------------------------------------------------------
        #PRIMERA PARTE DE LA CINEMATICA
        #------------------------------------------------------

        #------------------------------------------------------
        #SEGUNDA PARTE DE LA CINEMATICA
        #------------------------------------------------------

        a = math.sqrt((x**2) + (z-l1)**2)

        beta = math.atan2((z-l1), math.sqrt((x**2)+(z**2)))
        cos_q3 = ((l3**2)+(l2**2)-(a**2))/(2*l2*l3)
        cos_alfa = ((l2**2)+(a**2)-(l3**2))/(2*l2*a)

        sen_q3 = math.sqrt(1-(cos_q3**2))
        sen_alfa = math.sqrt(1-(cos_alfa**2))

        self.q3 = math.atan2(sen_q3, cos_q3)
        alfa = math.atan2(sen_alfa, cos_alfa)

        self.q2 = alfa + beta 

def main(args=None):

    rclpy.init(args=args)

    x1 = 0.05
    y1 = 0.0
    z1 = 0.05

    while x1 < 0.10:
        
        #x,y,z/l1,l2,l3
        datos = inverseKinematic(x1, y1, z1, 0.05, 0.12, 0.14)
        #Guardar las variables de la clase
        q2 = datos.q2
        q3 = datos.q3
            #Publicar las variables de la clase cinematica inversa a la clase publicador
        joint_AnglePosition = AnglePosition(0.0, q2, q3)
        rclpy.spin(joint_AnglePosition)
        x1 = x1 + 0.01

    time.sleep(5)

    if x1 == 0.05:
        datos = inverseKinematic(x1, y1, z1, 0.05, 0.12, 0.14)
        #Guardar las variables de la clase
        q2 = datos.q2
        q3 = datos.q3
        #Publicar las variables de la clase cinematica inversa a la clase publicador
        joint_AnglePosition = AnglePosition(0.0, q2, q3)
        rclpy.spin(joint_AnglePosition)
        x1 = 0.01

    joint_AnglePosition.destroy_node()
    rclpy.shutdown()

    
if __name__ == '__main__':
    main()

