#!/usr/bin/env python3
#Librerias de ROS2
from typing import List
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
#Librerias de Python
import math
import time

class InverseKinematicsPublisher(Node):
    def __init__(self, desired_goal):
        super().__init__('inverse_kinematic_publisher')

        self.desired_goal = desired_goal
            
        self.names_of_joints = [
            "front_left_wrist_joint",
            "front_left_leg_joint",
            "front_left_foot_joint",
            "back_left_wrist_joint",
            "back_left_leg_joint",
            "back_left_foot_joint",
            "front_right_wrist_joint",
            "front_right_leg_joint",
            "front_right_foot_joint",
            "back_right_wrist_joint",
            "back_right_leg_joint",
            "back_right_foot_joint"
        ]

        self.l1 = 0.05
        self.l2 = 0.12
        self.l3 = 0.14
        
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0

        self.angle_publisher = self.create_publisher(
            JointState, 
            '/joint_states',
            10
        )

        self.goal_index = 0
        self.timer = self.create_timer(0.5, self.publish_values_in_joints)
        
    def publish_values_in_joints(self):

        self.compute_inverse_kinematics(
            self.desired_goal[self.goal_index][0],
            self.desired_goal[self.goal_index][1],
            self.desired_goal[self.goal_index][2],
        )
        print(f"Goal #{self.goal_index+ 1}")
        print(f"\tx: {self.desired_goal[self.goal_index][0]}")
        print(f"\ty: {self.desired_goal[self.goal_index][1]}")
        print(f"\tz: {self.desired_goal[self.goal_index][2]}\n")

        front_left_q1 = self.q1
        front_left_q2 = self.q2
        front_left_q3 = self.q3
        back_left_q1 = self.q1
        back_left_q2 = self.q2
        back_left_q3 = self.q3
        front_right_q1 = self.q1
        front_right_q2 = self.q2
        front_right_q3 = self.q3
        back_right_q1 = self.q1
        back_right_q2 = self.q2
        back_right_q3 = self.q3

        self.values_in_joints = [
            front_left_q1,
            front_left_q2, 
            front_left_q3,
            back_left_q1,
            back_left_q2,
            back_left_q3,
            front_right_q1,
            front_right_q2, 
            front_right_q3,
            back_right_q1,
            back_right_q2,
            back_right_q3,
        ]

        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.names_of_joints
        joint_state.position = self.values_in_joints

        self.angle_publisher.publish(joint_state)

        if (self.goal_index < len(self.desired_goal) - 1):
            self.goal_index += 1
        else:
            self.goal_index = len(self.desired_goal) - 1

    def compute_inverse_kinematics(self, x, y, z):

        #------------------------------------------------------
        #PRIMERA PARTE DE LA CINEMATICA
        #------------------------------------------------------

        #------------------------------------------------------
        #SEGUNDA PARTE DE LA CINEMATICA
        #------------------------------------------------------

        a = math.sqrt((x)**2 + (z - self.l1)**2)

        beta = math.atan2((z - self.l1), math.sqrt((x**2)+(z**2)))
        cos_q3 = ((self.l3**2)+(self.l2**2)-(a**2))/(2*self.l2*self.l3)
        cos_alfa = ((self.l2**2)+(a**2)-(self.l3**2))/(2*self.l2*a)

        sen_q3 = math.sqrt(1-(cos_q3)**2)
        sen_alfa = math.sqrt(1-(cos_alfa)**2)

        self.q3 = math.atan2(sen_q3, cos_q3)
        alfa = math.atan2(sen_alfa, cos_alfa)

        self.q2 = -alfa - beta 

def main(args=None):
    rclpy.init(args=args)
    desired_goal = [
        [0.0, 0.0, 0.0],
        [0.01, 0.0, 0.0],
        [0.02, 0.0, 0.0],
        [0.03, 0.0, 0.0],
        [0.04, 0.0, 0.0],
        [0.05, 0.0, 0.0],
        [0.06, 0.0, 0.0],
        [0.07, 0.0, 0.0],
        [0.08, 0.0, 0.0],
        [0.09, 0.0, 0.0],
        [0.1, 0.0, 0.0],
        [0.11, 0.0, 0.0],
        [0.12, 0.0, -0.01],
        [0.13, 0.0, -0.02],
        [0.14, 0.0, -0.03],
        [0.09, 0.0, -0.04],
        [0.08, 0.0, -0.05],
        [0.07, 0.0, -0.06],
        [0.06, 0.0, -0.07],
        [0.05, 0.0, -0.08],
        [0.04, 0.0, -0.07],
        [0.03, 0.0, -0.06],
        [0.03, 0.0, -0.05],
        [0.03, 0.0, -0.04],
        [0.03, 0.0, -0.03],
        [0.03, 0.0, -0.02],

    ]
    inverse_kinematics_publisher_node = InverseKinematicsPublisher(desired_goal)
    
    try:
        rclpy.spin(inverse_kinematics_publisher_node)
    except KeyboardInterrupt:
        inverse_kinematics_publisher_node.destroy_node()
        rclpy.try_shutdown()

    
if __name__ == '__main__':
    main()



