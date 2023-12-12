#!/usr/bin/env python3
# ROS2 libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
# Python libraries
import math

class JointAnglePositionPublisher(Node):
    def __init__(self):
        super().__init__('joint_group_position_publisher')

        self.final_position = math.radians(30)

        self.back_left_wrist_joint = 0.0
        self.back_left_leg_joint = 0.0
        self.back_left_foot_joint = 0.0
        self.back_right_wrist_joint = 0.0
        self.back_right_leg_joint = 0.0
        self.back_right_foot_joint = 0.0
        self.front_left_wrist_joint = 0.0
        self.front_left_leg_joint = 0.0
        self.front_left_foot_joint = 0.0
        self.front_right_wrist_joint = 0.0
        self.front_right_leg_joint = 0.0
        self.front_right_foot_joint = 0.0

        self.angle_publisher = self.create_publisher(
            Float64MultiArray, 
            "/joint_group_position_controller/commands", 
            10
        )
         
        self.angular_position_msg = Float64MultiArray()
        self.position_timer = self.create_timer(
            1.5, 
            self.publish_position_in_joints
        )
    
    def publish_position_in_joints(self):
        self.angular_position_msg.data = [
            self.back_left_wrist_joint,
            self.back_left_leg_joint,
            self.back_left_foot_joint,
            self.back_right_wrist_joint,
            self.back_right_leg_joint,
            self.back_right_foot_joint,
            self.front_left_wrist_joint,
            self.front_left_leg_joint,
            self.front_left_foot_joint,
            self.front_right_wrist_joint,
            self.front_right_leg_joint,
            self.front_right_foot_joint
        ]
        self.angle_publisher.publish(self.angular_position_msg)
        
        print("\nFront Right Limb:")
        print(f"front_left_wrist_joint: {math.degrees(self.front_right_wrist_joint)} °")
        print(f"front_left_leg_joint: {math.degrees(self.front_right_leg_joint)} °")
        print(f"front_left_foot_joint: {math.degrees(self.front_right_foot_joint)} °") 
        print("Front Left Limb:")
        print(f"front_left_wrist_joint: {math.degrees(self.front_left_wrist_joint)} °")
        print(f"front_left_leg_joint: {math.degrees(self.front_left_leg_joint)} °")
        print(f"front_left_foot_joint: {math.degrees(self.front_left_foot_joint)} °") 
        print("Back Right Limb:")
        print(f"back_right_wrist_joint: {math.degrees(self.back_right_wrist_joint)} °")
        print(f"back_right_leg_joint: {math.degrees(self.back_right_leg_joint)} °")
        print(f"back_right_foot_joint: {math.degrees(self.back_right_foot_joint)} °")
        print("Back Left Limb:")
        print(f"back_left_wrist_joint: {math.degrees(self.back_left_wrist_joint)} °")
        print(f"back_left_leg_joint: {math.degrees(self.back_left_leg_joint)} °")
        print(f"back_left_foot_joint: {math.degrees(self.back_left_foot_joint)} °\n")

        self.front_right_wrist_joint += math.radians(1)

        if(self.front_right_wrist_joint >= self.final_position):
                self.front_right_wrist_joint = self.final_position

def main(args=None):
    rclpy.init(args=args)
    joint_angle_position_publisher_node = JointAnglePositionPublisher()
    try:
        rclpy.spin(joint_angle_position_publisher_node)
    except KeyboardInterrupt:
        joint_angle_position_publisher_node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()