import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.timer_ = self.create_timer(1.0, self.publish_initial_pose)

    def publish_initial_pose(self):
        msg = JointTrajectory()
        msg.joint_names = [
            'hombro_DA_joint', 'brazo_DA_joint', 'muneca_DA_joint', 'end_effector_DA_joint',
            'hombro_IA_joint', 'brazo_IA_joint', 'muneca_IA_joint', 'end_effector_IA_joint',
            'hombro_DT_joint', 'brazo_DT_joint', 'muneca_DT_joint', 'end_effector_DT_joint',
            'hombro_IT_joint', 'brazo_IT_joint', 'muneca_IT_joint', 'end_effector_IT_joint'
        ]

        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Ajusta según la posición deseada
        point.time_from_start.sec = 1

        msg.points.append(point)
        self.publisher_.publish(msg)
        self.get_logger().info('Publicando posición inicial')

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()