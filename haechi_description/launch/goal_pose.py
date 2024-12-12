import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class GoalPoseSubscriber(Node):
    def __init__(self):
        super().__init__('goal_pose_subscriber')
        # /goal_pose 토픽 구독
        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.listener_callback,
            10  # QoS depth 설정
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # PoseStamped 메시지에서 x, y, z 값 추출 및 소수점 3자리로 제한
        current_x = msg.pose.position.x
        current_y = msg.pose.position.y
        
        # 터미널에 출력
        self.get_logger().info(
            f"Current Position and Orientation: x={current_x:.3f}, y={current_y:.3f},"
        )

def main(args=None):
    rclpy.init(args=args)
    node = GoalPoseSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
