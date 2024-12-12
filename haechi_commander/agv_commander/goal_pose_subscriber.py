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
            self.goal_pose_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

    def goal_pose_callback(self, msg):
        # x, y 좌표를 소수점 3자리로 표시
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.get_logger().info(f"Received goal pose:\nx: {x:.3f}, y: {y:.3f}")

def main(args=None):
    rclpy.init(args=args)
    node = GoalPoseSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
