import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class AGVCommanderNode(Node):
    def __init__(self):
        super().__init__('agv_commander')

        # 명령 토픽 퍼블리셔
        self.publisher = self.create_publisher(String, '/agv_command', 10)
        self.get_logger().info("Ready to send commands to /agv_command topic")

    def send_command(self):
        self.get_logger().info("Enter 'w' for forward, 'q' for rotate, 'stop' to stop, 'exit' to quit")
        while rclpy.ok():
            command = input("Enter command: ").strip()
            if command == 'exit':
                self.get_logger().info("Exiting commander")
                break
            msg = String()
            msg.data = command
            self.publisher.publish(msg)
            self.get_logger().info(f"Published command: {command}")


def main(args=None):
    rclpy.init(args=args)
    node = AGVCommanderNode()
    try:
        node.send_command()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
