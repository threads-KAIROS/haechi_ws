import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String


class VehicleNavigation(Node):
    def __init__(self):
        super().__init__('vehicle_navigation')
        self.stage = 0  # 단계 추적기

        # Localization Pose 구독자
        self.localization_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/rtabmap/localization_pose',
            self.localization_callback,
            10
        )
        self.get_logger().info("Subscribed to /rtabmap/localization_pose")

        # Goal Pose 구독자
        self.goal_subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_pose_callback,
            10
        )
        self.get_logger().info("Subscribed to /goal_pose")

        # AGV 명령 퍼블리셔
        self.command_publisher = self.create_publisher(String, '/agv_command', 10)

        # 제어 로직을 위한 타이머
        self.control_timer = self.create_timer(0.1, self.control_logic)  # 0.1초마다 control_logic 호출

        # 목표 위치
        self.target_x = None
        self.target_y = None
        self.target_z = 0  # 고정된 z 좌표

        # 현재 위치 및 방향
        self.current_x = None
        self.current_y = None
        self.current_z = None

        # 초기 위치 및 거리
        self.initial_x = None
        self.initial_y = None
        self.initial_distance_x = None
        self.initial_distance_y = None

        # 이동 중 도착 여부
        self.arrived_x = False
        self.arrived_y = False

    def localization_callback(self, msg):
        """현재 위치 데이터를 업데이트하는 콜백."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.orientation.z

        # 초기 위치 설정
        if self.initial_x is None and self.initial_y is None:
            self.initial_x = self.current_x
            self.initial_y = self.current_y
            self.get_logger().info(
                f"Initial position set -> x: {self.initial_x:.3f}, y: {self.initial_y:.3f}"
            )

    def goal_pose_callback(self, msg):
        """/goal_pose로부터 목표 지점을 설정하는 콜백."""
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y
        self.get_logger().info(f"Received new goal: x: {self.target_x:.3f}, y: {self.target_y:.3f}, z: {self.target_z:.3f}")

        # 초기 거리 계산
        if self.initial_x is not None and self.initial_y is not None:
            self.initial_distance_x = abs(self.target_x - self.initial_x)
            self.initial_distance_y = abs(self.target_y - self.initial_y)
            self.get_logger().info(
                f"Initial distance -> x: {self.initial_distance_x:.3f}, y: {self.initial_distance_y:.3f}"
            )

    def publish_command(self, command):
        """AGV에 명령을 퍼블리시합니다."""
        msg = String()
        msg.data = command
        self.command_publisher.publish(msg)
        self.get_logger().info(f"Published command: {command}")

    def log_current_position(self):
        """현재 위치와 방향을 로그로 출력합니다."""
        if self.current_x is not None and self.current_y is not None and self.current_z is not None:
            self.get_logger().info(
                f"Current Position -> x: {self.current_x:.3f}, y: {self.current_y:.3f}, z_orientation: {self.current_z:.3f}"
            )
        else:
            self.get_logger().info("Current position data is not yet available.")

    def control_logic(self):
        """목표를 향해 이동하기 위한 제어 로직."""
        current_x = self.current_x
        current_y = self.current_y
        current_z = self.current_z

        # 현재 위치와 목표 위치가 설정되었는지 확인
        if current_x is None or current_y is None or current_z is None:
            self.get_logger().info("현지화 데이터를 기다리는 중...")
            return

        if self.target_x is None or self.target_y is None:
            self.get_logger().info("목표 위치 데이터를 기다리는 중...")
            return

        # 현재 위치 로그
        self.log_current_position()

        # 거리 계산 (수정된 부분)
        distance_x = current_x - self.target_x  # 목표 지점에 대한 현재 위치의 거리 (양수: +X 방향)
        distance_y = current_y - self.target_y  # 목표 지점에 대한 현재 위치의 거리 (양수: +Y 방향)

        # 최종 목표 도달 조건
        if abs(distance_x) <= 0.1 and abs(distance_y) <= 0.1:
            self.get_logger().info(
                f"Stage 5: 최종 목표 지점에 도달했습니다. AGV 정지. [current_x: {current_x:.3f}, current_y: {current_y:.3f}, "
                f"distance_x: {distance_x:.3f}, distance_y: {distance_y:.3f}]"
            )
            self.publish_command('stop')
            self.stage = 6  # 종료 단계로 이동
            return

        # 단계별 로직
        if self.stage == 0:  # 초기 Z 축 정렬
            self.get_logger().info("Stage 0: 초기 Z 축 정렬 중...")
            # 여기서는 단순히 Z 축 정렬을 위한 조건 확인
            if -0.1 <= current_z <= 0.1:
                self.get_logger().info("Stage 0: Z 축 정렬 완료. Stage 1로 이동합니다.")
                self.stage = 1
            else:
                self.get_logger().info(f"Stage 0: 현재 Z={current_z:.3f}, Z 축 정렬 중...")
                self.publish_command('q')  # Rotate counterclockwise

        elif self.stage == 1:  # X 축 방향으로 이동
            self.get_logger().info("Stage 1: X 축 방향으로 이동 중...")
            if abs(distance_x) <= 0.1:
                self.get_logger().info("Stage 1: X 축 도달 범위 내. 정지하고 Stage 2로 이동합니다.")
                self.publish_command('stop')
                self.stage = 2
            else:
                self.publish_command('w')  # Move forward

        elif self.stage == 2:  # X, Y 축의 중간 방향을 위한 Z 축 정렬
            self.get_logger().info("Stage 2: X, Y 축의 중간 방향을 위한 Z 축 정렬 중...")
            # 사분면에 따른 Z 축 정렬
            if distance_x > 0 and distance_y > 0:  # 1사분면
                required_z_min = -0.8
                required_z_max = -0.6
            elif distance_x < 0 and distance_y > 0:  # 2사분면
                required_z_min = -0.4
                required_z_max = -0.2
            elif distance_x < 0 and distance_y < 0:  # 3사분면
                required_z_min = 0.2
                required_z_max = 0.4
            elif distance_x > 0 and distance_y < 0:  # 4사분면
                required_z_min = 0.6
                required_z_max = 0.8
            else:
                # 예외 처리: 사분면이 정의되지 않은 경우
                self.get_logger().info("Stage 2: 사분면 정의 오류. 정지합니다.")
                self.publish_command('stop')
                self.stage = 6  # 종료 단계로 이동
                return

            if required_z_min <= current_z <= required_z_max:
                self.get_logger().info("Stage 2: Z 축 정렬 완료. Stage 3로 이동합니다.")
                self.publish_command('stop')
                self.stage = 3
            else:
                self.get_logger().info(
                    f"Stage 2: 현재 Z={current_z:.3f}, [{required_z_min} <= Z <= {required_z_max}] 범위로 회전 중..."
                )
                self.publish_command('q')  # Rotate counterclockwise

        elif self.stage == 3:  # 이동 중
            self.get_logger().info("Stage 3: 이동 중...")
            self.publish_command('w')  # Move forward

            # 도착 여부 확인
            if abs(distance_x) <= 0.1 and abs(distance_y) <= 0.1:
                self.get_logger().info("Stage 3: 최종 목표 지점 도착.")
                self.publish_command('stop')
                self.stage = 5  # 최종 단계로 이동
                return

            # X 축 도달 확인
            if abs(distance_x) <= 0.1 and not self.arrived_x:
                self.get_logger().info("Stage 3: X 축 도달. Z 축 정렬을 위해 정지합니다.")
                self.publish_command('stop')
                self.stage = 4
                self.arrived_x = True  # 중복 정렬 방지
                return

            # Y 축 도달 확인
            if abs(distance_y) <= 0.1 and not self.arrived_y:
                self.get_logger().info("Stage 3: Y 축 도달. Z 축 정렬을 위해 정지합니다.")
                self.publish_command('stop')
                self.stage = 4
                self.arrived_y = True  # 중복 정렬 방지
                return

        elif self.stage == 4:  # X, Y 축 도달 후 Z 축 정렬
            self.get_logger().info("Stage 4: Z 축 정렬 중...")
            if self.arrived_x:
                required_z_min = -0.6
                required_z_max = -0.4
                axis = 'X 축'
            elif self.arrived_y:
                required_z_min = -0.1
                required_z_max = 0.1
                axis = 'Y 축'
            else:
                self.get_logger().info("Stage 4: 도착 축 정보 없음. 정지합니다.")
                self.publish_command('stop')
                self.stage = 6  # 종료 단계로 이동
                return

            if required_z_min <= current_z <= required_z_max:
                self.get_logger().info(f"Stage 4: {axis} 조건 후 Z 축 정렬 완료. Stage 5로 이동합니다.")
                self.publish_command('stop')
                self.stage = 5
            else:
                self.get_logger().info(
                    f"Stage 4: 현재 Z={current_z:.3f}, [{required_z_min} <= Z <= {required_z_max}] 범위로 회전 중..."
                )
                self.publish_command('q')  # Rotate counterclockwise

        elif self.stage == 5:  # 최종 목표 지점 도달을 위한 직진
            self.get_logger().info("Stage 5: 최종 목표 지점 도달을 위해 직진 중...")
            self.publish_command('w')  # Move forward

            # 최종 목표 지점 도착 확인
            if abs(distance_x) <= 0.1 and abs(distance_y) <= 0.1:
                self.get_logger().info("Stage 5: 최종 목표 지점에 도착했습니다. AGV 정지합니다.")
                self.publish_command('stop')
                self.stage = 6  # 종료 단계로 이동

        elif self.stage == 6:  # 종료 단계
            self.get_logger().info("Stage 6: 모든 단계 완료. 노드를 종료합니다.")
            self.publish_command('stop')
            # 노드를 종료하려면 외부에서 ROS 노드 종료 로직을 추가하거나, 여기서 직접 종료할 수 있습니다.
            # 예를 들어, 타이머를 취소하고 노드를 종료하는 방법:
            self.control_timer.cancel()
            self.destroy_node()
            rclpy.shutdown()

        else:
            self.get_logger().info(f"알 수 없는 단계: {self.stage}. AGV 정지.")
            self.publish_command('stop')


def main(args=None):
    rclpy.init(args=args)
    node = VehicleNavigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
