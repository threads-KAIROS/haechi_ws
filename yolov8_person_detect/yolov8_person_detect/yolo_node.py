import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2


class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        # QoS 설정: BEST_EFFORT로 설정
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 카메라 이미지 토픽 구독
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            qos_profile
        )

        # 결과 이미지 및 Marker 퍼블리셔 설정
        self.image_pub = self.create_publisher(Image, '/yolo/detected_image', qos_profile)
        self.marker_pub = self.create_publisher(MarkerArray, '/yolo/detected_markers', qos_profile)

        # OpenCV와 YOLO 설정
        self.bridge = CvBridge()
        self.model = YOLO("yolov8n.pt")  # YOLOv8 모델 로드
        self.model.classes = [0]  # 사람만 감지

        self.get_logger().info("YOLO Node initialized successfully!")

    def image_callback(self, msg):
        try:
            # ROS 이미지 메시지를 OpenCV 이미지로 변환
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # YOLOv8으로 프레임 처리
            results = self.model.predict(source=frame, verbose=False)

            # MarkerArray 초기화
            markers = MarkerArray()

            # Bounding Box를 그리기
            for i, result in enumerate(results[0].boxes):  # YOLOv8의 boxes 정보
                x1, y1, x2, y2 = map(int, result.xyxy[0])  # 좌표 정보
                conf = result.conf[0]  # confidence score
                cls = int(result.cls[0])  # class ID

                # 클래스가 사람(0)인 경우에만 처리
                if cls == 0:
                    # OpenCV에 박스 그리기
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(
                        frame,
                        f"Person: {conf:.2f}",
                        (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        2,
                    )

                    # Marker 생성
                    marker = Marker()
                    marker.header.frame_id = "camera_link"  # 카메라 프레임 설정
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.ns = "yolo_bounding_box"
                    marker.id = i
                    marker.type = Marker.CUBE
                    marker.action = Marker.ADD
                    marker.pose.position.x = (x1 + x2) / 2 / frame.shape[1]
                    marker.pose.position.y = (y1 + y2) / 2 / frame.shape[0]
                    marker.pose.position.z = 0.5  # 깊이는 고정값
                    marker.pose.orientation.w = 1.0
                    marker.scale.x = (x2 - x1) / frame.shape[1]
                    marker.scale.y = (y2 - y1) / frame.shape[0]
                    marker.scale.z = 0.1
                    marker.color.a = 0.8  # 투명도
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0

                    markers.markers.append(marker)

            # OpenCV 창에서 결과 표시
            cv2.imshow("YOLOv8 Person Detection", frame)
            cv2.waitKey(1)

            # 결과 이미지를 ROS 2 이미지 메시지로 퍼블리시
            detected_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            detected_image.header.frame_id = "camera_link"
            self.image_pub.publish(detected_image)

            # MarkerArray 퍼블리시
            self.marker_pub.publish(markers)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down YOLO Node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
