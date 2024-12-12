import os
import cv2
import time
import rclpy
from threading import Thread, Timer, Lock
from sensor_msgs.msg import Image
from visualization_msgs.msg import MarkerArray
from cv_bridge import CvBridge
from flask import Flask, Response, render_template, jsonify
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Flask 앱 설정
app = Flask(__name__)

# ROS 2 초기화
rclpy.init()
bridge = CvBridge()
image_data = None
robot_position = {'x': 0.0, 'y': 0.0}
yolo_image_data = None
yolo_marker_data = None

# 공유 데이터 보호용 Lock
data_lock = Lock()

# 영상 캡처 플래그
is_capturing = False

# 캡처된 이미지 저장 디렉토리
capture_dir = "captured_images"
if not os.path.exists(capture_dir):
    os.makedirs(capture_dir)

# QoS 설정 (퍼블리셔와 일치하도록 조정 필요)
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,  # 퍼블리셔도 BSET_EFFORT 사용 시
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

# 카메라와 위치를 동시에 구독하는 ROS 2 노드
class MultiSubscriber(Node):
    def __init__(self):
        super().__init__('multi_subscriber')

        # 카메라 이미지 구독
        self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            qos_profile
        )

        # 로봇 위치 구독
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/rtabmap/localization_pose',
            self.localization_callback,
            qos_profile
        )

        # YOLO로 검출된 이미지 구독
        self.create_subscription(
            Image,
            '/yolo/detected_image',
            self.yolo_image_callback,
            qos_profile
        )

        # YOLO로 검출된 마커 구독
        self.create_subscription(
            MarkerArray,
            '/yolo/detected_markers',
            self.yolo_marker_callback,
            qos_profile
        )

    def image_callback(self, msg):
        global image_data
        with data_lock:
            try:
                image_data = bridge.imgmsg_to_cv2(msg, 'bgr8')
            except Exception as e:
                self.get_logger().error(f"Error converting ROS Image to OpenCV: {e}")

    def localization_callback(self, msg):
        global robot_position
        with data_lock:
            robot_position['x'] = msg.pose.pose.position.x
            robot_position['y'] = msg.pose.pose.position.y

    def yolo_image_callback(self, msg):
        global yolo_image_data
        with data_lock:
            try:
                yolo_image_data = bridge.imgmsg_to_cv2(msg, 'bgr8')
            except Exception as e:
                self.get_logger().error(f"Error converting YOLO Image to OpenCV: {e}")

    def yolo_marker_callback(self, msg):
        global yolo_marker_data
        with data_lock:
            yolo_marker_data = msg.markers

# Flask 라우터: 메인 페이지
@app.route('/')
def index():
    return render_template('index.html')

# Flask 라우터: 카메라 비디오 스트리밍
@app.route('/video_feed')
def video_feed():
    return Response(generate_stream(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

# Flask 라우터: YOLO 비디오 스트리밍
@app.route('/yolo_video_feed')
def yolo_video_feed():
    return Response(yolo_generate_stream(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

# Flask 라우터: 로봇 위치
@app.route('/robot_position')
def robot_position_view():
    with data_lock:
        return jsonify(robot_position)

# Flask 라우터: 사람 감지
@app.route('/human_detection', methods=['GET'])
def human_detection():
    global yolo_marker_data
    detected = False
    with data_lock:
        if yolo_marker_data:
            for marker in yolo_marker_data:
                if marker.ns == "yolo_bounding_box":
                    detected = True
                    break
    return jsonify({"detected": detected})

# Flask 라우터: 영상 캡처 요청
@app.route('/start_capture', methods=['POST'])
def start_capture():
    global is_capturing
    if not is_capturing:
        is_capturing = True
        capture_video_frames()
        return jsonify({"status": "Capture started"})
    return jsonify({"status": "Capture already running"})

# Flask 라우터: 영상 캡처 중단
@app.route('/stop_capture', methods=['POST'])
def stop_capture():
    global is_capturing
    is_capturing = False
    return jsonify({"status": "Capture stopped"})

# 비디오 프레임 생성
def generate_stream():
    while True:
        with data_lock:
            current_image = image_data
        if current_image is not None:
            ret, jpeg = cv2.imencode('.jpg', current_image)
            if ret:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n\r\n')
        time.sleep(0.1)  # CPU 사용률을 낮추기 위해 짧은 대기 시간 추가

def yolo_generate_stream():
    while True:
        with data_lock:
            current_yolo_image = yolo_image_data
        if current_yolo_image is not None:
            ret, jpeg = cv2.imencode('.jpg', current_yolo_image)
            if ret:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n\r\n')
        time.sleep(0.1)

# 캡처된 프레임 저장
def capture_video_frames():
    global is_capturing
    if not is_capturing:
        return
    with data_lock:
        if image_data is not None:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            file_path = os.path.join(capture_dir, f"capture_{timestamp}.jpg")
            cv2.imwrite(file_path, image_data)
    Timer(5, capture_video_frames).start()

# ROS 2 노드를 별도의 스레드에서 실행
def start_ros2_node():
    node = MultiSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("ROS 2 노드가 종료되었습니다.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

# Flask 웹 서버 실행
if __name__ == '__main__':
    ros_thread = Thread(target=start_ros2_node)
    ros_thread.start()

    try:
        app.run(host='0.0.0.0', port=5000, threaded=True, debug=False)
    except KeyboardInterrupt:
        print("Flask 서버가 종료되었습니다.")
    finally:
        rclpy.shutdown()
