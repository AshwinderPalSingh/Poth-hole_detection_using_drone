"""
pothole_detector.py — YOLOv8 Pothole Detection ROS 2 Node
═══════════════════════════════════════════════════════════
Subscribes to the drone's downward camera feed, runs YOLOv8
inference for pothole detection, and publishes:
  • Annotated images with bounding boxes
  • Detection messages with class, confidence, and bbox

Supports both real-time and batch inference modes.

Usage:
    ros2 run phoenix_drone_perception pothole_detector

Dependencies:
    pip install ultralytics opencv-python-headless
"""
import json
import os
import time
from typing import Optional

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Point

# cv_bridge ships a C extension built against the NumPy the distro shipped.
# Importing it under a newer NumPy can abort the process outright (not just
# raise), so check ABI compatibility FIRST and skip the import when the major
# versions differ.  The pure-NumPy conversion below is used instead and is
# functionally equivalent for the encodings Gazebo publishes.
CvBridge = None
_CV_BRIDGE_ERROR = None
try:
    import numpy as _np
    _NUMPY_MAJOR = int(_np.__version__.split('.')[0])
except Exception:                          # noqa: BLE001
    _NUMPY_MAJOR = 1

if _NUMPY_MAJOR >= 2:
    _CV_BRIDGE_ERROR = RuntimeError(
        f'cv_bridge is built against NumPy 1.x but NumPy '
        f'{_np.__version__} is installed')
else:
    try:
        from cv_bridge import CvBridge
    except BaseException as _e:            # noqa: BLE001
        CvBridge = None
        _CV_BRIDGE_ERROR = _e

try:
    from ultralytics import YOLO
except BaseException:                  # noqa: BLE001
    YOLO = None


class PotholeDetector(Node):
    """
    Real-time pothole detection node using YOLOv8.

    Subscribes to the drone camera and publishes detection results.
    Falls back to OpenCV-based detection if YOLOv8 is not available.
    """

    def __init__(self):
        super().__init__('pothole_detector')

        # ── Parameters ──
        self.declare_parameter('model_path', '')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('image_size', 640)
        self.declare_parameter('device', 'cpu')  # 'cpu', 'cuda', or 'cuda:0'
        self.declare_parameter('camera_topic', '/phoenix/camera/image_raw')
        self.declare_parameter('use_yolo', True)
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('max_inference_fps', 10.0)
        # Classical-CV fallback tuning
        self.declare_parameter('min_area_px', 150)
        self.declare_parameter('min_circularity', 0.45)
        self.declare_parameter('darkness_ratio', 0.35)

        self._model_path = self.get_parameter('model_path').value
        self._conf_threshold = self.get_parameter('confidence_threshold').value
        self._img_size = self.get_parameter('image_size').value
        self._device = self.get_parameter('device').value
        self._camera_topic = self.get_parameter('camera_topic').value
        self._use_yolo = self.get_parameter('use_yolo').value
        self._publish_annotated = self.get_parameter('publish_annotated').value
        self._max_fps = self.get_parameter('max_inference_fps').value
        self._min_area_px = self.get_parameter('min_area_px').value
        self._min_circularity = self.get_parameter('min_circularity').value
        self._darkness_ratio = self.get_parameter('darkness_ratio').value

        # ── CV Bridge ──
        if CvBridge is not None:
            self._bridge = CvBridge()
        else:
            self._bridge = None
            self.get_logger().warn(
                f'cv_bridge unavailable ({type(_CV_BRIDGE_ERROR).__name__}: '
                f'{_CV_BRIDGE_ERROR}) — using the built-in NumPy image '
                f'conversion instead. Detection is unaffected.'
            )

        # ── Load YOLOv8 model ──
        self._model: Optional[object] = None
        if self._use_yolo and YOLO is not None:
            self._load_yolo_model()
        elif self._use_yolo:
            self.get_logger().warn(
                'ultralytics not installed. Install with: '
                'pip install ultralytics\n'
                'Falling back to OpenCV-based detection.'
            )
            self._use_yolo = False

        # ── Rate limiting ──
        self._min_interval = 1.0 / self._max_fps if self._max_fps > 0 else 0
        self._last_inference_time = 0.0

        # ── Detection statistics ──
        self._frame_count = 0
        self._detection_count = 0

        # ── QoS for the camera ──
        # gazebo_ros_camera publishes RELIABLE.  A BEST_EFFORT subscription
        # is an incompatible-QoS match and receives nothing at all, so use
        # RELIABLE with a shallow queue (old frames are useless anyway).
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Subscribers ──
        self.create_subscription(
            Image,
            self._camera_topic,
            self._image_callback,
            sensor_qos,
        )

        # ── Publishers ──
        self._annotated_pub = self.create_publisher(
            Image,
            '/phoenix/perception/annotated_image',
            10,
        )
        self._detections_pub = self.create_publisher(
            String,
            '/phoenix/perception/detections',
            10,
        )
        self._pothole_point_pub = self.create_publisher(
            Point,
            '/phoenix/perception/pothole_center',
            10,
        )

        self.get_logger().info(
            '╔══════════════════════════════════════════════╗\n'
            '║  PhoenixDrone Pothole Detector Started       ║\n'
            f'║  Model: {"YOLOv8" if self._use_yolo else "OpenCV fallback":37s}║\n'
            f'║  Device: {self._device:36s}║\n'
            f'║  Conf threshold: {self._conf_threshold:<28.2f}║\n'
            '╚══════════════════════════════════════════════╝'
        )

    # ═══════════════════════════════════════════════════════════
    #  MODEL LOADING
    # ═══════════════════════════════════════════════════════════

    def _load_yolo_model(self):
        """Load the YOLOv8 model for pothole detection."""
        if self._model_path and os.path.isfile(self._model_path):
            self.get_logger().info(f'Loading custom model: {self._model_path}')
            self._model = YOLO(self._model_path)
        else:
            # Use YOLOv8n as a placeholder — user must train/provide
            # a pothole-specific model for real detection
            self.get_logger().warn(
                'No custom pothole model found. Using YOLOv8n (generic).\n'
                'For real pothole detection, train a model and set '
                '"model_path" parameter.\n'
                'See: models/README.md for instructions.'
            )
            self._model = YOLO('yolov8n.pt')

        self.get_logger().info('YOLOv8 model loaded successfully')

    # ═══════════════════════════════════════════════════════════
    #  IMAGE PROCESSING
    # ═══════════════════════════════════════════════════════════

    def _image_callback(self, msg: Image):
        """Process incoming camera image for pothole detection."""
        # Rate limiting
        now = time.time()
        if now - self._last_inference_time < self._min_interval:
            return
        self._last_inference_time = now
        self._frame_count += 1

        # Convert ROS Image → OpenCV
        cv_image = self._ros_to_cv(msg)
        if cv_image is None:
            return

        # Run detection
        if self._use_yolo and self._model is not None:
            detections, annotated = self._detect_yolo(cv_image)
        else:
            detections, annotated = self._detect_opencv_fallback(cv_image)

        # Publish results as JSON so downstream nodes get structured bboxes
        if detections:
            self._detection_count += len(detections)
            det_msg = String()
            det_msg.data = json.dumps({
                'stamp': {
                    'sec': msg.header.stamp.sec,
                    'nanosec': msg.header.stamp.nanosec,
                },
                'frame': self._frame_count,
                'image_width': cv_image.shape[1],
                'image_height': cv_image.shape[0],
                'detections': detections,
            })
            self._detections_pub.publish(det_msg)

            # Publish center points of each detection
            for d in detections:
                pt = Point()
                pt.x = (d['bbox'][0] + d['bbox'][2]) / 2.0
                pt.y = (d['bbox'][1] + d['bbox'][3]) / 2.0
                pt.z = d['confidence']
                self._pothole_point_pub.publish(pt)

            self.get_logger().info(
                f'Frame {self._frame_count}: {len(detections)} pothole(s) '
                f'detected (total: {self._detection_count})',
                throttle_duration_sec=2.0,
            )

        # Publish annotated image
        if self._publish_annotated and annotated is not None:
            ann_msg = self._cv_to_ros(annotated, msg.header)
            if ann_msg is not None:
                self._annotated_pub.publish(ann_msg)

    def _detect_yolo(self, image: np.ndarray):
        """Run YOLOv8 inference on the image."""
        results = self._model(
            image,
            conf=self._conf_threshold,
            imgsz=self._img_size,
            device=self._device,
            verbose=False,
        )

        detections = []
        annotated = image.copy()

        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                cls_name = (
                    self._model.names[cls_id]
                    if hasattr(self._model, 'names')
                    else f'class_{cls_id}'
                )

                detections.append({
                    'class': cls_name,
                    'confidence': conf,
                    'bbox': [float(x1), float(y1), float(x2), float(y2)],
                })

                # Draw bounding box
                cv2.rectangle(
                    annotated,
                    (int(x1), int(y1)),
                    (int(x2), int(y2)),
                    (0, 0, 255),  # Red
                    2,
                )
                label = f'{cls_name} {conf:.2f}'
                cv2.putText(
                    annotated, label,
                    (int(x1), int(y1) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 0, 255), 2,
                )

        return detections, annotated

    def _detect_opencv_fallback(self, image: np.ndarray):
        """
        Fallback pothole detection using classical computer vision.

        Potholes on asphalt are darker than the surrounding road and
        roughly convex.  Rather than a fixed threshold (which fails as
        lighting changes with altitude), this adapts to the road itself:

          1. Estimate the road brightness from the image median.
          2. Threshold pixels significantly darker than that median.
          3. Reject lane markings (bright), shadows at frame edges,
             and non-convex shapes via contour analysis.

        Confidence blends circularity, solidity and darkness contrast.
        """
        detections = []
        annotated = image.copy()

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (7, 7), 0)

        # ── Adaptive dark-region threshold, relative to the road ──
        road_level = float(np.median(blurred))
        cut = max(8.0, road_level * self._darkness_ratio)
        _, thresh = cv2.threshold(
            blurred, max(0.0, road_level - cut), 255, cv2.THRESH_BINARY_INV)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        h, w = gray.shape[:2]
        frame_area = float(h * w)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self._min_area_px or area > frame_area * 0.25:
                continue

            perimeter = cv2.arcLength(contour, True)
            if perimeter <= 0:
                continue

            circularity = 4.0 * np.pi * area / (perimeter * perimeter)
            if circularity < self._min_circularity:
                continue

            hull = cv2.convexHull(contour)
            hull_area = cv2.contourArea(hull)
            solidity = area / hull_area if hull_area > 0 else 0.0
            if solidity < 0.75:
                continue

            x, y, bw, bh = cv2.boundingRect(contour)

            # Reject extreme slivers
            aspect = bw / float(bh) if bh > 0 else 99.0
            if aspect < 0.35 or aspect > 2.85:
                continue

            # Reject blobs clipped by the frame edge (partial, unreliable)
            if x <= 1 or y <= 1 or x + bw >= w - 1 or y + bh >= h - 1:
                continue

            # Darkness contrast: how much darker than the road is it?
            mask = np.zeros(gray.shape, dtype=np.uint8)
            cv2.drawContours(mask, [contour], -1, 255, -1)
            blob_level = float(cv2.mean(blurred, mask=mask)[0])
            contrast = (road_level - blob_level) / max(road_level, 1.0)
            if contrast < 0.12:
                continue

            confidence = float(np.clip(
                0.45 * circularity + 0.25 * solidity + 0.30 * min(contrast * 2.5, 1.0),
                0.0, 0.99))
            if confidence < self._conf_threshold:
                continue

            detections.append({
                'class': 'pothole',
                'confidence': confidence,
                'bbox': [float(x), float(y), float(x + bw), float(y + bh)],
            })

            cv2.rectangle(annotated, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
            cv2.putText(
                annotated, f'pothole {confidence:.2f}',
                (x, max(12, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return detections, annotated

    # ═══════════════════════════════════════════════════════════
    #  IMAGE CONVERSION UTILITIES
    # ═══════════════════════════════════════════════════════════

    def _ros_to_cv(self, msg: Image) -> Optional[np.ndarray]:
        """Convert ROS Image message to OpenCV BGR format."""
        if self._bridge is not None:
            try:
                return self._bridge.imgmsg_to_cv2(msg, 'bgr8')
            except Exception as e:
                self.get_logger().error(f'cv_bridge conversion failed: {e}')
                return None

        # Manual fallback conversion
        try:
            dtype = np.uint8
            if msg.encoding in ('rgb8', 'bgr8'):
                channels = 3
            elif msg.encoding in ('rgba8', 'bgra8'):
                channels = 4
            else:
                channels = 3

            img = np.frombuffer(msg.data, dtype=dtype)
            img = img.reshape((msg.height, msg.width, channels))

            if msg.encoding == 'rgb8':
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            elif msg.encoding == 'rgba8':
                img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)

            return img
        except Exception as e:
            self.get_logger().error(f'Manual image conversion failed: {e}')
            return None

    def _cv_to_ros(self, image: np.ndarray, header=None) -> Optional[Image]:
        """Convert OpenCV BGR image to ROS Image message."""
        if self._bridge is not None:
            try:
                msg = self._bridge.cv2_to_imgmsg(image, 'bgr8')
                if header:
                    msg.header = header
                return msg
            except Exception as e:
                self.get_logger().error(f'cv_bridge conversion failed: {e}')
                return None

        # Manual fallback
        try:
            msg = Image()
            if header:
                msg.header = header
            msg.height = image.shape[0]
            msg.width = image.shape[1]
            msg.encoding = 'bgr8'
            msg.step = image.shape[1] * 3
            msg.data = image.tobytes()
            return msg
        except Exception as e:
            self.get_logger().error(f'Manual conversion failed: {e}')
            return None


def main(args=None):
    rclpy.init(args=args)
    node = PotholeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Pothole detector stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
