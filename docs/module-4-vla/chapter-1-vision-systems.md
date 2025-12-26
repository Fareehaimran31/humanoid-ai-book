---
sidebar_position: 1
title: "Vision Systems for Robotics"
---

# Vision Systems for Robotics

## Concept Overview

Vision systems are critical components of modern robotics, enabling robots to perceive and understand their environment through visual information. In robotics applications, vision systems go beyond simple image capture to include sophisticated processing pipelines that extract meaningful information for navigation, manipulation, and interaction. This chapter explores the integration of vision systems with robotic platforms, covering both traditional computer vision techniques and modern deep learning approaches.

## Architecture Diagram: Vision System Integration

```
┌─────────────────────────────────────────────────────────────────┐
│                    Vision System Architecture                   │
├─────────────────────────────────────────────────────────────────┤
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────┐ │
│  │   Image         │    │   Preprocessing │    │   Feature   │ │
│  │   Acquisition   │───▶│   & Enhancement│───▶│   Extraction│ │
│  │   (Cameras,     │    │   (Filtering,  │    │   (Detection│ │
│  │   Sensors)      │    │   Calibration)  │    │   , Tracking)│ │
│  └─────────────────┘    └─────────────────┘    └─────────────┘ │
│              │                   │                    │        │
│              └───────────┐     ┌─┘                  ┌─┘        │
│                          ▼     ▼                  ▼            │
│                    ┌─────────────────────────────────────────┐ │
│                    │      Perception Pipeline              │ │
│                    │   (Object Detection, Recognition)     │ │
│                    └─────────────────────────────────────────┘ │
│                                   │                           │
│                                   ▼                           │
│                    ┌─────────────────────────────────────────┐ │
│                    │    Decision & Action Layer            │ │
│                    │   (Navigation, Manipulation, Control) │ │
│                    └─────────────────────────────────────────┘ │
│                                   │                           │
│                                   ▼                           │
│                    ┌─────────────────────────────────────────┐ │
│                    │      Integration Layer                │ │
│                    │   (ROS 2, Isaac ROS, Isaac Lab)       │ │
│                    └─────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
```

## Vision System Components

### Camera Types and Selection

Different camera types serve different robotic applications:

1. **RGB Cameras**: Standard color cameras for general-purpose vision
2. **Depth Cameras**: Provide depth information for 3D scene understanding
3. **Stereo Cameras**: Generate depth maps through stereo vision
4. **Thermal Cameras**: Detect heat signatures, useful in low-light conditions
5. **Event Cameras**: Ultra-fast response to changes in scene, good for high-speed applications

### Camera Calibration

Proper camera calibration is essential for accurate vision-based robotics:

```python
import numpy as np
import cv2
from cv2 import aruco
from sensor_msgs.msg import CameraInfo

class CameraCalibrator:
    def __init__(self, board_size=(9, 6), square_size=0.025):
        self.board_size = board_size  # Number of inner corners per chessboard row and column
        self.square_size = square_size  # Size of chessboard squares in meters
        self.obj_points = []  # 3D points in real world space
        self.img_points = []  # 2D points in image plane

    def create_object_points(self):
        """Create object points for chessboard corners."""
        objp = np.zeros((self.board_size[0] * self.board_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.board_size[0], 0:self.board_size[1]].T.reshape(-1, 2)
        objp *= self.square_size
        return objp

    def find_corners(self, image):
        """Find chessboard corners in image."""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, self.board_size, None)

        if ret:
            # Refine corner positions
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), criteria
            )
            return corners, gray
        return None, gray

    def calibrate_camera(self, images):
        """Calibrate camera using chessboard images."""
        objp = self.create_object_points()

        for img in images:
            corners, gray = self.find_corners(img)
            if corners is not None:
                self.obj_points.append(objp)
                self.img_points.append(corners)

        if len(self.obj_points) > 0:
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
                self.obj_points, self.img_points, gray.shape[::-1], None, None
            )
            return ret, mtx, dist, rvecs, tvecs
        else:
            return False, None, None, None, None

    def get_camera_info_msg(self, camera_matrix, dist_coeffs, image_width, image_height):
        """Create a CameraInfo message from calibration results."""
        camera_info = CameraInfo()
        camera_info.width = image_width
        camera_info.height = image_height
        camera_info.distortion_model = 'plumb_bob'
        camera_info.d = dist_coeffs.flatten().tolist()
        camera_info.k = camera_matrix.flatten().tolist()

        # Set rectification matrix to identity
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

        # Set projection matrix (for rectified image)
        camera_info.p = [
            camera_matrix[0, 0], 0.0, camera_matrix[0, 2], 0.0,
            0.0, camera_matrix[1, 1], camera_matrix[1, 2], 0.0,
            0.0, 0.0, 1.0, 0.0
        ]

        return camera_info
```

## ROS 2 Vision Pipeline

### Image Processing Node

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Point

class VisionProcessor(Node):
    def __init__(self):
        super().__init__('vision_processor')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Create subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.info_callback,
            10
        )

        # Create publishers
        self.processed_image_pub = self.create_publisher(
            Image,
            '/camera/image_processed',
            10
        )

        self.object_detection_pub = self.create_publisher(
            String,
            '/vision/object_detections',
            10
        )

        self.object_position_pub = self.create_publisher(
            Point,
            '/vision/object_position',
            10
        )

        # Camera parameters (will be filled from camera_info)
        self.camera_matrix = None
        self.dist_coeffs = None

        # Processing parameters
        self.object_color_lower = np.array([20, 100, 100])  # HSV values for yellow
        self.object_color_upper = np.array([30, 255, 255])  # HSV values for yellow

        self.get_logger().info('Vision Processor initialized')

    def info_callback(self, msg):
        """Process camera info for calibration parameters."""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Camera calibration parameters loaded')

    def image_callback(self, msg):
        """Process incoming image."""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
            return

        # Apply camera distortion correction if calibration is available
        if self.camera_matrix is not None and self.dist_coeffs is not None:
            cv_image = cv2.undistort(
                cv_image,
                self.camera_matrix,
                self.dist_coeffs,
                None,
                self.camera_matrix
            )

        # Process the image to detect objects
        processed_image, detections = self.process_image(cv_image)

        # Publish processed image
        try:
            processed_msg = self.bridge.cv2_to_imgmsg(processed_image, "bgr8")
            processed_msg.header = msg.header
            self.processed_image_pub.publish(processed_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing processed image: {e}')

        # Publish object detections
        if detections:
            detection_msg = String()
            detection_msg.data = str(detections)
            self.object_detection_pub.publish(detection_msg)

            # Publish position of the largest detected object
            if detections:
                largest_obj = max(detections, key=lambda x: x['area'])
                pos_msg = Point()
                pos_msg.x = float(largest_obj['center'][0])
                pos_msg.y = float(largest_obj['center'][1])
                pos_msg.z = 0.0  # Depth will be estimated separately
                self.object_position_pub.publish(pos_msg)

    def process_image(self, image):
        """Process image to detect objects of interest."""
        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Create mask for the target color
        mask = cv2.inRange(hsv, self.object_color_lower, self.object_color_upper)

        # Apply morphological operations to clean up the mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Process contours to find objects
        detections = []
        output_image = image.copy()

        for contour in contours:
            area = cv2.contourArea(contour)

            # Filter small contours
            if area > 500:  # Minimum area threshold
                # Calculate bounding box
                x, y, w, h = cv2.boundingRect(contour)

                # Calculate center
                center_x = x + w // 2
                center_y = y + h // 2

                # Calculate aspect ratio to filter out non-object shapes
                aspect_ratio = float(w) / h
                if 0.5 < aspect_ratio < 2.0:  # Reasonable aspect ratio
                    # Draw bounding box
                    cv2.rectangle(output_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.circle(output_image, (center_x, center_y), 5, (0, 0, 255), -1)

                    detections.append({
                        'center': (center_x, center_y),
                        'bbox': (x, y, w, h),
                        'area': area,
                        'aspect_ratio': aspect_ratio
                    })

        # Draw detections count
        cv2.putText(
            output_image,
            f'Objects: {len(detections)}',
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (255, 255, 255),
            2
        )

        return output_image, detections

def main(args=None):
    rclpy.init(args=args)
    vision_processor = VisionProcessor()

    try:
        rclpy.spin(vision_processor)
    except KeyboardInterrupt:
        pass
    finally:
        vision_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 3D Vision and Depth Processing

### Point Cloud Processing

```python
import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header

class PointCloudProcessor:
    def __init__(self):
        self.voxel_size = 0.05  # 5cm voxels
        self.distance_threshold = 0.02  # 2cm for plane segmentation

    def process_pointcloud(self, pointcloud_msg):
        """Process a ROS PointCloud2 message."""
        # Convert ROS point cloud to numpy array
        points_list = []
        for point in pc2.read_points(pointcloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            points_list.append([point[0], point[1], point[2]])

        if not points_list:
            return None, None

        points = np.array(points_list)

        # Create Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # Downsample point cloud
        downsampled_pcd = pcd.voxel_down_sample(voxel_size=self.voxel_size)

        # Segment plane (e.g., ground plane)
        plane_model, inliers = downsampled_pcd.segment_plane(
            distance_threshold=self.distance_threshold,
            ransac_n=3,
            num_iterations=1000
        )

        # Separate plane and objects
        plane_cloud = downsampled_pcd.select_by_index(inliers)
        object_cloud = downsampled_pcd.select_by_index(inliers, invert=True)

        return plane_cloud, object_cloud

    def extract_objects(self, pointcloud_msg):
        """Extract objects from point cloud using clustering."""
        plane_cloud, object_cloud = self.process_pointcloud(pointcloud_msg)

        if object_cloud is None or len(object_cloud.points) == 0:
            return []

        # Cluster objects using DBSCAN
        labels = np.array(object_cloud.cluster_dbscan(eps=0.1, min_points=10))

        # Group points by cluster
        unique_labels = set(labels)
        objects = []

        for label in unique_labels:
            if label == -1:  # Skip noise points
                continue

            # Get points for this cluster
            cluster_indices = np.where(labels == label)[0]
            cluster_cloud = object_cloud.select_by_index(cluster_indices)

            # Calculate object properties
            points = np.asarray(cluster_cloud.points)
            centroid = np.mean(points, axis=0)
            size = np.max(points, axis=0) - np.min(points, axis=0)

            objects.append({
                'centroid': centroid,
                'size': size,
                'point_count': len(cluster_indices),
                'cloud': cluster_cloud
            })

        return objects

    def estimate_surface_normals(self, pointcloud_msg):
        """Estimate surface normals for point cloud."""
        plane_cloud, object_cloud = self.process_pointcloud(pointcloud_msg)

        if object_cloud is not None:
            # Estimate normals
            object_cloud.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
            )

            # Orient normals consistently
            object_cloud.orient_normals_to_align_with_direction()

            return np.asarray(object_cloud.normals)

        return np.array([])
```

## Deep Learning-Based Vision

### Object Detection with YOLO Integration

```python
import torch
import torchvision.transforms as transforms
from PIL import Image
import numpy as np
import cv2

class YOLODetector:
    def __init__(self, model_path=None):
        # Load YOLO model (using torchvision's implementation as an example)
        # In practice, you might use ultralytics yolov5/yolov8
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
        self.model.eval()

        # COCO dataset class names
        self.class_names = [
            'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train',
            'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign',
            'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep',
            'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella',
            'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard',
            'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard',
            'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup', 'fork',
            'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange',
            'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair',
            'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv',
            'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 'microwave',
            'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase',
            'scissors', 'teddy bear', 'hair drier', 'toothbrush'
        ]

    def detect_objects(self, image):
        """Detect objects in image using YOLO."""
        # Convert OpenCV image (BGR) to PIL (RGB)
        if isinstance(image, np.ndarray):
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            pil_image = Image.fromarray(image_rgb)
        else:
            pil_image = image

        # Perform inference
        results = self.model(pil_image)

        # Extract detections
        detections = []
        for *xyxy, conf, cls in results.xyxy[0].tolist():
            x1, y1, x2, y2 = map(int, xyxy)
            class_id = int(cls)
            confidence = float(conf)

            detections.append({
                'bbox': [x1, y1, x2, y2],
                'class_id': class_id,
                'class_name': self.class_names[class_id] if class_id < len(self.class_names) else f'unknown_{class_id}',
                'confidence': confidence,
                'center': ((x1 + x2) // 2, (y1 + y2) // 2),
                'area': (x2 - x1) * (y2 - y1)
            })

        return detections

    def draw_detections(self, image, detections, confidence_threshold=0.5):
        """Draw detection results on image."""
        output_image = image.copy()

        for detection in detections:
            if detection['confidence'] > confidence_threshold:
                x1, y1, x2, y2 = detection['bbox']

                # Draw bounding box
                cv2.rectangle(output_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

                # Draw label
                label = f"{detection['class_name']}: {detection['confidence']:.2f}"
                cv2.putText(
                    output_image,
                    label,
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1
                )

                # Draw center
                center_x, center_y = detection['center']
                cv2.circle(output_image, (center_x, center_y), 3, (0, 0, 255), -1)

        return output_image
```

## Isaac ROS Vision Integration

### Isaac ROS Image Pipeline

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class IsaacVisionNode(Node):
    def __init__(self):
        super().__init__('isaac_vision_node')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Create subscriber for Isaac Sim camera
        self.image_sub = self.create_subscription(
            Image,
            '/front_stereo_camera/left/image_rect_color',
            self.image_callback,
            10
        )

        # Create publisher for processed image
        self.processed_pub = self.create_publisher(
            Image,
            '/isaac_vision/processed_image',
            10
        )

        # Initialize Isaac-specific vision components
        self.initialize_isaac_vision()

        self.get_logger().info('Isaac Vision Node initialized')

    def initialize_isaac_vision(self):
        """Initialize Isaac-specific vision processing."""
        # Set up Isaac ROS vision components
        # This might include:
        # - Isaac ROS apriltag detection
        # - Isaac ROS stereo processing
        # - Isaac ROS image rectification
        pass

    def image_callback(self, msg):
        """Process Isaac Sim camera image."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
            return

        # Isaac-specific image processing
        processed_image = self.process_isaac_image(cv_image)

        # Publish processed image
        try:
            processed_msg = self.bridge.cv2_to_imgmsg(processed_image, "bgr8")
            processed_msg.header = msg.header
            self.processed_pub.publish(processed_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing processed image: {e}')

    def process_isaac_image(self, image):
        """Process image with Isaac-specific algorithms."""
        # Example: Apply Isaac-specific image enhancements
        # This could include:
        # - Lens distortion correction optimized for Isaac Sim
        # - Exposure compensation for simulated lighting
        # - Color correction for realistic rendering

        # Placeholder: Apply basic enhancement
        enhanced = cv2.detailEnhance(image, sigma_s=10, sigma_r=0.15)

        # Add Isaac branding or processing indicators
        cv2.putText(
            enhanced,
            'Isaac Vision Processing',
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (255, 255, 255),
            2
        )

        return enhanced

def main(args=None):
    rclpy.init(args=args)
    isaac_vision_node = IsaacVisionNode()

    try:
        rclpy.spin(isaac_vision_node)
    except KeyboardInterrupt:
        pass
    finally:
        isaac_vision_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Vision-Based Navigation

### Visual Servoing

```python
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class VisualServoController:
    def __init__(self):
        self.kp_linear = 0.5  # Linear velocity gain
        self.kp_angular = 1.0  # Angular velocity gain
        self.target_x = 320  # Target x-coordinate (center of 640px image)
        self.target_y = 240  # Target y-coordinate (center of 480px image)
        self.target_tolerance = 10  # Tolerance for reaching target

    def calculate_control(self, object_position):
        """Calculate velocity commands based on object position."""
        # Calculate error from target position
        error_x = object_position.x - self.target_x
        error_y = object_position.y - self.target_y

        # Calculate control outputs
        linear_vel = self.kp_linear * (self.target_y - object_position.y) / self.target_y
        angular_vel = -self.kp_angular * error_x / self.target_x

        # Limit velocities
        linear_vel = np.clip(linear_vel, -0.5, 0.5)
        angular_vel = np.clip(angular_vel, -1.0, 1.0)

        # Check if target is reached
        distance_to_target = np.sqrt(error_x**2 + error_y**2)
        target_reached = distance_to_target < self.target_tolerance

        return linear_vel, angular_vel, target_reached

    def create_twist_message(self, linear_vel, angular_vel):
        """Create Twist message from velocities."""
        twist_msg = Twist()
        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        return twist_msg
```

## Lab Instructions

### Exercise 1: Camera Calibration

1. Print and use a chessboard calibration pattern
2. Capture multiple images of the chessboard from different angles
3. Implement the camera calibration algorithm
4. Verify the calibration by checking reprojected points

### Exercise 2: Object Detection Pipeline

1. Create a ROS 2 node that processes camera images
2. Implement color-based object detection
3. Add capability to detect multiple objects
4. Test the system with various lighting conditions

### Exercise 3: 3D Vision Processing

1. Set up a depth camera in simulation
2. Implement point cloud processing for object segmentation
3. Create a system that can distinguish between different objects
4. Integrate the 3D vision system with navigation

## Troubleshooting Vision Systems

### Common Issues and Solutions

1. **Poor Detection Accuracy**:
   - Improve lighting conditions
   - Adjust color thresholds or retrain models
   - Use better quality cameras

2. **Performance Issues**:
   - Optimize algorithms for real-time processing
   - Reduce image resolution if possible
   - Use GPU acceleration for deep learning models

3. **Calibration Problems**:
   - Ensure good calibration pattern visibility
   - Use sufficient number of calibration images
   - Verify camera mounting stability

## Learning Objectives

After completing this chapter, you should be able to:
- Implement camera calibration procedures for robotic vision systems
- Create robust object detection pipelines using both traditional and deep learning methods
- Process point clouds for 3D scene understanding
- Integrate vision systems with robot navigation and control
- Apply Isaac ROS vision packages for enhanced perception
- Design visual servoing controllers for robot manipulation
- Troubleshoot common vision system issues in robotics applications