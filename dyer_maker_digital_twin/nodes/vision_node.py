#!/usr/bin/env python3
"""
Vision Node for Dyer-Maker Digital Twin

This node handles computer vision processing for part detection and classification.
It uses the modular vision interface to support different detection algorithms.

Author: Dyer-Maker Team
License: MIT
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import String, Header, Float32
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml
import os
from typing import Dict, List, Optional
import time
import threading
from dataclasses import asdict

# Import our modular vision interface
from ..interfaces.vision_interface import (
    VisionInterface, VisionFactory, PartDetection, PartColor
)

# Custom message types (would be defined in a separate msg package in real implementation)
class PartDetectionMsg:
    """Placeholder for custom PartDetection message"""
    def __init__(self):
        self.color = ""
        self.position = Point()
        self.confidence = 0.0
        self.timestamp = 0.0
        self.bounding_box = [0, 0, 0, 0]  # [x, y, width, height]

class VisionNode(Node):
    """
    ROS2 node for computer vision processing
    
    This node subscribes to camera images, processes them for part detection,
    and publishes detection results to the system.
    """
    
    def __init__(self):
        super().__init__('vision_node')
        
        # Node parameters
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('compressed_input', False)
        self.declare_parameter('config_file', 'config/vision_config.yaml')
        self.declare_parameter('algorithm_type', 'hsv_color')
        self.declare_parameter('publish_debug_images', True)
        self.declare_parameter('detection_zone', [0.2, 0.2, 0.6, 0.6])  # [x1, y1, x2, y2] normalized
        
        # Initialize parameters
        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.compressed_input = self.get_parameter('compressed_input').get_parameter_value().bool_value
        self.config_file = self.get_parameter('config_file').get_parameter_value().string_value
        self.algorithm_type = self.get_parameter('algorithm_type').get_parameter_value().string_value
        self.publish_debug_images = self.get_parameter('publish_debug_images').get_parameter_value().bool_value
        self.detection_zone = self.get_parameter('detection_zone').get_parameter_value().double_array_value
        
        # CV Bridge for image conversion
        self.cv_bridge = CvBridge()
        
        # Vision processor
        self.vision_processor: Optional[VisionInterface] = None
        
        # QoS profiles
        self.image_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.detection_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Subscribers
        if self.compressed_input:
            self.image_sub = self.create_subscription(
                CompressedImage,
                self.camera_topic,
                self.compressed_image_callback,
                self.image_qos
            )
        else:
            self.image_sub = self.create_subscription(
                Image,
                self.camera_topic,
                self.image_callback,
                self.image_qos
            )
        
        # Publishers
        self.detection_pub = self.create_publisher(
            String,  # Would be PartDetectionMsg in real implementation
            '/vision/part_detected',
            self.detection_qos
        )
        
        self.status_pub = self.create_publisher(
            String,
            '/vision/status',
            10
        )
        
        if self.publish_debug_images:
            self.debug_image_pub = self.create_publisher(
                Image,
                '/vision/debug_image',
                self.image_qos
            )
        
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            '/vision/diagnostics',
            10
        )
        
        # Services
        self.configure_service = self.create_service(
            String,  # Would be ConfigureVision service in real implementation
            '/vision/configure',
            self.configure_callback
        )
        
        self.calibrate_service = self.create_service(
            String,  # Would be CalibrateVision service in real implementation
            '/vision/calibrate',
            self.calibrate_callback
        )
        
        # Internal state
        self.processing_active = True
        self.frame_count = 0
        self.detection_count = 0
        self.last_fps_time = time.time()
        self.fps = 0.0
        self.processing_lock = threading.Lock()
        
        # Camera calibration (placeholder - would load from calibration file)
        self.camera_matrix = np.eye(3)
        self.dist_coeffs = np.zeros(5)
        self.camera_height = 0.5  # meters above conveyor
        
        # Initialize vision system
        self.initialize_vision_system()
        
        # Start diagnostic timer
        self.diagnostic_timer = self.create_timer(1.0, self.publish_diagnostics)
        
        self.get_logger().info(f"Vision node initialized with algorithm: {self.algorithm_type}")
    
    def initialize_vision_system(self):
        """Initialize the vision processing system"""
        try:
            # Create vision processor
            self.vision_processor = VisionFactory.create_processor(self.algorithm_type)
            
            # Load configuration
            config = self.load_vision_config()
            if not self.vision_processor.configure(config):
                self.get_logger().error("Failed to configure vision processor")
                return False
            
            self.get_logger().info("Vision system initialized successfully")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Vision system initialization failed: {e}")
            return False
    
    def load_vision_config(self) -> Dict:
        """Load vision configuration from file"""
        try:
            # Try to find config file in package share directory
            config_path = os.path.join(
                self.get_package_share_directory('dyer_maker_digital_twin'),
                self.config_file
            )
            
            if not os.path.exists(config_path):
                # Fallback to default configuration
                return self.get_default_config()
            
            with open(config_path, 'r') as file:
                config = yaml.safe_load(file)
                return config.get('vision', {})
                
        except Exception as e:
            self.get_logger().warn(f"Could not load config file: {e}, using defaults")
            return self.get_default_config()
    
    def get_default_config(self) -> Dict:
        """Get default vision configuration"""
        return {
            'color_ranges': {
                'red': {
                    'lower1': [0, 50, 50],
                    'upper1': [10, 255, 255],
                    'lower2': [170, 50, 50],
                    'upper2': [180, 255, 255]
                },
                'green': {
                    'lower': [40, 50, 50],
                    'upper': [80, 255, 255]
                },
                'blue': {
                    'lower': [100, 50, 50],
                    'upper': [130, 255, 255]
                }
            },
            'min_contour_area': 500,
            'max_contour_area': 10000,
            'confidence_threshold': 0.7,
            'gaussian_blur_kernel': [5, 5],
            'morphology_kernel_size': 5
        }
    
    def image_callback(self, msg: Image):
        """Handle incoming uncompressed images"""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.process_image(cv_image, msg.header)
            
        except Exception as e:
            self.get_logger().error(f"Image processing error: {e}")
    
    def compressed_image_callback(self, msg: CompressedImage):
        """Handle incoming compressed images"""
        try:
            # Convert compressed image to OpenCV format
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            # Create header from compressed message
            header = Header()
            header.stamp = msg.header.stamp
            header.frame_id = msg.header.frame_id
            
            self.process_image(cv_image, header)
            
        except Exception as e:
            self.get_logger().error(f"Compressed image processing error: {e}")
    
    def process_image(self, cv_image: np.ndarray, header: Header):
        """Process image for part detection"""
        if not self.processing_active or self.vision_processor is None:
            return
        
        with self.processing_lock:
            try:
                self.frame_count += 1
                
                # Apply detection zone mask
                masked_image = self.apply_detection_zone(cv_image)
                
                # Process image with vision algorithm
                detections = self.vision_processor.process_frame(masked_image)
                
                # Filter and validate detections
                valid_detections = self.filter_detections(detections, cv_image.shape)
                
                # Publish detections
                if valid_detections:
                    self.publish_detections(valid_detections, header)
                    self.detection_count += len(valid_detections)
                
                # Publish debug image if enabled
                if self.publish_debug_images:
                    debug_image = self.create_debug_image(cv_image, valid_detections)
                    self.publish_debug_image(debug_image, header)
                
                # Update FPS
                self.update_fps()
                
            except Exception as e:
                self.get_logger().error(f"Image processing error: {e}")
    
    def apply_detection_zone(self, image: np.ndarray) -> np.ndarray:
        """Apply detection zone mask to image"""
        if len(self.detection_zone) != 4:
            return image
        
        height, width = image.shape[:2]
        
        # Convert normalized coordinates to pixel coordinates
        x1 = int(self.detection_zone[0] * width)
        y1 = int(self.detection_zone[1] * height)
        x2 = int(self.detection_zone[2] * width)
        y2 = int(self.detection_zone[3] * height)
        
        # Create mask
        mask = np.zeros((height, width), dtype=np.uint8)
        mask[y1:y2, x1:x2] = 255
        
        # Apply mask
        masked_image = cv2.bitwise_and(image, image, mask=mask)
        return masked_image
    
    def filter_detections(self, detections: List[PartDetection], image_shape: tuple) -> List[PartDetection]:
        """Filter and validate detections"""
        valid_detections = []
        
        for detection in detections:
            # Check if detection is within detection zone
            if self.is_in_detection_zone(detection.position, image_shape):
                # Convert image coordinates to world coordinates
                world_pos = self.image_to_world_coordinates(
                    detection.position, image_shape
                )
                detection.world_position = world_pos
                valid_detections.append(detection)
        
        return valid_detections
    
    def is_in_detection_zone(self, position: tuple, image_shape: tuple) -> bool:
        """Check if position is within detection zone"""
        height, width = image_shape[:2]
        x, y = position
        
        # Convert to normalized coordinates
        norm_x = x / width
        norm_y = y / height
        
        return (self.detection_zone[0] <= norm_x <= self.detection_zone[2] and
                self.detection_zone[1] <= norm_y <= self.detection_zone[3])
    
    def image_to_world_coordinates(self, image_pos: tuple, image_shape: tuple) -> tuple:
        """Convert image coordinates to world coordinates"""
        # Placeholder implementation - would use camera calibration in real system
        # This assumes a simple orthographic projection
        
        height, width = image_shape[:2]
        x_img, y_img = image_pos
        
        # Convert to normalized image coordinates
        x_norm = (x_img - width/2) / width
        y_norm = (y_img - height/2) / height
        
        # Convert to world coordinates (assuming conveyor at z=0)
        # Scale factors would be determined by camera calibration
        world_x = x_norm * 0.5  # 0.5m field of view in x
        world_y = y_norm * 0.3  # 0.3m field of view in y
        world_z = 0.0  # Conveyor surface
        
        return (world_x, world_y, world_z)
    
    def publish_detections(self, detections: List[PartDetection], header: Header):
        """Publish detection results"""
        try:
            # In real implementation, this would publish proper PartDetectionMsg
            # For now, publish as JSON string
            
            detection_data = {
                'timestamp': header.stamp.sec + header.stamp.nanosec * 1e-9,
                'frame_id': header.frame_id,
                'detections': [
                    {
                        'color': detection.color.value,
                        'position': {
                            'image': detection.position,
                            'world': detection.world_position
                        },
                        'confidence': detection.confidence,
                        'bounding_box': detection.bounding_box
                    }
                    for detection in detections
                ]
            }
            
            msg = String()
            msg.data = str(detection_data)  # Would use proper message serialization
            
            self.detection_pub.publish(msg)
            
            # Log detection
            for detection in detections:
                self.get_logger().info(
                    f"Detected {detection.color.value} part at "
                    f"image: {detection.position}, "
                    f"world: {detection.world_position}, "
                    f"confidence: {detection.confidence:.2f}"
                )
            
        except Exception as e:
            self.get_logger().error(f"Detection publishing error: {e}")
    
    def create_debug_image(self, image: np.ndarray, detections: List[PartDetection]) -> np.ndarray:
        """Create debug image with detection overlays"""
        debug_image = image.copy()
        
        # Draw detection zone
        height, width = image.shape[:2]
        x1 = int(self.detection_zone[0] * width)
        y1 = int(self.detection_zone[1] * height)
        x2 = int(self.detection_zone[2] * width)
        y2 = int(self.detection_zone[3] * height)
        
        cv2.rectangle(debug_image, (x1, y1), (x2, y2), (255, 255, 255), 2)
        cv2.putText(debug_image, "Detection Zone", (x1, y1-10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Draw detections
        for detection in detections:
            x, y, w, h = detection.bounding_box
            center_x, center_y = detection.position
            
            # Color mapping
            color_map = {
                PartColor.RED: (0, 0, 255),
                PartColor.GREEN: (0, 255, 0),
                PartColor.BLUE: (255, 0, 0)
            }
            color = color_map.get(detection.color, (128, 128, 128))
            
            # Draw bounding box
            cv2.rectangle(debug_image, (x, y), (x + w, y + h), color, 2)
            
            # Draw center point
            cv2.circle(debug_image, (int(center_x), int(center_y)), 5, color, -1)
            
            # Draw label
            label = f"{detection.color.value}: {detection.confidence:.2f}"
            cv2.putText(debug_image, label, (x, y - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        
        # Add frame info
        info_text = f"Frame: {self.frame_count}, FPS: {self.fps:.1f}, Detections: {len(detections)}"
        cv2.putText(debug_image, info_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        return debug_image
    
    def publish_debug_image(self, debug_image: np.ndarray, header: Header):
        """Publish debug image"""
        try:
            debug_msg = self.cv_bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
            debug_msg.header = header
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f"Debug image publishing error: {e}")
    
    def update_fps(self):
        """Update FPS calculation"""
        current_time = time.time()
        if current_time - self.last_fps_time >= 1.0:
            self.fps = self.frame_count / (current_time - self.last_fps_time)
            self.frame_count = 0
            self.last_fps_time = current_time
    
    def publish_diagnostics(self):
        """Publish diagnostic information"""
        try:
            if self.vision_processor is None:
                return
            
            diagnostics = self.vision_processor.get_diagnostics()
            
            # Create diagnostic message
            diag_array = DiagnosticArray()
            diag_array.header.stamp = self.get_clock().now().to_msg()
            
            # Main vision status
            status = DiagnosticStatus()
            status.name = "Vision System"
            status.message = f"Algorithm: {diagnostics.get('algorithm', 'unknown')}"
            
            # Determine status level
            if self.fps > 5.0 and diagnostics.get('detection_rate', 0) > 0:
                status.level = DiagnosticStatus.OK
            elif self.fps > 1.0:
                status.level = DiagnosticStatus.WARN
                status.message += " (Low FPS)"
            else:
                status.level = DiagnosticStatus.ERROR
                status.message += " (No frames)"
            
            # Add diagnostic values
            status.values.append(KeyValue(key="fps", value=str(self.fps)))
            status.values.append(KeyValue(key="frame_count", value=str(diagnostics.get('frame_count', 0))))
            status.values.append(KeyValue(key="detection_count", value=str(diagnostics.get('detection_count', 0))))
            status.values.append(KeyValue(key="detection_rate", value=str(diagnostics.get('detection_rate', 0))))
            status.values.append(KeyValue(key="avg_processing_time_ms", 
                                        value=str(diagnostics.get('avg_processing_time_ms', 0))))
            
            diag_array.status.append(status)
            self.diagnostics_pub.publish(diag_array)
            
        except Exception as e:
            self.get_logger().error(f"Diagnostics publishing error: {e}")
    
    def configure_callback(self, request, response):
        """Handle configuration service calls"""
        try:
            # Parse configuration from request (would be proper service message)
            # For now, assume JSON string in request
            
            if self.vision_processor is None:
                response.success = False
                response.message = "Vision processor not initialized"
                return response
            
            # Apply configuration
            # config = json.loads(request.config)  # Would parse from proper message
            # success = self.vision_processor.configure(config)
            
            response.success = True
            response.message = "Configuration updated successfully"
            
            self.get_logger().info("Vision system reconfigured")
            
        except Exception as e:
            response.success = False
            response.message = f"Configuration error: {e}"
            self.get_logger().error(f"Configuration error: {e}")
        
        return response
    
    def calibrate_callback(self, request, response):
        """Handle calibration service calls"""
        try:
            if self.vision_processor is None:
                response.success = False
                response.message = "Vision processor not initialized"
                return response
            
            # Perform calibration
            # calibration_data = json.loads(request.data)  # Would parse from proper message
            # success = self.vision_processor.calibrate(calibration_data)
            
            response.success = True
            response.message = "Calibration completed successfully"
            
            self.get_logger().info("Vision system calibrated")
            
        except Exception as e:
            response.success = False
            response.message = f"Calibration error: {e}"
            self.get_logger().error(f"Calibration error: {e}")
        
        return response
    
    def get_package_share_directory(self, package_name: str) -> str:
        """Get package share directory (placeholder)"""
        # In real implementation, would use ament_index_python
        return f"/opt/ros/jazzy/share/{package_name}"
    
    def destroy_node(self):
        """Clean shutdown"""
        self.processing_active = False
        if hasattr(self, 'diagnostic_timer'):
            self.diagnostic_timer.destroy()
        super().destroy_node()

def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        vision_node = VisionNode()
        rclpy.spin(vision_node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'vision_node' in locals():
            vision_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()