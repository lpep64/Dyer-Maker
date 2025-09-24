#!/usr/bin/env python3
"""
Vision Interface Module for Dyer-Maker Digital Twin

This module provides a modular interface for computer vision algorithms,
allowing easy swapping of different detection methods while maintaining
consistent API for the rest of the system.

Author: Dyer-Maker Team
License: MIT
"""

from abc import ABC, abstractmethod
from typing import Dict, List, Tuple, Optional
import numpy as np
import cv2
from dataclasses import dataclass
from enum import Enum

class PartColor(Enum):
    """Enumeration of detectable part colors"""
    RED = "red"
    GREEN = "green"
    BLUE = "blue"
    UNKNOWN = "unknown"

@dataclass
class PartDetection:
    """Data class for part detection results"""
    color: PartColor
    position: Tuple[float, float]  # (x, y) in image coordinates
    confidence: float  # 0.0 to 1.0
    timestamp: float
    bounding_box: Tuple[int, int, int, int]  # (x, y, width, height)
    world_position: Optional[Tuple[float, float, float]] = None  # (x, y, z) in world coordinates

class VisionInterface(ABC):
    """Abstract base class for vision processing algorithms"""
    
    @abstractmethod
    def configure(self, config: Dict) -> bool:
        """
        Configure the vision algorithm with parameters
        
        Args:
            config: Dictionary containing configuration parameters
            
        Returns:
            bool: True if configuration successful, False otherwise
        """
        pass
    
    @abstractmethod
    def process_frame(self, frame: np.ndarray) -> List[PartDetection]:
        """
        Process a single frame and return detected parts
        
        Args:
            frame: Input image frame as numpy array
            
        Returns:
            List of PartDetection objects
        """
        pass
    
    @abstractmethod
    def get_diagnostics(self) -> Dict:
        """
        Get diagnostic information about the vision system
        
        Returns:
            Dictionary containing diagnostic data
        """
        pass
    
    @abstractmethod
    def calibrate(self, calibration_data: Dict) -> bool:
        """
        Calibrate the vision system with provided data
        
        Args:
            calibration_data: Dictionary containing calibration parameters
            
        Returns:
            bool: True if calibration successful, False otherwise
        """
        pass

class HSVColorDetector(VisionInterface):
    """
    HSV-based color detection implementation
    
    This is the default implementation using HSV color space filtering
    for robust color detection under varying lighting conditions.
    """
    
    def __init__(self):
        """Initialize the HSV color detector with default parameters"""
        self.color_ranges = {
            PartColor.RED: {
                'lower1': np.array([0, 50, 50]),    # Lower red range
                'upper1': np.array([10, 255, 255]),
                'lower2': np.array([170, 50, 50]),  # Upper red range
                'upper2': np.array([180, 255, 255])
            },
            PartColor.GREEN: {
                'lower': np.array([40, 50, 50]),
                'upper': np.array([80, 255, 255])
            },
            PartColor.BLUE: {
                'lower': np.array([100, 50, 50]),
                'upper': np.array([130, 255, 255])
            }
        }
        
        # Processing parameters
        self.min_contour_area = 500
        self.max_contour_area = 10000
        self.confidence_threshold = 0.7
        self.gaussian_blur_kernel = (5, 5)
        self.morphology_kernel_size = 5
        
        # Diagnostics
        self.frame_count = 0
        self.detection_count = 0
        self.processing_times = []
        
    def configure(self, config: Dict) -> bool:
        """Configure HSV color detector parameters"""
        try:
            # Update color ranges if provided
            if 'color_ranges' in config:
                for color_name, ranges in config['color_ranges'].items():
                    if hasattr(PartColor, color_name.upper()):
                        color_enum = PartColor(color_name.lower())
                        if color_enum in self.color_ranges:
                            self.color_ranges[color_enum].update(ranges)
            
            # Update processing parameters
            if 'min_contour_area' in config:
                self.min_contour_area = config['min_contour_area']
            if 'max_contour_area' in config:
                self.max_contour_area = config['max_contour_area']
            if 'confidence_threshold' in config:
                self.confidence_threshold = config['confidence_threshold']
            if 'gaussian_blur_kernel' in config:
                self.gaussian_blur_kernel = tuple(config['gaussian_blur_kernel'])
            if 'morphology_kernel_size' in config:
                self.morphology_kernel_size = config['morphology_kernel_size']
                
            return True
        except Exception as e:
            print(f"Configuration error: {e}")
            return False
    
    def process_frame(self, frame: np.ndarray) -> List[PartDetection]:
        """Process frame using HSV color detection"""
        import time
        start_time = time.time()
        
        detections = []
        self.frame_count += 1
        
        if frame is None or frame.size == 0:
            return detections
        
        # Convert to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(hsv, self.gaussian_blur_kernel, 0)
        
        # Detect each color
        for color in [PartColor.RED, PartColor.GREEN, PartColor.BLUE]:
            color_detections = self._detect_color(blurred, color, frame.shape)
            detections.extend(color_detections)
        
        # Sort detections by confidence
        detections.sort(key=lambda x: x.confidence, reverse=True)
        
        # Update diagnostics
        self.detection_count += len(detections)
        processing_time = time.time() - start_time
        self.processing_times.append(processing_time)
        if len(self.processing_times) > 100:  # Keep last 100 measurements
            self.processing_times.pop(0)
        
        return detections
    
    def _detect_color(self, hsv_frame: np.ndarray, color: PartColor, frame_shape: Tuple) -> List[PartDetection]:
        """Detect specific color in HSV frame"""
        detections = []
        
        if color not in self.color_ranges:
            return detections
        
        ranges = self.color_ranges[color]
        
        # Create color mask
        if color == PartColor.RED:
            # Red wraps around in HSV, so we need two ranges
            mask1 = cv2.inRange(hsv_frame, ranges['lower1'], ranges['upper1'])
            mask2 = cv2.inRange(hsv_frame, ranges['lower2'], ranges['upper2'])
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            mask = cv2.inRange(hsv_frame, ranges['lower'], ranges['upper'])
        
        # Apply morphological operations to clean up the mask
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, 
                                         (self.morphology_kernel_size, self.morphology_kernel_size))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            
            # Filter by area
            if self.min_contour_area <= area <= self.max_contour_area:
                # Calculate bounding rectangle
                x, y, w, h = cv2.boundingRect(contour)
                
                # Calculate center position
                center_x = x + w // 2
                center_y = y + h // 2
                
                # Calculate confidence based on contour properties
                confidence = self._calculate_confidence(contour, area, mask[y:y+h, x:x+w])
                
                if confidence >= self.confidence_threshold:
                    detection = PartDetection(
                        color=color,
                        position=(center_x, center_y),
                        confidence=confidence,
                        timestamp=time.time(),
                        bounding_box=(x, y, w, h)
                    )
                    detections.append(detection)
        
        return detections
    
    def _calculate_confidence(self, contour: np.ndarray, area: float, mask_roi: np.ndarray) -> float:
        """Calculate detection confidence based on contour properties"""
        # Base confidence from area (normalized)
        area_confidence = min(area / self.max_contour_area, 1.0)
        
        # Shape confidence (how circular/rectangular the contour is)
        perimeter = cv2.arcLength(contour, True)
        if perimeter > 0:
            circularity = 4 * np.pi * area / (perimeter * perimeter)
            shape_confidence = min(circularity * 2, 1.0)  # Boost circular shapes
        else:
            shape_confidence = 0.0
        
        # Fill confidence (how well the mask fills the bounding box)
        if mask_roi.size > 0:
            fill_ratio = np.sum(mask_roi > 0) / mask_roi.size
            fill_confidence = fill_ratio
        else:
            fill_confidence = 0.0
        
        # Weighted average of confidence factors
        total_confidence = (0.4 * area_confidence + 
                          0.3 * shape_confidence + 
                          0.3 * fill_confidence)
        
        return min(total_confidence, 1.0)
    
    def get_diagnostics(self) -> Dict:
        """Get diagnostic information"""
        avg_processing_time = (sum(self.processing_times) / len(self.processing_times) 
                             if self.processing_times else 0.0)
        
        detection_rate = (self.detection_count / self.frame_count 
                         if self.frame_count > 0 else 0.0)
        
        return {
            'algorithm': 'HSV_Color_Detector',
            'frame_count': self.frame_count,
            'detection_count': self.detection_count,
            'detection_rate': detection_rate,
            'avg_processing_time_ms': avg_processing_time * 1000,
            'color_ranges': {color.value: ranges for color, ranges in self.color_ranges.items()},
            'parameters': {
                'min_contour_area': self.min_contour_area,
                'max_contour_area': self.max_contour_area,
                'confidence_threshold': self.confidence_threshold,
                'gaussian_blur_kernel': self.gaussian_blur_kernel,
                'morphology_kernel_size': self.morphology_kernel_size
            }
        }
    
    def calibrate(self, calibration_data: Dict) -> bool:
        """Calibrate color ranges based on sample data"""
        try:
            # This would implement automatic color range calibration
            # based on user-provided samples or training data
            
            if 'sample_images' in calibration_data:
                # Process sample images to determine optimal color ranges
                # This is a placeholder for more sophisticated calibration
                print("Automatic calibration from sample images not yet implemented")
                return False
            
            if 'manual_ranges' in calibration_data:
                # Update color ranges with manually specified values
                return self.configure({'color_ranges': calibration_data['manual_ranges']})
            
            return True
            
        except Exception as e:
            print(f"Calibration error: {e}")
            return False

class VisionFactory:
    """Factory class for creating vision processors"""
    
    @staticmethod
    def create_processor(algorithm_type: str = "hsv_color") -> VisionInterface:
        """
        Create a vision processor of the specified type
        
        Args:
            algorithm_type: Type of algorithm to create
            
        Returns:
            VisionInterface implementation
        """
        if algorithm_type.lower() == "hsv_color":
            return HSVColorDetector()
        else:
            raise ValueError(f"Unknown algorithm type: {algorithm_type}")
    
    @staticmethod
    def get_available_algorithms() -> List[str]:
        """Get list of available vision algorithms"""
        return ["hsv_color"]