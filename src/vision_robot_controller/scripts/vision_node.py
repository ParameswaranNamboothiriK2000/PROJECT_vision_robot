#!/usr/bin/env python3
import os
os.environ["DISPLAY"] = ":0"
os.environ["QT_QPA_PLATFORM"] = "xcb"

"""
=============================================================
 PART 2 — vision_node.py
 Project: Autonomous Vision-to-Motion Robotic Inspection
 Node   : YOLO11 Vision Node (Python)
 Role   : Detects target object from webcam → publishes
          3D workspace coordinates to ROS 2 topic
 Topic  : Publishes  → /target_pose  (geometry_msgs/Point)
          Publishes  → /vision_status (std_msgs/String)
 Author : Vision Robot Project
=============================================================
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String

import cv2
import numpy as np
from ultralytics import YOLO
from collections import deque


# =============================================================
#  CONFIGURATION — Change these to match your setup
# =============================================================

# Target object to track (YOLO COCO class name)
# Common options: 'bottle', 'cup', 'cell phone', 'person', 'ball'
TARGET_CLASS = 'bottle'

# Camera index (0 = default webcam, 1 = external USB camera)
CAMERA_INDEX = 0

# YOLO model — yolo11n.pt is fastest (nano), good for GTX 1650
YOLO_MODEL = 'yolo11n.pt'

# Moving average filter window size (higher = smoother but slower)
SMOOTHING_WINDOW = 8

# Camera intrinsic parameters (approximate for standard 1080p webcam)
# These convert pixels → meters in 3D workspace
# Focal length in pixels (tune based on your webcam)
FOCAL_LENGTH_PX = 600.0

# Real-world height of target object in meters (bottle ≈ 0.25m)
TARGET_REAL_HEIGHT_M = 0.25

# Robot workspace bounds (meters) — Panda arm safe reach
WORKSPACE_X_MIN, WORKSPACE_X_MAX = 0.2, 0.7   # forward reach
WORKSPACE_Y_MIN, WORKSPACE_Y_MAX = -0.4, 0.4  # left-right
WORKSPACE_Z = 0.3                              # fixed height above table

# Publish rate in Hz
PUBLISH_RATE_HZ = 15.0

# Minimum confidence threshold for detection
CONFIDENCE_THRESHOLD = 0.50

# =============================================================


class VisionNode(Node):
    """
    YOLO11 Vision Node for ROS 2.

    Pipeline:
    1. Capture frame from webcam via OpenCV
    2. Run YOLO11n inference (GPU-accelerated on GTX 1650)
    3. Filter detections → keep only TARGET_CLASS above threshold
    4. Pick highest-confidence detection
    5. Convert bounding box center (pixels) → 3D workspace (meters)
    6. Apply moving average filter to smooth coordinates
    7. Clamp to robot workspace safety bounds
    8. Publish geometry_msgs/Point on /target_pose
    """

    def __init__(self):
        super().__init__('vision_node')

        # ----- ROS 2 Publishers -----
        self.target_publisher = self.create_publisher(
            Point,
            '/target_pose',
            10  # QoS queue depth
        )
        self.status_publisher = self.create_publisher(
            String,
            '/vision_status',
            10
        )

        # ----- Timer: main processing loop -----
        self.timer = self.create_timer(
            1.0 / PUBLISH_RATE_HZ,
            self.timer_callback
        )

        # ----- Load YOLO11 model -----
        self.get_logger().info(f'Loading YOLO11 model: {YOLO_MODEL}')
        self.model = YOLO(YOLO_MODEL)
        # Force GPU inference (CUDA) — falls back to CPU if unavailable
        self.device = 'cuda' if self._check_cuda() else 'cpu'
        self.get_logger().info(f'YOLO11 running on: {self.device.upper()}')

        # ----- OpenCV camera capture -----
        self.get_logger().info(f'Opening camera index: {CAMERA_INDEX}')
        self.cap = cv2.VideoCapture(CAMERA_INDEX)
        if not self.cap.isOpened():
            self.get_logger().error('Cannot open camera! Check CAMERA_INDEX.')
            raise RuntimeError('Camera not available')

        # Get camera frame dimensions
        self.frame_width  = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(
            f'Camera opened: {self.frame_width}x{self.frame_height}'
        )

        # ----- Moving Average Filter buffers -----
        # Separate deques for X, Y, Z — each holds last N values
        self.x_buffer = deque(maxlen=SMOOTHING_WINDOW)
        self.y_buffer = deque(maxlen=SMOOTHING_WINDOW)
        self.z_buffer = deque(maxlen=SMOOTHING_WINDOW)

        # ----- State tracking -----
        self.detection_count    = 0   # total detections published
        self.no_detection_count = 0   # consecutive frames with no target
        self.last_valid_point   = None

        self.get_logger().info(
            f'\n'
            f'╔══════════════════════════════════════════╗\n'
            f'║  Vision Node READY                      ║\n'
            f'║  Target class : {TARGET_CLASS:<24s}║\n'
            f'║  Model        : {YOLO_MODEL:<24s}║\n'
            f'║  Device       : {self.device.upper():<24s}║\n'
            f'║  Publishing   : /target_pose            ║\n'
            f'╚══════════════════════════════════════════╝'
        )

    # ----------------------------------------------------------
    #  MAIN TIMER CALLBACK — runs at PUBLISH_RATE_HZ
    # ----------------------------------------------------------
    def timer_callback(self):
        # Step 1: Read frame from camera
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to read camera frame')
            return

        # Step 2: Run YOLO11 inference
        results = self.model(
            frame,
            device=self.device,
            conf=CONFIDENCE_THRESHOLD,
            verbose=False   # suppress per-frame console spam
        )

        # Step 3: Find best detection of TARGET_CLASS
        best_box  = None
        best_conf = 0.0
        best_bbox_h = 0

        for result in results:
            for box in result.boxes:
                # Get class name from YOLO model
                class_id   = int(box.cls[0])
                class_name = self.model.names[class_id]
                confidence = float(box.conf[0])

                if class_name == TARGET_CLASS and confidence > best_conf:
                    best_conf   = confidence
                    best_box    = box.xyxy[0].cpu().numpy()  # [x1,y1,x2,y2]
                    best_bbox_h = int(best_box[3]) - int(best_box[1])

        # Step 4: Convert detection to 3D coordinates
        if best_box is not None:
            # Bounding box center in pixels
            cx_px = (best_box[0] + best_box[2]) / 2.0
            cy_px = (best_box[1] + best_box[3]) / 2.0

            # 2D → 3D projection using pinhole camera model
            # Estimate distance Z from apparent height of bounding box
            # Z = (real_height * focal_length) / bbox_height_pixels
            if best_bbox_h > 0:
                z_world = (TARGET_REAL_HEIGHT_M * FOCAL_LENGTH_PX) / best_bbox_h
            else:
                z_world = 0.5  # fallback distance

            # Convert pixel offsets from image center → meters
            # x_world = forward distance (Z from camera)
            # y_world = lateral offset (left/right)
            img_cx  = self.frame_width  / 2.0
            img_cy  = self.frame_height / 2.0

            x_world = z_world                              # depth = forward
            y_world = -(cx_px - img_cx) / FOCAL_LENGTH_PX * z_world  # lateral
            z_world_arm = WORKSPACE_Z                      # fixed table height

            # Step 5: Apply moving average filter
            self.x_buffer.append(x_world)
            self.y_buffer.append(y_world)
            self.z_buffer.append(z_world_arm)

            x_smooth = float(np.mean(self.x_buffer))
            y_smooth = float(np.mean(self.y_buffer))
            z_smooth = float(np.mean(self.z_buffer))

            # Step 6: Clamp to robot workspace safety bounds
            x_safe = float(np.clip(x_smooth, WORKSPACE_X_MIN, WORKSPACE_X_MAX))
            y_safe = float(np.clip(y_smooth, WORKSPACE_Y_MIN, WORKSPACE_Y_MAX))
            z_safe = float(np.clip(z_smooth, 0.1, 0.8))

            # Step 7: Build and publish Point message
            point_msg       = Point()
            point_msg.x     = x_safe
            point_msg.y     = y_safe
            point_msg.z     = z_safe
            self.target_publisher.publish(point_msg)

            # Publish status
            status_msg      = String()
            status_msg.data = (
                f'DETECTED | class={TARGET_CLASS} | '
                f'conf={best_conf:.2f} | '
                f'pos=({x_safe:.3f}, {y_safe:.3f}, {z_safe:.3f})'
            )
            self.status_publisher.publish(status_msg)

            self.detection_count += 1
            self.no_detection_count = 0
            self.last_valid_point = (x_safe, y_safe, z_safe)

            # Log every 30 detections to avoid flooding
            if self.detection_count % 30 == 0:
                self.get_logger().info(
                    f'[{self.detection_count}] Target @ '
                    f'x={x_safe:.3f}m  y={y_safe:.3f}m  '
                    f'z={z_safe:.3f}m  conf={best_conf:.2f}'
                )

        else:
            # No target detected in this frame
            self.no_detection_count += 1

            status_msg      = String()
            status_msg.data = f'NO_TARGET | searching for: {TARGET_CLASS}'
            self.status_publisher.publish(status_msg)

            if self.no_detection_count % 30 == 0:
                self.get_logger().warn(
                    f'No "{TARGET_CLASS}" detected for '
                    f'{self.no_detection_count} frames'
                )

        # Step 8: Draw debug overlay on frame and show window
        self._draw_debug_overlay(frame, best_box, best_conf)

    # ----------------------------------------------------------
    #  DEBUG OVERLAY — draws bounding box + info on frame
    # ----------------------------------------------------------
    def _draw_debug_overlay(self, frame, best_box, confidence):
        """Draw detection overlay on the camera frame."""
        overlay = frame.copy()

        if best_box is not None:
            x1, y1, x2, y2 = [int(v) for v in best_box]

            # Green bounding box
            cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Label background
            label = f'{TARGET_CLASS} {confidence:.2f}'
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(overlay, (x1, y1-th-8), (x1+tw+4, y1), (0,255,0), -1)
            cv2.putText(overlay, label, (x1+2, y1-4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 2)

            # Center crosshair
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            cv2.drawMarker(overlay, (cx, cy), (0,0,255),
                           cv2.MARKER_CROSS, 20, 2)

            # 3D coordinate display
            if self.last_valid_point:
                x, y, z = self.last_valid_point
                coord_text = f'3D: ({x:.2f}, {y:.2f}, {z:.2f}) m'
                cv2.putText(overlay, coord_text, (10, frame.shape[0]-15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,255,255), 2)
        else:
            # Red "searching" indicator
            cv2.putText(overlay,
                        f'Searching for: {TARGET_CLASS}...',
                        (10, 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # Status bar at top
        cv2.putText(overlay,
                    f'ROS2 Vision Node | Detections: {self.detection_count}',
                    (10, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        # Show window (press Q to quit)
        cv2.imshow('Vision Node — YOLO11 Detection', overlay)
        cv2.waitKey(1)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info('Q pressed — shutting down vision node')
            self.destroy_node()

    # ----------------------------------------------------------
    #  UTILITY
    # ----------------------------------------------------------
    def _check_cuda(self):
        """Check if CUDA (GPU) is available for YOLO inference."""
        try:
            import torch
            return torch.cuda.is_available()
        except ImportError:
            return False

    def destroy_node(self):
        """Clean up camera and windows on shutdown."""
        self.get_logger().info('Releasing camera and closing windows...')
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


# =============================================================
#  MAIN ENTRY POINT
# =============================================================
def main(args=None):
    rclpy.init(args=args)

    try:
        vision_node = VisionNode()
        rclpy.spin(vision_node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        print(f'[ERROR] {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
