#!/usr/bin/env python3
# object_tracker_csrt_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from yolov8_msgs.msg import Yolov8Inference
from cv_bridge import CvBridge
import cv2
import time
from yolov8_msgs.msg import Yolov8Inference  # reuse message, republish single detection
import numpy as np

# Tracker node: subscribe image + yolov8_inference, init CSRT when YOLO detects, publish /camera/yolo_annotated and /object_tracker/inference

class ObjectTrackerCSRT(Node):
    def __init__(self):
        super().__init__('object_tracker_csrt')
        self.bridge = CvBridge()
        self.image = None
        self.last_detection = None
        self.tracker = None
        self.tracking = False
        self.tracking_lost_time = None
        self.tracking_lost_timeout = 1.5
        self.selected_class = None
        self.missed_frames = 0
        self.max_missed_frames = 10

        self.create_subscription(Image, '/camera/image_raw', self.image_cb, 5)
        self.create_subscription(Yolov8Inference, '/Yolov8_Inference', self.det_cb, 10)
        # pub annotated image
        self.ann_pub = self.create_publisher(Image, '/camera/yolo_annotated', 1)
        # republish filtered inference
        self.out_pub = self.create_publisher(Yolov8Inference, '/object_tracker/inference', 10)

        self.timer = self.create_timer(1.0/15.0, self.timer_cb)  # 15 Hz

    def image_cb(self,msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.image = cv_img
        except Exception as e:
            self.get_logger().warn(f"cv_bridge image error: {e}")

    def det_cb(self,msg:Yolov8Inference):
        # choose selected_class if not set; or you could read /selected_object_class
        if not getattr(msg,'yolov8_inference', None):
            return
        # take largest detection for selected_class (or first if selected_class None)
        best = None; best_area = 0
        for d in msg.yolov8_inference:
            w = max(0, d.right - d.left); h = max(0, d.bottom - d.top)
            area = w*h
            if area > best_area:
                best_area = area; best = d
        if best is not None:
            self.last_detection = best
            self.missed_frames = 0
            # if tracker not running and image available -> init
            if not self.tracking and self.image is not None:
                try:
                    x = int(best.left); y = int(best.top)
                    w = int(best.right - best.left); h = int(best.bottom - best.top)
                    bbox = (x,y,w,h)
                    self.tracker = cv2.TrackerCSRT_create()
                    ok = self.tracker.init(self.image, bbox)
                    if ok:
                        self.tracking = True
                        self.get_logger().info("CSRT tracker initialized")
                except Exception as e:
                    self.get_logger().warn(f"Tracker init failed: {e}")

    def timer_cb(self):
        annotated = None
        out_msg = Yolov8Inference()
        out_msg.yolov8_inference = []
        if self.tracking and self.image is not None:
            ok, bbox = self.tracker.update(self.image)
            if ok:
                x,y,w,h = [int(v) for v in bbox]
                # create synthetic inference message
                det = type('D',(),{})()  # lightweight struct
                det.class_name = "tracked"
                det.left = x; det.top = y; det.right = x+w; det.bottom = y+h
                out_msg.yolov8_inference.append(det)
                annotated = self.image.copy()
                cv2.rectangle(annotated,(x,y),(x+w,y+h),(0,255,0),2)
                cv2.putText(annotated,"CSRT",(x,y-5),cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,0),2)
                self.tracking_lost_time = None
            else:
                # tracker lost
                if self.tracking_lost_time is None:
                    self.tracking_lost_time = time.time()
                elif time.time() - self.tracking_lost_time > self.tracking_lost_timeout:
                    self.get_logger().info("Tracker lost -> disable tracking")
                    self.tracking = False
                    self.tracker = None
        else:
            # fallback: if last_detection exists, we can publish it as inference (so follower sees detections)
            if self.last_detection:
                det = self.last_detection
                out_msg.yolov8_inference.append(det)
                # optionally annotate
                if self.image is not None:
                    annotated = self.image.copy()
                    cv2.rectangle(annotated, (int(det.left),int(det.top)), (int(det.right),int(det.bottom)), (0,0,255),2)
        # publish annotated image if available
        if annotated is not None:
            try:
                img_msg = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
                self.ann_pub.publish(img_msg)
            except Exception as e:
                self.get_logger().warn(f"annot pub error: {e}")
        # republish /object_tracker/inference for follower to consume (QoS same as YOLO)
        if out_msg.yolov8_inference:
            try:
                self.out_pub.publish(out_msg)
            except Exception as e:
                self.get_logger().warn(f"republish inference error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ObjectTrackerCSRT()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
