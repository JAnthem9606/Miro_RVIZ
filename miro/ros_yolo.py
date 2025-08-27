#!/usr/bin/env python
import argparse
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from std_msgs.msg import String
import torch
import time
import cv2
import numpy as np
from models.experimental import attempt_load
from utils.general import non_max_suppression, scale_coords, xyxy2xywh, set_logging
from utils.torch_utils import select_device, time_synchronized
from utils.plots import plot_one_box
from utils.general import check_img_size
class ObjectDetectorROS:
    def __init__(self, weights, img_size, conf_thres, iou_thres, device, classes=None, agnostic_nms=False, augment=False):
        self.weights = weights
        self.img_size = img_size
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres
        self.device = device
        self.classes = classes
        self.agnostic_nms = agnostic_nms
        self.augment = augment

        # Initialize detector
        set_logging()
        self.device = select_device(device)
        self.model = attempt_load(weights, map_location=self.device)
        self.stride = int(self.model.stride.max())
        self.img_size = int(check_img_size(img_size, s=self.stride))
        if self.device.type != 'cpu':
            self.model.half()  # to FP16

        self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names
        self.colors = [[np.random.randint(0, 255) for _ in range(3)] for _ in range(len(self.names))]

        # ROS
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/miro/sensors/caml/compressed", CompressedImage, self.image_callback)
        self.image_pub = rospy.Publisher("/object_detection/compressed", CompressedImage, queue_size=100)
        self.class_pub = rospy.Publisher("/class",String,queue_size=10)

    def image_callback(self, compressed_image):
        # Convert compressed image message to OpenCV image
        np_arr = np.frombuffer(compressed_image.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Perform object detection
        img = cv2.resize(cv_image, (self.img_size, self.img_size))
        img = img[:, :, ::-1].transpose(2, 0, 1).copy()  # BGR to RGB, HWC to CHW, and copy the array
        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.device.type != 'cpu' else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        t1 = time_synchronized()
        with torch.no_grad():
            pred = self.model(img, augment=self.augment)[0]
        t2 = time_synchronized()

        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, classes=self.classes, agnostic=self.agnostic_nms)

        for i, det in enumerate(pred):
            if len(det):
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], cv_image.shape).round()
                for *xyxy, conf, cls in reversed(det):
                    label = f'{self.names[int(cls)]} {conf:.2f}'
                    c_label = f'{self.names[int(cls)]}'
                    plot_one_box(xyxy, cv_image, label=label, color=self.colors[int(cls)], line_thickness=1)
                    #self.class_pub.publish(c_label)  
        # Publish the annotated image
        #cv2.imshow("Frames",cv_image)
        #cv2.waitKey(1)
        compressed_img_msg = self.bridge.cv2_to_compressed_imgmsg(cv_image)
        self.image_pub.publish(compressed_img_msg)
        
        self.class_pub.publish(c_label)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', nargs='+', type=str, default='yolov7-tiny.pt', help='model.pt path(s)')
    parser.add_argument('--img-size', type=int, default=128, help='inference size (pixels)')
    parser.add_argument('--conf-thres', type=float, default=0.25, help='object confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.45, help='IOU threshold for NMS')
    parser.add_argument('--device', default='cpu', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --class 0, or --class 0 2 3')
    parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    parser.add_argument('--augment', action='store_true', help='augmented inference')
    args = parser.parse_args()

    rospy.init_node('object_detector_ros', anonymous=True)
    detector = ObjectDetectorROS(weights=args.weights, img_size=args.img_size, conf_thres=args.conf_thres,
                                 iou_thres=args.iou_thres, device=args.device, classes=args.classes,
                                 agnostic_nms=args.agnostic_nms, augment=args.augment)
    #rospy.rate(100)
    rospy.spin()

if __name__ == '__main__':
    main()
