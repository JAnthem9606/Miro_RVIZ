import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
import torch.nn.functional as F
from torchvision.transforms import Compose
from depth_anything.dpt import DepthAnything
from depth_anything.util.transform import Resize, NormalizeImage, PrepareForNet
depth_image = rospy.Publisher("/depth_anything/video/compressed",CompressedImage)

def image_callback(msg):
    bridge = CvBridge()
    try:
        # Convert the compressed image to a cv2 format
        raw_frame = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')
        frame_height, frame_width, _ = raw_frame.shape

        frame = cv2.cvtColor(raw_frame, cv2.COLOR_BGR2RGB) / 255.0
        frame = transform({'image': frame})['image']
        frame = torch.from_numpy(frame).unsqueeze(0).to(DEVICE)

        with torch.no_grad():
            depth = depth_anything(frame)

        # Interpolating the depth to match the frame size
        depth = F.interpolate(depth[None], (frame_height, frame_width), mode='bilinear', align_corners=False)[0, 0]
        depth = (depth - depth.min()) / (depth.max() - depth.min()) * 255.0
        depth = depth.cpu().numpy().astype(np.uint8)

        # Convert depth to color map for better visualization
        depth_color = cv2.applyColorMap(depth, cv2.COLORMAP_INFERNO)

        # Create split region and combine the raw frame and depth frame
        split_region = np.ones((frame_height, margin_width, 3), dtype=np.uint8) * 255
        combined_frame = cv2.hconcat([raw_frame, split_region, depth_color])

        # Display the result
        cv2.imshow('Depth', depth_color)
        depth_color_ros = bridge.cv2_to_compressed_imgmsg(depth_color)
        depth_image.publish(depth_color_ros)
        cv2.waitKey(1)

    except Exception as e:
        rospy.logerr("Error processing image: {}".format(e))


if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('depth_estimation_from_camera', anonymous=True)

    # Set up the device and model
    DEVICE = 'cuda' if torch.cuda.is_available() else 'cpu'
    depth_anything = DepthAnything.from_pretrained('LiheYoung/depth_anything_vitl14').to(DEVICE).eval()

    total_params = sum(param.numel() for param in depth_anything.parameters())
    print('Total parameters: {:.2f}M'.format(total_params / 1e6))

    margin_width = 50

    # Transformations for the input frame
    transform = Compose([
        Resize(
            width=128,
            height=128,
            resize_target=False,
            keep_aspect_ratio=True,
            ensure_multiple_of=14,
            resize_method='lower_bound',
            image_interpolation_method=cv2.INTER_CUBIC,
        ),
        NormalizeImage(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
        PrepareForNet(),
    ])

    # Set up ROS subscriber to the video/compressed topic
    #rospy.Subscriber('/miro/sensors/caml/compressed', CompressedImage, image_callback)
    rospy.Subscriber('/miro/sensors/caml/compressed', CompressedImage, image_callback)
    # Keep the node running to continuously receive images
    rospy.spin()
