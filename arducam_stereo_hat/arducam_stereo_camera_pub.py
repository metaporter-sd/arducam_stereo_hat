#!/usr/bin/env python3
# Credit to ArduCAM/Camarray_HAT repo for script set-up

import cv2
import numpy as np

import rclpy
from cv_bridge import CvBridge, CvBridgeError 
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
#from camera_info_manager import CameraInfoManager # TODO: ros1_bridge for camera info


class CameraPublisher(Node):

    def __init__(self, left_info_url, right_info_url):
        super().__init__('arducam_camera_publisher')
        '''
        # Set up CameraInfoManager's for left & right cameras
        self.left_info_mgr = CameraInfoManager(cname='left_camera', namespace='left')
        self.right_info_mgr = CameraInfoManager(cname='right_camera', namespace='right')
        
        self.left_info_mgr.setURL(left_info_url)
        self.right_info_mgr.setURL(right_info_url)
        
        self.left_info_mgr.loadCameraInfo()
        self.right_info_mgr.loadCameraInfo()

        '''

        # Create image publishers
        self.left_pub = self.create_publisher(Image, 'left_image', 10)
        self.right_pub = self.create_publisher(Image, 'right_image', 10)

        # Create cameraInfo publishers
        #self.left_info_pub = self.create_publisher(CameraInfo, 'left_camera_info', 10)
        #self.right_info_pub = self.create_publisher(CameraInfo, 'right_camera_info', 10)

        print("start\n")
        print(gstreamer_pipeline())
        print("\nstop\n")

        self.cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            print("Cannot open camera")
            exit()

        self.bridge = CvBridge()

        # Set up timer & rate
        timer_period = 1    # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        # Capture frame-by-frame
        ret, frame = self.cap.read()

        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            raise SystemExit

        # Our operations on the frame come here
        #gray = cv2.cvtColor(frame, cv.COLOR_BGR2GRAY)

        # Get the proper encoding for image publishing
        encoding = "bgr8" if len(frame.shape) == 3 and frame.shape[2] >= 3 else "mono8"

        width = frame.shape[1]
        height = frame.shape[0]

        # Get the left & right imgs from the frame
        left_img = frame[:, :width//2]
        right_img = frame[:, width//2:]

        # Display the resulting frame
        cv2.imshow('frame', frame)
        keyCode = cv2.waitKey(30) & 0xFF

        # Stop the program on the ESC key
        if keyCode == 27:
            raise SystemExit

        # Get the capture time of the images for msg header
        capture_time = self.get_clock().now().to_msg()

        # Set up left_img_msg with cv_bridge & header info
        left_img_msg = self.bridge.cv2_to_imgmsg(left_img, encoding)
        left_img_msg.header.frame_id = 0   #TODO: add frame_id from ros parameters
        left_img_msg.header.stamp = capture_time

        # Set up right_img_msg with cv_bridge & header info
        right_img_msg = self.bridge.cv2_to_imgmsg(right_img, encoding)
        right_img_msg.header.frame_id = 0   #TODO: add frame_id from ros parameters
        right_img_msg.header.stamp = capture_time

        '''
        # Get cameraInfo from the managers
        info_left = self.left_info_mgr.getCameraInfo()
        info_right = self.right_info_mgr.getCameraInfo()

        # Set the header info
        info_left.header.stamp = capture_time
        info_right.header.stamp = capture_time
        info_left.header.frame_id = 0   #TODO: add frame_id from ros parameters
        info_right.header.frame_id = 0  #TODO: add frame_id from ros parameters
        '''
        
        # Publish imgs & cameraInfo
        self.left_pub.publish(left_img_msg)
        self.right_pub.publish(right_img_msg)
        #self.left_info_pub.publish(info_left)
        #self.right_info_pub.publish(info_right)

        self.get_logger().info('Publishing: "%s"' % str(self.i))
        self.i += 1
        

def gstreamer_pipeline(capture_width=2592, capture_height=1944, display_width=1440,
    display_height=1080,
    framerate=10,
    flip_method=2,
):
	#credit: JetsonHacks
    return (
		"nvarguscamerasrc sensor-id=0 ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

def main(args=None):
    rclpy.init(args=args)

    camera_publisher = CameraPublisher("left_url", "right_url") #TODO: add CameraInfoManager URLs (using ros parameters [see comment block below])

    try:
        rclpy.spin(camera_publisher)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')

    camera_publisher.cap.release()
    cv2.destroyAllWindows()

    camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


'''
    #parameters and functions that might be useful from original script (converted to ROS2)

    try:
        device = rclpy.get_parameter("~device")
    except:
        device = 0

    # open camera
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)

    try:
        # set pixel format
        pixfmt = rclpy.get_parameter("~pixelformat")
        if not cap.set(cv2.CAP_PROP_FOURCC, pixelformat(pixfmt)):
            print("Failed to set pixel format.")
    except:
        pass
    
    try:
        width = rclpy.get_parameter("~width")
        # set width
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)

        height = rclpy.get_parameter("~height")
        # set height
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    except:
        pass

    try:
        frame_id = rclpy.get_parameter("~frame_id")
    except:
        frame_id = "cam0"

    try:
        left_info_url = rclpy.get_parameter("~left/camera_info_url")
        right_info_url = rclpy.get_parameter("~right/camera_info_url")
    except:
        left_info_url = None
        right_info_url = None

    run(cap)

    # release camera
    cap.release()
'''
