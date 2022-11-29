#!/usr/bin/env python3
# Credit to Rethink Robotics, Inc. for the msg listener_callback setup from their ros_image_saver.py

import cv2

import rclpy
from cv_bridge import CvBridge, CvBridgeError 
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo


class CameraSubscriber(Node):

	def __init__(self):
		super().__init__('arducam_camera_subscriber')

		# Create left image subscriber
		self.get_logger().info('Listener Created')
		self.i = 0
		self.bridge = CvBridge()
		self.left_sub = self.create_subscription(Image, 'left_image', self.listener_callback, 10)
		self.left_sub


	def listener_callback(self, msg):

        # Convert msg to img in cv2
		try:
			cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
			'''
			cv2.imshow('frame', cv2_img)
			keyCode = cv2.waitKey(30) & 0xFF
			
			# Stop the program on the ESC key
			if keyCode == 27:
				raise SystemExit
			'''
			# Write img to file in imgs/ folder
			cv2.imwrite('data/img_' + str(self.i) + '.jpeg', cv2_img)

			self.get_logger().info('Listening: "%s"' % str(self.i))
			self.i += 1
		except CvBridgeError as e:
			print(e)



def main(args=None):
	rclpy.init(args=args)

	camera_subscriber = CameraSubscriber()

	try:
		rclpy.spin(camera_subscriber)
	except SystemExit:
		rclpy.logging.get_logger("Quitting").info('Done')

	camera_subscriber.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()

