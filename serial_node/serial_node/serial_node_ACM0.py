#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Int32
import serial
import numpy as np

class serial_node_ACM0(Node):
	def __init__(self):
		super().__init__('serial_node')
		self.subscription_ = self.create_subscription(String, 'teamthree_serial_ACM0', 
					self.command_callback, 10)
		self.serial_port = serial.Serial('/dev/ttyACM0', 9600, timeout = 0,
					write_timeout = 1)
		self.subscription_ # prevent unused variable warning
		
	def command_callback(self, msg):
		try:
			self.serial_port.write(msg.data.encode('utf-8'))
			self.get_logger().info(f'Sent to Arduino: {msg.data!r}')
		except serial.SerialException as error:
			self.get_logger().error(f'Serial write failed. {error}')
		
	def destroy_node(self):
		self.serial_port.close()
		super().destroy_node()

def main(args=None):
	rclpy.init(args=args)
	node = None
	try:		
		serial_node = serial_node_ACM0()
		rclpy.spin(serial_node)
	except KeyboardInterrupt:
		pass
	finally:
		if node is not None:
			node.destroy_node()
		if rclpy.ok():
			rclpy.shutdown()
	serial_node.destroy_node()
	rclpy.shutdown()
	
if __name__ == '__main__' : main()

