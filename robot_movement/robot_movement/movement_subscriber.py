#python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MovementSubscriber(Node):
    def __init__(self):
        super().__init__('movement_subscriber')
        self.subscription_py = self.create_subscription(
            String,
            'py_movement_commands',
            self.py_listener_callback,
            10
        )
        self.subscription_cpp = self.create_subscription(
            String,
            'cpp_movement_commands',
            self.cpp_listener_callback,
            10
        )
        self.subscription_py  # prevent unused variable warning
        self.subscription_cpp
        self.get_logger().info('Movement Python Subscriber Node Initialized.')

    def py_listener_callback(self, msg):
        self.get_logger().info('Received python movement command: "%s"' % msg.data)
        #Simulate robot movement based on the received command
        if msg.data == 'Move Forward':
            self.move_forward()
        elif msg.data == 'Turn Left':
            self.turn_left()
        elif msg.data == 'Turn Right':
            self.turn_right()

    def cpp_listener_callback(self, msg):
        self.get_logger().info('Received c++ movement command: "%s"' % msg.data)
        #Simulate robot movement based on the received command
        if msg.data == 'Move Forward':
            self.move_forward()
        elif msg.data == 'Turn Left':
            self.turn_left()
        elif msg.data == 'Turn Right':
            self.turn_right()

    def move_forward(self):
        self.get_logger().info('Robot is moving forward.')

    def turn_left(self):
        self.get_logger().info('Robot is turning left.')

    def turn_right(self):
        self.get_logger().info('Robot is turning right.')

def main(args=None):
    rclpy.init(args=args)
    subscriber_node = MovementSubscriber()
    try:
    	rclpy.spin(subscriber_node)
    	
    except KeyboardInterrupt:
    	pass
    	
    finally:
    	subscriber_node.destroy_node()
    	
    	if rclpy.ok():
    		rclpy.shutdown() 
    
if __name__ == '__main__':
    main()
