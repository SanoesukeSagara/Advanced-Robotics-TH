#python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MovementPublisher(Node):
    def __init__(self):
        super().__init__('movement_publisher')
        self.publisher = self.create_publisher(String, 'py_movement_commands', 10) 
        timer_period = 2.0 #seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Movement Python Publisher Node Initialized .')

    def timer_callback(self):
        msg = String()
        msg.data = 'Move Forward'
        self.publisher.publish(msg)
        self.get_logger().info('Publishing python command: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    publisher_node = MovementPublisher()
    #Destroy the node explicitly
    #rclpy.shutdown() will destroy the node automatically
    try:
    	rclpy.spin(publisher_node)
    	
    except KeyboardInterrupt:
    	pass
    	
    finally:
    	publisher_node.destroy_node()
    	
    	if rclpy.ok():
    		rclpy.shutdown()        

if __name__ == '__main__':
    main()
