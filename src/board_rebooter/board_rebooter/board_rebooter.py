import rclpy
from rclpy.node import Node
from gpiozero import LED

class GPIOnode(Node):
    def __init__(self):
        super().__init__('gpio_control_node')
        
        # Настройка GPIO
        self.out = LED(25)
        self.out.off()
        self.timer = self.create_timer(0.2, self.timer_callback)
    
    def timer_callback(self):
        self.out.on()
        self.get_logger().info("Pin set to HIGH")
        self.destroy_timer(self.timer)
        #GPIO.cleanup()
        self.get_logger().info("Shutting down")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = GPIOnode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
