import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO

class GPIOnode(Node):
    def __init__(self):
        super().__init__('gpio_control_node')
        
        # Настройка GPIO
        self.pin = 25  # GPIO6
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.OUT)
        
        # Установка LOW и запуск таймера
        GPIO.output(self.pin, GPIO.LOW)
        self.get_logger().info("Pin set to LOW")
        self.create_timer(0.2, self.timer_callback)
    
    def timer_callback(self):
        GPIO.output(self.pin, GPIO.HIGH)
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