import rclpy
from rclpy.node import Node
from kellerLD import KellerLD

from std_msgs.msg import Float32


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.depth_publisher = self.create_publisher(Float32, 'depth', 10)
        self.pressure_publisher = self.create_publisher(Float32, 'temperature', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.sensor = KellerLD()
        if(self.sensor.init()):
            self.get_logger().info("Depth/Temperature sensor initialized successfully.")
        else:
            self.get_logger().error("Depth/Temperature sensor initialization failed.")

    def timer_callback(self):
        self.sensor.read()
        temperature = Float32()
        pressure = Float32()
        pressure.data = self.sensor.pressure()
        temperature.data = self.sensor.temperature()
        self.depth_publisher.publish(temperature)
        self.pressure_publisher.publish(pressure)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()