#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from kellerLD import KellerLD

from std_msgs.msg import Float32


class Bar100Sensor(Node):

    def __init__(self):
        super().__init__('bar100_sensor')
        self.depth_publisher = self.create_publisher(Float32, 'depth', 10)
        self.temperature_publisher = self.create_publisher(Float32, 'temperature', 10)
        self.declare_parameter('timer_period', 0.05)
        self.declare_parameter('surface_pressure', 101325)
        self.declare_parameter('water_density', 1025)
        self.declare_parameter('gravity_acceleration', 9.80665)

        timer_period = self.get_parameter('timer_period').get_parameter_value().double_value  # seconds

        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.sensor = KellerLD()
        if(self.sensor.init()):
            self.get_logger().info("Depth/Temperature sensor initialized successfully.")
        else:
            self.get_logger().error("Depth/Temperature sensor initialization failed.")

    def timer_callback(self):
        surface_pressure = self.get_parameter('surface_pressure').get_parameter_value().double_value
        water_density = self.get_parameter('water_density').get_parameter_value().double_value
        gravity_acceleration = self.get_parameter('gravity_acceleration').get_parameter_value().double_value
        
        self.sensor.read()
        if self.sensor.pressure() is None or self.sensor.temperature() is None:
            self.get_logger().warn("Sensor returned None. Skipping publish.")
            return
        
        
        temperature = Float32()
        depth = Float32()
        depth.data = (self.sensor.pressure() - surface_pressure) / (water_density * gravity_acceleration)
        temperature.data = self.sensor.temperature()
        self.temperature_publisher.publish(temperature)
        self.depth_publisher.publish(depth)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = Bar100Sensor()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()