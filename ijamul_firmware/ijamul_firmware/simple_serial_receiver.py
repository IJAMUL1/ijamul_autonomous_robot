#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial



class SimpleSerialReceiver(Node):
    def __init__(self):
        super().__init__("simple_serial_receiver")
        self.get_logger().info("Initializing Simple Serial Reciever")


        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)

        self.port_ = self.get_parameter("port").value
        self.baudrate_ = self.get_parameter("baudrate").value

        self.get_logger().info(f"Port: {self.port_}, Baudrate: {self.baudrate_}")


        try:
            self.arduino_ = serial.Serial(port=self.port_, baudrate=self.baudrate_, timeout=0.1)
            self.get_logger().info("Serial port opened successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
        
        self.pub_ = self.create_publisher(String, "serial_receiver", 10)
        self.get_logger().info("Publisher set up")
        self.frequency_ = 0.01
        self.timer_ = self.create_timer(self.frequency_, self.timerCallback)

    def timerCallback(self):
        if rclpy.ok() and self.arduino_.is_open:
            data = self.arduino_.readline()
            try:
                data.decode("utf-8")
            except:
                return

            msg = String()
            msg.data = str(data)
            self.pub_.publish(msg)


def main():
    rclpy.init()

    simple_serial_receiver = SimpleSerialReceiver()
    rclpy.spin(simple_serial_receiver)
    
    simple_serial_receiver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()