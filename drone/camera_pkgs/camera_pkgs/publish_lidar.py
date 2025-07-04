import rclpy
import rclpy.logging
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

import serial
import time


class TF350:
    def __init__(self):
        self.port = '/dev/ttyUSB1'
        self.baud_rate = 115200
        self.packet_size = 9
        self.trigger_command = bytes([0x5A, 0x04, 0x04, 0x62])      # Command to trigger the Lidar/altimeter

        self.ser = serial.Serial(self.port, self.baud_rate, timeout=1)
        
        
    def parse_packet(self, packet):
        try:
            distance = int.from_bytes(packet[2:4], byteorder='little') / 100.0  # convert to meter
            return distance
            
        except Exception as e:
            return None
        
        
    def enable_command_trigger(self):
        trigger_enable_command = bytes([0x5A, 0x04, 0x07, 0x00, 0x66])
        self.ser.write(trigger_enable_command)
        
        
    def read_lidar_data(self):
        try:
            self.ser.write(self.trigger_command)
            data = self.ser.read(self.packet_size)
            distance = self.parse_packet(data)
            return distance
                
        except Exception as e:
            return None
            
            
class LidarPublisher(Node):
    
    def __init__(self):
        super().__init__('distance_publisher_node')

        self.TF350 = TF350()
        
        self.lidar_publisher_ = self.create_publisher(Float32, 'distance', 10)
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            1)
        

    def imu_callback(self, msg):
        
        # Lidar data
        distance = self.TF350.read_lidar_data()
            
        distance_msg = Float32()
        distance_msg.data = distance
        self.lidar_publisher_.publish(distance_msg)
        
        
    def run(self):
        rclpy.spin(self)
    

def main(args=None):

    rclpy.init(args=args)
    node = LidarPublisher()
    
    try:
        node.run()
    
    except KeyboardInterrupt:
        print("Keyboard interrupt detected... Shutting down distance publishing node!")
        
    except Exception as e:
        print(f"Error occured: {e}")
        
    finally:
        node.destroy_node()

if __name__ == "__main__":
    main()
    