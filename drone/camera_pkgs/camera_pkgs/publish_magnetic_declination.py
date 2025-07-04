import rclpy
from rclpy.node import Node
from gps_msgs.msg import GPSFix
from std_msgs.msg import Float32
import requests
from rclpy.qos import QoSProfile, QoSDurabilityPolicy


class MagneticDeclinationNode(Node):
    def __init__(self):
        super().__init__('magnetic_declination_node')

        # Create a transient local QoS profile
        qos_profile = QoSProfile(depth=1)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        # Latched-style publisher
        self.publisher = self.create_publisher(Float32, '/magnetic_declination', qos_profile)

        # GPS subscriber
        self.subscription = self.create_subscription(
            GPSFix,
            '/gpsfix',
            self.gps_callback,
            1
        )
        self.has_published = False
        

    def gps_callback(self, msg):
        if self.has_published:
            return

        lat = msg.latitude
        lon = msg.longitude

        url = "https://geomag.bgs.ac.uk/web_service/GMModels/wmm/2020v2"
        params = {
            "latitude": lat,
            "longitude": lon,
            "format": "json"
        }
        
        try:
            response = requests.get(url, params=params, timeout=5)
            response.raise_for_status()
            data = response.json()
            declination = data['geomagnetic-field-model-result']['field-value']['declination']['value']

            msg_out = Float32()
            msg_out.data = declination
            self.publisher.publish(msg_out)

            self.has_published = True

        except Exception as e:
            self.get_logger().error(f"Error retrieving magnetic declination: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MagneticDeclinationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
