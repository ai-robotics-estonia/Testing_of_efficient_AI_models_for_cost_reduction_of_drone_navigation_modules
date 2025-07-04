import rclpy
import rclpy.clock
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Float32
from sensor_msgs.msg import Image, Imu, Range, MagneticField
from custom_msgs.msg import Barometer, Azimuth
import ros2_numpy as rnp

from picamera2 import Picamera2
import smbus2
from concurrent.futures import ThreadPoolExecutor

import numpy as np
import requests

# Magnetometer
class LIS3MDL:
    def __init__(self):
        self.bus = smbus2.SMBus(1)

        # I2C address of LIS3MDL
        self.LIS3MDL_ADDRESS = 0x1c

        # Register addresses
        self.CTRL_REG1 = 0x20
        self.CTRL_REG2 = 0x21
        self.CTRL_REG3 = 0x22
        self.OUT_X_L = 0x28
        self.OUT_X_H = 0x29
        self.OUT_Y_L = 0x2A
        self.OUT_Y_H = 0x2B
        self.OUT_Z_L = 0x2C
        self.OUT_Z_H = 0x2D
        
               
        # convert from LSB to Gauss
        self.sensitivity = {
            4: 1/6842,
            8: 1/3421,
            12: 1/2281,
            16: 1/1711
            }
        
        magnetic_range = 4		# set at +-4 gauss
        self.LSB_TO_GAUSS = self.sensitivity[magnetic_range]        

        # Initializing the magnetometer
        self.configure_lis3mdl()
        print("LIS3MDL magnetometer intialized")

    def configure_lis3mdl(self):
        self.bus.write_byte_data(self.LIS3MDL_ADDRESS, self.CTRL_REG1, 0x70)  # Enable temperature sensor, set ODR to 80 Hz
        self.bus.write_byte_data(self.LIS3MDL_ADDRESS, self.CTRL_REG2, 0x00)  # Set full-scale range to +/- 4 gauss
        self.bus.write_byte_data(self.LIS3MDL_ADDRESS, self.CTRL_REG3, 0x00)
    
    def read_magnetometer_data(self):
        x_l = self.bus.read_byte_data(self.LIS3MDL_ADDRESS, self.OUT_X_L)
        x_h = self.bus.read_byte_data(self.LIS3MDL_ADDRESS, self.OUT_X_H)
        x = np.float64((x_h << 8) | x_l)

        y_l = self.bus.read_byte_data(self.LIS3MDL_ADDRESS, self.OUT_Y_L)
        y_h = self.bus.read_byte_data(self.LIS3MDL_ADDRESS, self.OUT_Y_H)
        y = np.float64((y_h << 8) | y_l)

        z_l = self.bus.read_byte_data(self.LIS3MDL_ADDRESS, self.OUT_Z_L)
        z_h = self.bus.read_byte_data(self.LIS3MDL_ADDRESS, self.OUT_Z_H)
        z = np.float64((z_h << 8) | z_l)

        # Convert to signed values
        if x > 32767:
            x -= 65536
        if y > 32767:
            y -= 65536
        if z > 32767:
            z -= 65536  
        
        
        mag_x_gauss = x * self.LSB_TO_GAUSS
        mag_y_gauss = y * self.LSB_TO_GAUSS
        mag_z_gauss = z * self.LSB_TO_GAUSS
        
        # 1 Gauss = 100 uT --> uT = G * 100         
        mag_x_uT = mag_x_gauss * 100
        mag_y_uT = mag_y_gauss * 100
        mag_z_uT = mag_z_gauss * 100        

        return mag_x_uT, mag_y_uT, mag_z_uT

    def calibrated_magnetic_field(self, raw_mag_x, raw_mag_y, raw_mag_z):
        
        b = np.array([-33.321212, 7.169298, -8.956170])    # hard-iron bias
        A = np.array([                                      # soft-iron bias
                     [0.927597, 0.086125, -0.010263],
                     [0.086125, 0.937336, -0.003746],
                     [-0.010263, -0.003746, 0.984599]
                    ])
            
        raw_mag_data = np.array([raw_mag_x, raw_mag_y, raw_mag_z])
        
        calibrated_mag_data = A @ (raw_mag_data - b).T
        # print()
        mag_x, mag_y, mag_z = calibrated_mag_data

        return mag_x, mag_y, mag_z


# Temperature + Barometric Sensor
class DPS310:
    def __init__(self):
        self.bus = smbus2.SMBus(1)
        self.addr = 0x77        
        
        self.T0 = None
        self.P0 = None
        self.L = 0.0065         # Temperature Lapse rate in K/m
        self.R = 8.31432        # Universal Gas constant in J/(mol-K)
        self.M = 0.0289644      # Molar mass of Earth's air in kg/mol
        self.g = 9.80665        # Acceleration due to gravity in m/s^2
        
        # Correct scale factors from Table 9 for 8x oversampling
        self.kT = 7864320  # Scale factor for temperature (8x oversampling)
        self.kP = 7864320  # Scale factor for pressure (8x oversampling)

        # Read calibration coefficients and configure sensor
        self._read_calibration()
        self._config()
        self._start_background_mode()
        

    def _to_16bit_int(self, msb, lsb):
        """Read the MSB and LSB values and convert to 16-bit integer."""
        val = (msb << 8) | lsb
        return val - 65536 if val > 32767 else val

    def _read_calibration(self):
        """Read the calibration coefficients from the sensor."""
        coef = self.bus.read_i2c_block_data(self.addr, 0x10, 18)

        # Extract and calculate calibration coefficients
        c0 = (coef[0] << 4) | (coef[1] >> 4)
        if c0 > 2047: c0 -= 4096
        self.c0 = c0

        c1 = ((coef[1] & 0x0F) << 8) | coef[2]
        if c1 > 2047: c1 -= 4096
        self.c1 = c1

        c00 = (coef[3] << 12) | (coef[4] << 4) | (coef[5] >> 4)
        if c00 > 524287: c00 -= 1048576
        self.c00 = c00

        c10 = ((coef[5] & 0x0F) << 16) | (coef[6] << 8) | coef[7]
        if c10 > 524287: c10 -= 1048576
        self.c10 = c10

        self.c01 = self._to_16bit_int(coef[8], coef[9])
        self.c11 = self._to_16bit_int(coef[10], coef[11])
        self.c20 = self._to_16bit_int(coef[12], coef[13])
        self.c21 = self._to_16bit_int(coef[14], coef[15])
        self.c30 = self._to_16bit_int(coef[16], coef[17])

    def _config(self):
        """Configure the sensor for 8x oversampling and 4 Hz measurement rate."""
        # Set pressure configuration (4 Hz, 8x oversampling)
        prs_cfg = (0b010 << 4) | 0b0011
        self.bus.write_byte_data(self.addr, 0x06, prs_cfg)

        # Set temperature configuration, matching TMP_EXT to TMP_COEF_SRCE
        coef_src = (self.bus.read_byte_data(self.addr, 0x28) >> 7) & 0x01
        tmp_cfg = (coef_src << 7) | (0b010 << 4) | 0b0011
        self.bus.write_byte_data(self.addr, 0x07, tmp_cfg)

    def _start_background_mode(self):
        """Start the sensor in background measurement mode."""
        self.bus.write_byte_data(self.addr, 0x08, 0x07)

    def _read_raw(self):
        """Read raw pressure and temperature data from the sensor."""
        raw = self.bus.read_i2c_block_data(self.addr, 0x00, 6)
        prs_raw = self._twos_complement((raw[0] << 16) | (raw[1] << 8) | raw[2], 24)
        tmp_raw = self._twos_complement((raw[3] << 16) | (raw[4] << 8) | raw[5], 24)
        return prs_raw, tmp_raw

    def _twos_complement(self, val, bits):
        """Convert a value to signed 2's complement."""
        if val & (1 << (bits - 1)):
            val -= (1 << bits)
        return val

    def read(self):
        """Read compensated temperature and pressure values."""
        Praw, Traw = self._read_raw()

        # Scale raw values to compensation scales
        Traw_sc = Traw / self.kT
        Praw_sc = Praw / self.kP

        # Compensate temperature
        Tcomp = self.c0 * 0.5 + self.c1 * Traw_sc

        # Compensate pressure
        Pcomp = (
            self.c00
            + Praw_sc * (self.c10 + Praw_sc * (self.c20 + Praw_sc * self.c30))
            + Traw_sc * self.c01
            + Traw_sc * Praw_sc * (self.c11 + Praw_sc * self.c21)
        )

        return Tcomp, Pcomp/100  # T in °C, P in hPa


class TriggeredCameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')        
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        
        self.cam_front_publisher_ = self.create_publisher(Image, 'cam/front/image', qos)
        self.cam_bottom_publisher_ = self.create_publisher(Image, 'cam/bottom/image', qos)
        self.lidar_publisher_ = self.create_publisher(Range, 'lidar', qos)
        self.barometer_publisher_ = self.create_publisher(Barometer, 'barometer', qos)
        self.magnetometer_publisher_ = self.create_publisher(MagneticField, 'magnetic_field', qos)
        self.heading_publisher_ = self.create_publisher(Azimuth, 'heading', qos)
        
        
        self.dps310 = DPS310()        
        self.lis3mdl = LIS3MDL()

        self.magnetic_declination = None

        ################################################## Initialize cameras ##################################################

        self.cam_front = Picamera2(camera_num=0)
        self.cam_bottom = Picamera2(camera_num=1)
        
        # Create video configuration and configure the cameras
        cam_front_config = self.cam_front.create_video_configuration(main={"size": (640, 480), "format": "YUV420"})
        cam_bottom_config = self.cam_bottom.create_video_configuration(main={"size": (640, 480), "format": "YUV420"})
        
        self.cam_front.configure(cam_front_config)
        self.cam_bottom.configure(cam_bottom_config)
        
        self.cam_front.start()
        self.cam_bottom.start()

        ########################################################################################################################

        self.counter = 0

        self.distance = None
        
        self.thread_pool = ThreadPoolExecutor(max_workers=5)
        
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            1)
        
        self.distance_subscription = self.create_subscription(
            Float32,
            '/distance',
            self.distance_callback,
            1)
        
        self.magnetic_declination_subscription = self.create_subscription(
            Float32,
            '/magnetic_declination',
            self.magnetic_declination_callback,
            1)
    
    
    def distance_callback(self, msg):        
        self.distance = msg.data


    def magnetic_declination_callback(self, msg):
        
        if self.magnetic_declination is None:
            self.magnetic_declination = msg.data                # Assign it just once


    def imu_callback(self, msg):
        
        if self.distance is None :
            print("Altimeter distance not retrieved yet")
            print("------------------------------------")        
            return

        if self.magnetic_declination is None:
            print("Magnetic declination not retrieved yet")
            print("------------------------------------")        
            return
        
        self.counter += 1
        if self.counter >= 6:                
            
            # Submit tasks to executor                
            self.thread_pool.submit(self.magnetometer_capture_and_publish)
            self.thread_pool.submit(self.cam_capture_and_publish, "front", self.cam_front, self.cam_front_publisher_)
            self.thread_pool.submit(self.cam_capture_and_publish, "bottom", self.cam_bottom, self.cam_bottom_publisher_)                                
            self.thread_pool.submit(self.lidar_capture_and_publish)                
            self.thread_pool.submit(self.barometer_capture_and_publish, self.dps310.L, self.dps310.R, self.dps310.M, self.dps310.g)                               
            
            self.counter = 0
                

    def cam_capture_and_publish(self, cam_direction, cam, publisher_):
        
        timestamp = rclpy.clock.Clock().now().to_msg()

        image = cam.capture_array("main")
        image = image[:480, :640]

        img_msg = rnp.msgify(Image, image, encoding="mono8")
        
        img_msg.header.stamp = timestamp
        img_msg.header.frame_id = f"{cam_direction}_camera_optical_frame"
        
        publisher_.publish(img_msg)
        
        
    def lidar_capture_and_publish(self):
        timestamp = rclpy.clock.Clock().now().to_msg()
        
        distance = self.distance

        range_msg = Range()
        range_msg.header.stamp = timestamp
        range_msg.header.frame_id = "altimeter_link"

        range_msg.range = distance

        self.lidar_publisher_.publish(range_msg)
        
    
    def barometer_capture_and_publish(self, L, R, M, g):
        timestamp = rclpy.clock.Clock().now().to_msg()

        # Read Barometer data

        temperature, pressure = self.dps310.read()
        
        if self.dps310.T0 is None:
                self.dps310.T0 = temperature
        if self.dps310.P0 is None:
                self.dps310.P0 = pressure
        
        P0 = self.dps310.P0
        altitude = 44330 * (1-(pressure/P0) ** (1/5.255))

        barometer_msg = Barometer()        
        barometer_msg.header.stamp = timestamp
        barometer_msg.header.frame_id = "barometer_link"

        barometer_msg.pressure = pressure
        barometer_msg.temperature = temperature
        barometer_msg.altitude = altitude

        self.barometer_publisher_.publish(barometer_msg)
        

    def magnetometer_capture_and_publish(self):
        timestamp = rclpy.clock.Clock().now().to_msg()

        # Read magnetometer data
        raw_mag_x, raw_mag_y, raw_mag_z = self.lis3mdl.read_magnetometer_data()        
        mag_x, mag_y, mag_z = self.lis3mdl.calibrated_magnetic_field(raw_mag_x, raw_mag_y, raw_mag_z)
        
        ###################################################################################################################################################

        magnetometer_msg = MagneticField()
        magnetometer_msg.header.stamp = timestamp
        magnetometer_msg.header.frame_id = "magnetometer_link"

        magnetometer_msg.magnetic_field.x = mag_x
        magnetometer_msg.magnetic_field.y = mag_y
        magnetometer_msg.magnetic_field.z = mag_z

        self.magnetometer_publisher_.publish(magnetometer_msg)

        ###################################################################################################################################################

        heading = -1 * np.arctan2(mag_x, mag_y) * 180 / np.pi   # Converting radians to degrees

        heading_msg = Azimuth()
        heading_msg.header.stamp = timestamp
        heading_msg.header.frame_id = "magnetometer_link"

        heading_msg.azimuth = heading + self.magnetic_declination    # # Adding magnetic declination (~10.2 in Tartu) to get the true north heading
        heading_msg.unit = Azimuth.UNIT_DEG
        heading_msg.orientation = Azimuth.ORIENTATION_NED
        heading_msg.reference = Azimuth.REFERENCE_GEOGRAPHIC
        
        self.heading_publisher_.publish(heading_msg)    

        ###################################################################################################################################################
       

    def run(self):
        rclpy.spin(self)


def main(args=None):
    rclpy.init(args=args)
    camera_node = TriggeredCameraNode()
    
    try:
        camera_node.run()

    except KeyboardInterrupt:
        print("Keyboard interrupt detected... Shutting down trigger node!")

    except Exception as e:
        print(f"Error occured: {e}")
        
    finally:        
        rclpy.shutdown()
        
    
if __name__ == "__main__":
    main()
