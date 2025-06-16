import os
import argparse
import bisect
import numpy as np
from scipy.signal import butter, filtfilt
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory  
import matplotlib.pyplot as plt

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--directory",
                        type=str,
                        default="/home/rpi5/ros2_jazzy/src/drone/camera_pkgs/bags/data_1749031248.4807634_0_filtered", 
                        help="Directory to the ROS2 bag file."
                        )
    parser.add_argument("-f", "--filename",
                        type=str,
                        default="data_1749031248.4807634_0_filtered_0.mcap", 
                        help="Name of the ROS2 bag file."
                        )
    
    return parser.parse_args()

class BagEvaluate:
    def __init__(self):
        pass

    def butter_lowpass_filter(self, data, cutoff, fs, order=4):
        nyq = 0.5 * fs
        normal_cutoff = cutoff / nyq
        b, a = butter(order, normal_cutoff, btype='low', analog=False)
        return filtfilt(b, a, data)
    
    
    def smoothen(self, timestamps, data):
        # Estimate sampling frequency

        timestamps = timestamps * 1e-9  # Convert nanoseconds to seconds

        fs = 5*(1.0 / np.mean(np.diff(timestamps)))
        # print(f"Sampling frequency: {fs} Hz")
        cutoff = 1.0  # Hz

        smoothened_data = self.butter_lowpass_filter(data, cutoff, fs)
        return smoothened_data
    

    def find_closest(self, target_time, sorted_data, sorted_times):
        idx = bisect.bisect_left(sorted_times, target_time)
        if idx == 0:
            return sorted_data[0][1]
        
        if idx == len(sorted_times):
            return sorted_data[-1][1]
        
        before = sorted_data[idx-1]
        after = sorted_data[idx]

        return before[1] if abs(before[0] - target_time) < abs(after[0] - target_time) else after[1]
    

    def compute_heading(self, imu_accelerations, magnetic_fields):
        ax, ay, az = imu_accelerations[:, 0], imu_accelerations[:, 1], imu_accelerations[:, 2]
        mx_raw, my_raw, mz_raw = magnetic_fields[:, 0], magnetic_fields[:, 1], magnetic_fields[:, 2]

        mx = my_raw      # X --> forward
        my = -mx_raw     # Y --> right
        mz = mz_raw      # Z --> up (unchanged)

        g2 = ax*ax + ay*ay + az*az      # |a|^2
        dot = ax*mx + ay*my + az*mz     # a.m

        hx = mx - (ax * dot) / g2
        hy = my - (ay * dot) / g2

        yaw_rad = np.arctan2(hy, hx)
        yaw_deg = np.rad2deg(yaw_rad)

        return yaw_deg


    def extract_timestamp_matched_values(self, bag_filepath, gps_topic="/gpsfix", imu_topic="/imu/filtered", lidar_topic="/lidar", barometer_topic="/barometer", magnetic_field_topic="/magnetic_field"):
        
        gps_times, gps_alts, gps_speeds, gps_headings = [], [], [], []
        imu_times, imu_accelerations = [], []
        lidar_times, lidar_alts = [], []
        barometer_times, barometer_alts = [], []
        # heading_times, headings = [], []
        magnetic_filed_times, magnetic_fields = [], []

        with open(bag_filepath, "rb") as f:
        
            reader = make_reader(f, decoder_factories=[DecoderFactory()])
            print(f"Reading bag file: {bag_filepath}")

            valid_msg_topics = [imu_topic, gps_topic, lidar_topic, barometer_topic, magnetic_field_topic]
            
            for schema, channel, message, ros_msg in reader.iter_decoded_messages():

                if channel.topic not in valid_msg_topics:
                    continue

                if channel.topic == gps_topic:
                    if np.isnan(ros_msg.dip):
                        continue

                    gps_times.append(message.log_time)  # Convert nanoseconds to seconds
                    gps_alts.append(ros_msg.altitude)
                    gps_speeds.append(ros_msg.speed)
                    gps_headings.append(ros_msg.dip)

                elif channel.topic == imu_topic:
                    imu_times.append(message.log_time)
                    imu_accelerations.append((ros_msg.linear_acceleration.x, ros_msg.linear_acceleration.y, ros_msg.linear_acceleration.z))

                elif channel.topic == lidar_topic:
                    lidar_times.append(message.log_time)
                    lidar_alts.append(ros_msg.range)

                elif channel.topic == barometer_topic:
                    barometer_times.append(message.log_time)
                    barometer_alts.append(ros_msg.altitude)

                elif channel.topic == magnetic_field_topic:    
                    magnetic_filed_times.append(message.log_time)
                    magnetic_fields.append((ros_msg.magnetic_field.x, ros_msg.magnetic_field.y, ros_msg.magnetic_field.z))

            
        imu_data = sorted(zip(imu_times, imu_accelerations))
        imu_times_sorted = [t for t, _ in imu_data]
        
        lidar_data = sorted(zip(lidar_times, lidar_alts))
        lidar_times_sorted = [t for t, _ in lidar_data]

        barometer_data = sorted(zip(barometer_times, barometer_alts))
        barometer_times_sorted = [t for t, _ in barometer_data]

        magnetic_filed_data = sorted(zip(magnetic_filed_times, magnetic_fields))
        magnetic_filed_times_sorted = [t for t, _ in magnetic_filed_data]

        imu_accelerations_matched = [self.find_closest(ts, imu_data, imu_times_sorted) for ts in gps_times]
        lidar_altitudes_matched = [self.find_closest(ts, lidar_data, lidar_times_sorted) for ts in gps_times]
        barometer_altitudes_matched = [self.find_closest(ts, barometer_data, barometer_times_sorted) for ts in gps_times]
        magnetic_fields_matched = [self.find_closest(ts, magnetic_filed_data, magnetic_filed_times_sorted) for ts in gps_times]

        return np.array(gps_times), np.array(gps_alts), np.array(gps_headings), np.array(gps_speeds), np.array(imu_times), np.array(imu_accelerations_matched), np.array(lidar_altitudes_matched), np.array(barometer_altitudes_matched), np.array(magnetic_fields_matched)


    def evaluate(self, gps_times, gps_alts, gps_headings, gps_speeds, imu_times, imu_accelerations, lidar_alts, barometer_alts, magnetic_fields):
        initial_gps_alt = gps_alts[0]

        gps_alts = gps_alts - initial_gps_alt

        lidar_alts_imu_incorporated = lidar_alts * (imu_accelerations[:, 2]/(np.sqrt(imu_accelerations[:, 0]**2 + imu_accelerations[:, 1]**2 + imu_accelerations[:, 2]**2)))
        lidar_alts_imu_incorporated_smoothened = self.smoothen(gps_times, lidar_alts_imu_incorporated)

        lidar_to_gps_altitude_error = np.abs(lidar_alts - gps_alts)
        avg_lidar_gps_error = np.mean(lidar_to_gps_altitude_error)

        lidar_plus_imu_to_gps_altitude_error = np.abs(lidar_alts_imu_incorporated - gps_alts)
        avg_lidar_plus_imu_gps_error = np.mean(lidar_plus_imu_to_gps_altitude_error)

        lidar_plus_imu_plus_smoothening_to_gps_altitude_error = np.abs(lidar_alts_imu_incorporated_smoothened - gps_alts)
        lidar_plus_imu_plus_smoothening_error = np.mean(lidar_plus_imu_plus_smoothening_to_gps_altitude_error)
        
        barometer_to_gps_altitude_error = np.abs(barometer_alts - gps_alts)
        avg_barometer_gps_error = np.mean(barometer_to_gps_altitude_error)

        # print(imu_accelerations.shape)

        print("#####################################################################")
        
        print("Average error in altitudes:")
        print(f"Lidar to GPS: {avg_lidar_gps_error}")
        print(f"Lidar (with IMU) to GPS: {avg_lidar_plus_imu_gps_error}")
        print(f"Lidar (with IMU + smoothening) to GPS: {lidar_plus_imu_plus_smoothening_error}")
        print(f"Barometer to GPS: {avg_barometer_gps_error}")

        print("#####################################################################")

        raw_headings = -1 * np.arctan2(magnetic_fields[:, 0], magnetic_fields[:, 1]) * (180 / np.pi)
        raw_headings = (raw_headings + 360) % 360
        raw_diff = np.abs(raw_headings - gps_headings)
        raw_magnetometer_to_gps_heading_error = np.minimum(raw_diff, 360-raw_diff)

        raw_headings_smoothed = self.smoothen(gps_times, raw_headings)
        raw_headings_smoothed = (raw_headings_smoothed + 360) % 360
        smoothed_diff = np.abs(raw_headings_smoothed - gps_headings)
        smoothed_diff_magnetometer_to_gps_heading_error = np.minimum(smoothed_diff, 360-smoothed_diff)

        
        # avg_raw_magnetometer_to_gps_heading_error = np.mean(raw_magnetometer_to_gps_heading_error)
        
        headings_with_imu = self.compute_heading(imu_accelerations, magnetic_fields)
        headings_with_imu_smoothened = self.smoothen(gps_times, headings_with_imu)
        headings_with_imu_smoothened = (headings_with_imu_smoothened + 360) % 360
        diff = np.abs(headings_with_imu_smoothened - gps_headings)
        magnetometer_with_imu_to_gps_heading_error = np.minimum(diff, 360-diff)
        # avg_magnetometer_with_imu_to_gps_heading_error = np.mean(magnetometer_with_imu_to_gps_heading_error)

        print("Average error in headings:")

        print(f"Average raw magnetometer to GPS heading error: {np.mean(raw_magnetometer_to_gps_heading_error)} degrees")

        raw_heading_difference_smoothened = self.smoothen(gps_times, smoothed_diff_magnetometer_to_gps_heading_error)
        print(f"Average raw heading difference after smoothening: {np.mean(raw_heading_difference_smoothened)} degrees")

        heading_difference_with_imu_smoothened = self.smoothen(gps_times, magnetometer_with_imu_to_gps_heading_error)
        print(f"Average heading difference (with IMU) after smoothening: {np.mean(heading_difference_with_imu_smoothened)} degrees")

        imu_gps_ts_diff = np.abs(gps_times[0] - imu_times[0])
        gps_ts_since_start = gps_times - gps_times[0]
        gps_ts_since_start = gps_ts_since_start + imu_gps_ts_diff

        ts = gps_ts_since_start * 1e-9  # Convert nanoseconds to seconds

        print("#####################################################################")

        plt.figure(figsize=(18, 12))

        plt.title("Headings", fontweight="bold", fontsize=22)
        plt.xlabel("Time since recording in seconds", fontsize=18)
        plt.ylabel("Heading in degrees", fontsize=18)        

        plt.plot(ts, gps_headings, color="black", label="GPS heading", linewidth=2)
        plt.plot(ts, raw_headings, color="red", label="Magnetomeer heading (raw)", linewidth=1)
        plt.plot(ts, raw_headings_smoothed, color="blue", label="Magnetomeer heading + smoothing", linewidth=4)
        plt.plot(ts, headings_with_imu_smoothened, color="purple", label="Magnetomeer heading + IMU tilt correction + smoothing", linewidth=4)

        plt.legend()

        plt.grid()
        plt.show()

        # ###########################################################################################################################################

        plt.figure(figsize=(18, 12))

        plt.title("Headings Difference", fontweight="bold", fontsize=22)
        plt.xlabel("Time since recording in seconds", fontsize=18)        
        plt.ylabel("Heading difference in degrees", fontsize=18)

        plt.plot(ts, raw_magnetometer_to_gps_heading_error, label="Raw against GPS", color="red")
        plt.plot(ts, raw_heading_difference_smoothened, label="Raw (smoothing) against GPS", color="blue")
        plt.plot(ts, magnetometer_with_imu_to_gps_heading_error, label="Raw (IMU tilt correction + smoothing) against GPS", color="purple")

        plt.legend()

        plt.grid()
        plt.show()

        # ###########################################################################################################################################

        fig, ax1 = plt.subplots()
        ax1.set_xlabel('Time since recording in seconds', fontsize=18)
        ax1.set_ylabel('Magnetometer heading error (IMU + smoothing) in degrees', fontsize=18, color='blue')
        ax1.plot(ts, magnetometer_with_imu_to_gps_heading_error, label="Magnetometer heading error (IMU + smoothing)", color="blue")
        ax1.tick_params(axis='y', labelcolor='blue')

        ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis
        ax2.set_ylabel('GPS speed in m/s', fontsize=18, color='red')  # we already handled the x-label with ax1
        ax2.plot(ts, gps_speeds, label="GPS speed in m/s", color="red")
        ax2.tick_params(axis='y', labelcolor='red')

        plt.title("Heading error vs GPS speed", fontweight="bold", fontsize=22)

        plt.grid()
        plt.show()

        # ###########################################################################################################################################

        fig, ax1 = plt.subplots()
        ax1.set_xlabel('Time since recording in seconds', fontsize=18)
        ax1.set_ylabel('Magnetometer heading error (IMU + smoothing) in degrees', fontsize=18, color='blue')
        ax1.plot(ts, magnetometer_with_imu_to_gps_heading_error, label="Magnetometer heading error (IMU + smoothing)", color="blue")
        ax1.tick_params(axis='y', labelcolor='blue')

        ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis
        ax2.set_ylabel('GPS altitude in meters', fontsize=18, color='red')  # we already handled the x-label with ax1
        ax2.plot(ts, gps_alts, label="GPS altitude in meters", color="red")
        ax2.tick_params(axis='y', labelcolor='red')

        plt.title("Heading error vs GPS altitude", fontweight="bold", fontsize=22)

        plt.grid()
        plt.show()

        # ###########################################################################################################################################

        plt.figure(figsize=(18, 12))

        plt.title("Altitude values for different sensors", fontweight="bold", fontsize=22)
        plt.xlabel("Time since recording in seconds", fontsize=18)
        plt.ylabel("Altitude in meters", fontsize=18)     

        plt.plot(ts, barometer_alts, label="Barometer", color="blue")
        plt.plot(ts, lidar_alts_imu_incorporated_smoothened, label="Lidar + IMU + smoothing", color="red")
        plt.plot(ts, gps_alts, label="GPS", color="black")

        plt.legend()

        plt.grid()
        plt.show()

        # ###########################################################################################################################################

        plt.figure(figsize=(18, 12))

        plt.title("Altitude values for different sensors", fontweight="bold", fontsize=22)
        plt.xlabel("Time since recording in seconds", fontsize=18)
        plt.ylabel("Altitude in meters", fontsize=18)     

        plt.plot(ts, lidar_alts, color="blue", label="Lidar", linewidth=2)
        plt.plot(ts, lidar_alts_imu_incorporated, color="purple", label="Lidar + IMU", linewidth=1)
        plt.plot(ts, lidar_alts_imu_incorporated_smoothened, color="red", label="Lidar + IMU + smoothing", linewidth=4)
        plt.plot(ts, gps_alts, color="black", label="GPS", linewidth=2)

        plt.legend() 

        plt.grid()
        plt.show()

        # ###########################################################################################################################################

        
if __name__ == "__main__":
    
    args = parse_args()
    bag_filepath = os.path.join(args.directory, args.filename)
    
    try:
        evaluator = BagEvaluate()

        gps_times, gps_alts, gps_headings, gps_speeds, imu_times, imu_accelerations, lidar_alts, barometer_alts, magnetic_fields = evaluator.extract_timestamp_matched_values(bag_filepath)        
        evaluator.evaluate(gps_times, gps_alts, gps_headings, gps_speeds, imu_times, imu_accelerations, lidar_alts, barometer_alts, magnetic_fields)
        print("Evaluation completed.")

    except Exception as e:
        print(f"An error occurred: {e}")
    
    except KeyboardInterrupt:
        print("Evaluation interrupted by user. Keyboard interrrupt !!")
