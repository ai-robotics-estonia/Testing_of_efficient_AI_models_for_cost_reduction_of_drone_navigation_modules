import os
import argparse
import bisect
import numpy as np
from scipy.signal import butter, filtfilt
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory  
import matplotlib.pyplot as plt
from collections import deque

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--directory",
                        type=str,
                        default="/home/adl/lendurai_files/new_lendurai_bags/data_1749031248.4807634_0_filtered", 
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

        return (yaw_deg + 360) % 360


    def extract_timestamp_matched_values(self, bag_filepath, gps_topic="/gpsfix", imu_topic="/imu/filtered", lidar_topic="/lidar", barometer_topic="/barometer", magnetic_field_topic="/magnetic_field"):
        
        gps_times, gps_alts, gps_speeds, gps_headings = [], [], [], []
        imu_times, imu_accelerations = [], []
        lidar_times, lidar_alts = [], []
        barometer_times, barometer_alts = [], []
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

        imu_gps_ts_diff = np.abs(gps_times[0] - imu_times[0])
        gps_ts_since_start = gps_times - gps_times[0]
        gps_ts_since_start = gps_ts_since_start + imu_gps_ts_diff

        gps_ts = gps_ts_since_start * 1e-9  # Convert nanoseconds to seconds

        altitude_ema_aplhas = [0.04, 0.05, 0.06, 0.07, 0.08, 0.09, 0.10]
        ema_ts = []
        ema_averaged = []

        for alpha in altitude_ema_aplhas:
            
            ema_window = deque(maxlen=2) 
            alpha_gps_ts = []
            alpha_ema_averaged = []

            for ts, alt in zip(gps_ts, lidar_alts_imu_incorporated):    
                
                if len(ema_window) < 1:
                    ema_window.append(alt)
                    alpha_gps_ts.append(ts)
                    alpha_ema_averaged.append(alt)

                else :
                    ema = alpha * alt + (1 - alpha) * ema_window[-1]
                    ema_window.append(ema)
                    alpha_gps_ts.append(ts)
                    alpha_ema_averaged.append(ema)
            
            ema_ts.append(alpha_gps_ts)
            ema_averaged.append(alpha_ema_averaged)            

        ema_ts = np.array(ema_ts)
        ema_averaged = np.array(ema_averaged)

        print(ema_averaged.shape)
        print(gps_alts.shape)

        abs_error_lidar_imu = np.abs(lidar_alts_imu_incorporated - gps_alts)
        avg_error_lidar_imu = np.mean(abs_error_lidar_imu)

        abs_error_barometer = np.abs(barometer_alts - gps_alts)
        avg_error_barometer = np.mean(abs_error_barometer)

        abs_errors_ema = np.abs(ema_averaged - gps_alts)
        avg_errors_ema = np.mean(abs_errors_ema, axis=1)

        best_error_idx = np.argmin(avg_errors_ema)

        print(avg_error_lidar_imu)
        print(avg_errors_ema)

        ###############################################################################################################################################################
        
        raw_headings = -1 * np.arctan2(magnetic_fields[:, 0], magnetic_fields[:, 1]) * (180 / np.pi)
        raw_headings = (raw_headings + 360) % 360

        headings_with_imu = self.compute_heading(imu_accelerations, magnetic_fields)

        heading_ema_alphas = [0.08, 0.09, 1.0, 1.1, 1.2]
        heading_ts = []
        heading_error_averaged = []

        for alpha in heading_ema_alphas:
            
            heading_ema_window = deque(maxlen=2) 
            heading_alpha_gps_ts = []
            heading_alpha_ema_averaged = []

            for ts, heading in zip(gps_ts, headings_with_imu):    
                
                if len(heading_ema_window) < 1:
                    heading_ema_window.append(heading)
                    heading_alpha_gps_ts.append(ts)
                    heading_alpha_ema_averaged.append(heading)

                else :
                    heading_ema = alpha * heading + (1 - alpha) * heading_ema_window[-1]
                    heading_ema_window.append(heading_ema)
                    heading_alpha_gps_ts.append(ts)
                    heading_alpha_ema_averaged.append(heading_ema)
            
            heading_ts.append(heading_alpha_gps_ts)
            heading_error_averaged.append(heading_alpha_ema_averaged)

        heading_ts = np.array(heading_ts)
        heading_error_averaged = np.array(heading_error_averaged)

        abs_raw_heading_errors = np.abs(raw_headings - gps_headings)
        avg_raw_heading_error = np.mean(abs_raw_heading_errors)

        abs_heading_with_imu_errors = np.abs(headings_with_imu - gps_headings)
        avg_heading_with_imu_errors = np.mean(abs_heading_with_imu_errors)

        abs_ema_heading_errors = np.abs(heading_error_averaged - gps_headings)
        avg_ema_heading_errors = np.mean(abs_ema_heading_errors, axis=1)

        best_heading_ema_idx = np.argmin(avg_ema_heading_errors)

        ###############################################################################################################################################################

        plt.figure(figsize=(12, 8))

        plt.title(f"Heading Difference", fontweight="bold", fontsize=24)
        plt.plot(gps_ts, abs_raw_heading_errors, label=f"Raw magnetometer heading diference against GPS heading | error={avg_raw_heading_error}", color='blue', linewidth=1)
        plt.plot(gps_ts, abs_heading_with_imu_errors, label=f"Raw magnetometer heading diference (IMU) against GPS heading | error={avg_heading_with_imu_errors}", color='red', linewidth=1)
        
        for i in range(len(heading_ema_alphas)):
            plt.plot(heading_ts[i], abs_ema_heading_errors[i], label=f"Raw heading difference (IMU + EMA) against GPS heading  | aplha={heading_ema_alphas[i]} | error={avg_ema_heading_errors[i]}", linewidth=1)
        

        plt.xlabel("Time since recording start in seconds", fontsize=18)
        plt.ylabel("Heading Difference in degrees", fontsize=18)

        plt.legend()
        
        plt.grid()
        plt.show()
        
        ###############################################################################################################################################################

        plt.figure(figsize=(12, 8))

        plt.title(f"Heading Difference", fontweight="bold", fontsize=24)
        plt.plot(gps_ts, abs_raw_heading_errors, label=f"Raw magnetometer heading diference against GPS heading | error={avg_raw_heading_error}", color='blue', linewidth=1)
        plt.plot(gps_ts, abs_heading_with_imu_errors, label=f"Raw magnetometer heading diference (IMU) against GPS heading | error={avg_heading_with_imu_errors}", color='red', linewidth=1)
        
        plt.plot(heading_ts[best_heading_ema_idx], abs_ema_heading_errors[best_heading_ema_idx], label=f"Best heading difference (IMU + EMA) against GPS heading  | aplha={heading_ema_alphas[best_heading_ema_idx]} | error={avg_ema_heading_errors[best_heading_ema_idx]}", linewidth=1)
        

        plt.xlabel("Time since recording start in seconds", fontsize=18)
        plt.ylabel("Heading Difference in degrees", fontsize=18)

        plt.legend()
        
        plt.grid()
        plt.show()
        
        ###############################################################################################################################################################

        fig, ax1 = plt.subplots()
        ax1.set_xlabel('Time since recording in seconds', fontsize=18)
        ax1.set_ylabel('Magnetometer heading error (IMU + EMA) in degrees', fontsize=18, color='blue')
        ax1.plot(heading_ts[best_heading_ema_idx], abs_ema_heading_errors[best_heading_ema_idx], label="Best Magnetometer heading error (IMU + EMA)", color="blue")
        ax1.tick_params(axis='y', labelcolor='blue')

        ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis
        ax2.set_ylabel('GPS speed in m/s', fontsize=18, color='red')  # we already handled the x-label with ax1
        ax2.plot(gps_ts, gps_speeds, label="GPS speed in m/s", color="red")
        ax2.tick_params(axis='y', labelcolor='red')

        plt.title("Heading error vs GPS speed", fontweight="bold", fontsize=22)

        plt.grid()
        plt.show()

        # ###########################################################################################################################################

        fig, ax1 = plt.subplots()
        ax1.set_xlabel('Time since recording in seconds', fontsize=18)
        ax1.set_ylabel('Best magnetometer heading error (IMU + EMA) in degrees', fontsize=18, color='blue')
        ax1.plot(heading_ts[best_heading_ema_idx], abs_ema_heading_errors[best_heading_ema_idx], label="Best magnetometer heading error (IMU + EMA)", color="blue")
        ax1.tick_params(axis='y', labelcolor='blue')

        ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis
        ax2.set_ylabel('GPS altitude in meters', fontsize=18, color='red')  # we already handled the x-label with ax1
        ax2.plot(gps_ts, gps_alts, label="GPS altitude in meters", color="red")
        ax2.tick_params(axis='y', labelcolor='red')

        plt.title("Heading error vs GPS altitude", fontweight="bold", fontsize=22)

        plt.grid()
        plt.show()

        # ###########################################################################################################################################
        
        plt.figure(figsize=(12, 8))

        plt.title(f"Lidar (IMU tilt compensated) vs EMA", fontweight="bold", fontsize=24)
        plt.plot(gps_ts, lidar_alts_imu_incorporated, label=f"Lidar Altitude (IMU Incorporated) | error={avg_error_lidar_imu}", color='blue', linewidth=1)
        
        for i in range(len(altitude_ema_aplhas)):
            plt.plot(ema_ts[i], ema_averaged[i], label=f"Lidar Altitude (IMU + EMA) | aplha={altitude_ema_aplhas[i]} | error={avg_errors_ema[i]}", linewidth=2)
            
        plt.plot(gps_ts, gps_alts, label="GPS Altitude", color='black', linewidth=1)

        plt.xlabel("Time since recording start in seconds", fontsize=18)
        plt.ylabel("Altitude in meters", fontsize=18)

        plt.legend()
        
        plt.grid()
        plt.show()

        ###############################################################################################################################################################

        plt.figure(figsize=(12, 8))

        plt.title(f"Lidar (IMU tilt compensated) vs Best EMA", fontweight="bold", fontsize=24)
        plt.plot(gps_ts, lidar_alts_imu_incorporated, label=f"Lidar Altitude (IMU Incorporated) | error={avg_error_lidar_imu}", color='blue', linewidth=1)
        
       
        plt.plot(ema_ts[best_error_idx], ema_averaged[best_error_idx], label=f"Lidar Altitude (IMU + EMA) | aplha={altitude_ema_aplhas[best_error_idx]} | error={avg_errors_ema[best_error_idx]}", color="red", linewidth=2)
            
        plt.plot(gps_ts, gps_alts, label="GPS Altitude", color='black', linewidth=1)

        plt.xlabel("Time since recording start in seconds", fontsize=18)
        plt.ylabel("Altitude in meters", fontsize=18)

        plt.legend()
        
        plt.grid()
        plt.show()

        ###############################################################################################################################################################

        plt.figure(figsize=(12, 8))

        plt.title(f"Lidar (IMU tilt compensated),  vs Best EMA", fontweight="bold", fontsize=24)
        plt.plot(gps_ts, lidar_alts_imu_incorporated, label=f"Lidar Altitude (IMU Incorporated) | error={avg_error_lidar_imu}", color='blue', linewidth=1)
        plt.plot(gps_ts, barometer_alts, label=f"Barometer Altitude | error={avg_error_barometer}", color="brown", linewidth=1)
       
        plt.plot(ema_ts[best_error_idx], ema_averaged[best_error_idx], label=f"Lidar Altitude (IMU + EMA) | aplha={altitude_ema_aplhas[best_error_idx]} | error={avg_errors_ema[best_error_idx]}", color="red", linewidth=2)
            
        plt.plot(gps_ts, gps_alts, label="GPS Altitude", color='black', linewidth=1)

        plt.xlabel("Time since recording start in seconds", fontsize=18)
        plt.ylabel("Altitude in meters", fontsize=18)

        plt.legend()
        
        plt.grid()
        plt.show()


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
