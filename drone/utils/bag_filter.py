import rclpy
from rosbag2_py import SequentialReader, SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import Imu
from scipy.signal import butter, filtfilt
import numpy as np
import argparse
import os

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--directory",
                        type=str,
                        default="/home/rpi5/ros2_jazzy/src/drone/camera_pkgs/bags/data_1749031248.4807634_0_filtered", 
                        help="Directory to the ROS2 bag file."
                        )
    parser.add_argument("-f", "--filename",
                        type=str,
                        default="data_1749031248.4807634_0.mcap", 
                        help="Name of the original ROS2 bag file."
                        )
    parser.add_argument("-o", "--output",
                        type=str,
                        default="data_1749031248.4807634_0_filtered_0.mcap", 
                        help="Name of the output ROS2 bag file where the filtered IMU data is added."
                        )
    
    return parser.parse_args()

def butter_lowpass_filter(data, cutoff, fs, order=4):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    # print(normal_cutoff)
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return filtfilt(b, a, data)

def filter_imu_msgs(raw_imu_msgs, fs, cutoff):
    ax = [msg.linear_acceleration.x for msg in raw_imu_msgs]
    ay = [msg.linear_acceleration.y for msg in raw_imu_msgs]
    az = [msg.linear_acceleration.z for msg in raw_imu_msgs]

    ax_filterd = butter_lowpass_filter(ax, cutoff, fs)
    ay_filterd = butter_lowpass_filter(ay, cutoff, fs)
    az_filterd = butter_lowpass_filter(az, cutoff, fs)

    filtered_msgs = []
    for i, msg in enumerate(raw_imu_msgs):
        filtered_imu_msg = msg
        filtered_imu_msg.linear_acceleration.x = ax_filterd[i]
        filtered_imu_msg.linear_acceleration.y = ay_filterd[i]
        filtered_imu_msg.linear_acceleration.z = az_filterd[i]
        filtered_msgs.append(filtered_imu_msg)

    return filtered_msgs

def add_imu_filtered(original_bag_path, new_bag_path, imu_topic="/imu", filtered_topic="/imu/filtered"):
    

    storage_options = StorageOptions(uri=original_bag_path, storage_id='mcap')
    converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')

    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    topic_types = reader.get_all_topics_and_types()
    # type_map = {topic.name: topic.type for topic in topic_types}

    imu_msgs = []
    imu_timestamps = []
    all_msgs = []
    
    while reader.has_next():
        topic, data, t = reader.read_next()
        all_msgs.append((topic, data, t))

        if topic == imu_topic:
            msg = deserialize_message(data, Imu)
            imu_msgs.append(msg)
            imu_timestamps.append(t*1e-9)


    if not imu_msgs:
        print(f"No messages found in topic: {imu_topic}")
        rclpy.shutdown()
        return
    
    dt = np.diff(np.array(imu_timestamps))
    fs = 5 * (1 / np.mean(dt))
    cutoff = 10.0

    print(f"Found {len(imu_msgs)} Imu messages. Filtering ...")
    filtered_imu_msgs = filter_imu_msgs(imu_msgs, fs, cutoff)

    # Open new writer
    writer = SequentialWriter()
    writer.open(StorageOptions(uri=new_bag_path, storage_id='mcap'),
                converter_options)
    
    # Recreate original topics    
    for topic in topic_types:
        writer.create_topic(topic)

    filtered_topic_metadata = TopicMetadata(id=100,
                                            name=filtered_topic,
                                            type='sensor_msgs/msg/Imu',
                                            serialization_format='cdr')
    

    # Add new filtered topic
    writer.create_topic(filtered_topic_metadata)        
    
    # Write original messages
    for topic, data, t in all_msgs:
        writer.write(topic, data, t)
    
    # Write filtered messages to new topic
    for i,t in enumerate(imu_timestamps):
        filtered_msg = filtered_imu_msgs[i]
        writer.write(filtered_topic, serialize_message(filtered_msg), int(t*1e9))

    
    
if __name__ == "__main__":

    args = parse_args()     # Get the bags directory and bag filename from command line, or from default values of the arguments
    original_bag_filepath = os.path.join(args.directory, args.filename)
    output_bag_filepath = os.path.join(args.directory, args.output)


    try:
        rclpy.init()
        add_imu_filtered(original_bag_path=original_bag_filepath,
                        new_bag_path=output_bag_filepath)
    
    except Exception as e:
        print(f"Error: {e}")

    except KeyboardInterrupt:
        print("Keyboard Interrupt !!")

    finally:
        print(f"New bag with filtered IMU msgs saved to {output_bag_filepath}")
        rclpy.shutdown()