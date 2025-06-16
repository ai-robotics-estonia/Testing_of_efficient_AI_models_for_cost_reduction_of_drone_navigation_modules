import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
import launch.launch_description_sources
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess
import os
import time


def generate_launch_description():
    
    imu_package_dir = get_package_share_directory("mpu6050driver")
    camera_package_dir = get_package_share_directory("camera_pkgs")

    imu_launch_file = os.path.join(imu_package_dir, 'launch', 'mpu6050driver_launch.py')

    distance_node = Node(
            package='camera_pkgs',
            executable='publish_lidar',
            name='publish_lidar',
            output='screen',
            parameters=[]
        )

    trigger_node = Node(
            package='camera_pkgs',
            executable='trigger_node',
            name='trigger_node',
            output='screen',
            parameters=[]
        )

    # gps_node = Node(
            # package='camera_pkgs',
            # executable='publish_gps',
            # name='publish_gps',
            # output='screen',
            # parameters=[]
        # )    

    urdf_file = 'box.urdf'
    urdf_path = os.path.join(camera_package_dir, 'urdf', urdf_file)

    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()
    
    box_urdf_publisher= Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        )

    # bags_directory = "/home/rpi5/ros2_jazzy/src/drone/camera_pkgs/bags/"
    # os.makedirs(bags_directory, exist_ok=True)

    # bag_record = ExecuteProcess(
            # cmd=['ros2', 'bag', 'record', '-o', bags_directory+str(time.time()), 
                 # '--topics', '/barometer', 
                             # '/cam/front/image', 
                             # '/cam/bottom/image', 
                             # '/imu', 
                             # '/lidar', 
                             # '/magnetic_field',
                             # '/tf',
                             # '/tf_static',
                             # '/gps',
                             # '/heading'],
                # output='screen'
        # )   

    
    return LaunchDescription([        
        
        IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(imu_launch_file),
        ),
        
        box_urdf_publisher,
        distance_node,
        trigger_node,
        #gps_node,       

        # TimerAction(period=10.0,
                    # actions=[bag_record],
                    # ),               

    ])
