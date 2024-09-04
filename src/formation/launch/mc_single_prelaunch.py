from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os, yaml

def generate_launch_description():

    # get parameters from yaml file
    param_file_path = os.path.join(get_package_share_directory('formation'), 'config', 'params.yaml')

    with open(param_file_path, 'r') as file:
        params = yaml.safe_load(file)['/**']['ros__parameters']
        dds_baudrate = params['dds_baudrate']
        gcs_ip = params['gcs_ip']
    
    # declare launch arguments
    args = [DeclareLaunchArgument('dds_baudrate', default_value=str(dds_baudrate), description='DDS baudrate'),
            DeclareLaunchArgument('gcs_ip', default_value=str(gcs_ip), description='target GCS IP address')]

    dds_agent = ExecuteProcess(
                    cmd=[
                        'pgrep MicroXRCEAgent > /dev/null || MicroXRCEAgent serial --dev /dev/ttyS0 --baudrate', 
                        LaunchConfiguration('dds_baudrate'),
                    ],
                    shell=True,
                    output='screen',
                    name='dds_agent',
                )
    
    gcs_map = Node(
                package='formation',
                executable='serial_mapping_node',
                output='screen',
                shell=True,
                arguments=[LaunchConfiguration('gcs_ip'), '14550'],
            )

    return LaunchDescription([
        *args,
        dds_agent,
        gcs_map,
    ])