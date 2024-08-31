import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():

    # declare launch arguments
    args = [DeclareLaunchArgument('amc_id', default_value='1', description='AMC ID'), 
            DeclareLaunchArgument('test_phase', default_value='single', description='test phase'),
            DeclareLaunchArgument('lasting_time', default_value='30', description='fly lasting time'),
            DeclareLaunchArgument('origin_lat', default_value='24.43758', description='origin latitude'),
            DeclareLaunchArgument('origin_lon', default_value='118.09782', description='origin longitude'),
            DeclareLaunchArgument('dds_baudrate', default_value='2000000', description='DDS baudrate'),
            DeclareLaunchArgument('gcs_ip', default_value='127.0.0.1', description='target GCS IP address')]

    dds_agent = ExecuteProcess(
                    cmd=[
                        'pgrep MicroXRCEAgent > /dev/null || MicroXRCEAgent serial --dev /dev/ttyS0 --baudrate', 
                        LaunchConfiguration('dds_baudrate')
                    ],
                    shell=True,
                    output='screen',
                    name='dds_agent',
                )
    
    node = Node(
                package='formation',
                executable='mc_formation_control_node',
                output='screen',
                shell=True,
                arguments=[LaunchConfiguration('amc_id')],
                parameters=[{'test_phase': LaunchConfiguration('test_phase'),
                             'lasting_time': LaunchConfiguration('lasting_time'),
                             'origin_lat': LaunchConfiguration('origin_lat'),
                             'origin_lon': LaunchConfiguration('origin_lon')}],
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
        node,
    ])