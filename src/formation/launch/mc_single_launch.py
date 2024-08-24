import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():

    amc_id = 1 # from 1 to 3
    dds_conn_dev = '/dev/ttyS0'
    gcs_ip = '192.168.1.111'

    node = Node(
                package='formation',
                executable='mc_formation_control_node',
                output='screen',
                shell=True,
                arguments=['amc_' + str(amc_id)],
            )
    
    dds_agent = ExecuteProcess(
                    cmd=[[
                        'MicroXRCEAgent serial --dev ' + dds_conn_dev,
                    ]],
                    shell=True,
                    output='screen',
                )

    gcs_map = Node(
                package='formation',
                executable='serial_mapping_node',
                output='screen',
                shell=True,
                arguments=[gcs_ip, '14550'],
            )

    return LaunchDescription([
        dds_agent,
        gcs_map,
        node,
    ])