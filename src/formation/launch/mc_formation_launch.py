import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():

    dds_agent = ExecuteProcess(
                    cmd=[
                        'pgrep MicroXRCEAgent > /dev/null || MicroXRCEAgent udp4 --port 8888',
                    ],
                    shell=True,
                    output='screen',
                    name='dds_agent',
                )
    
    node = []
    for i in range(3):
        node.append(
            Node(
                package='formation',
                executable='mc_formation_control_node',
                output='screen',
                shell=True,
                arguments=[str(i + 1)],
                parameters=[{'test_phase': 'formation'}] # 'test_phase': 'single' or 'formation'
            )
        )
    delay_amc = TimerAction(period=10.0, actions=[*node])

    px4_client = []
    for i in range(3):
        px4_workdir = os.path.expanduser('~') + '/PX4-Autopilot'
        px4_env = { 'PX4_SYS_AUTOSTART': '4001', 
                    'PX4_GZ_MODEL': 'x500', 
                    'PX4_GZ_MODEL_POSE': '0,' + str(i * 2) + ',0.5'}
        if i > 0:
            px4_env['HEADLESS'] = '1'
        px4_client.append( 
            ExecuteProcess(
                cmd=[[
                    px4_workdir + '/build/px4_sitl_default/bin/px4 -i ' + str(i + 1),
                ]],
                shell=True,
                additional_env=px4_env,
                output='screen',
            )
        )

    return LaunchDescription([
        dds_agent,
        *px4_client,
        delay_amc
    ])