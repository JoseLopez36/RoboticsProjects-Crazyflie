import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # load crazyflies
    crazyflies_yaml = os.path.join(
        get_package_share_directory('roboticsprojects_crazyflie'),
        'config',
        'single_agent_crazyflie_sim.yaml')

    with open(crazyflies_yaml, 'r') as ymlfile:
        crazyflies = yaml.safe_load(ymlfile)

    fileversion = 1
    if "fileversion" in crazyflies:
        fileversion = crazyflies["fileversion"]

    # server params
    server_yaml = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'server.yaml')

    with open(server_yaml, 'r') as ymlfile:
        server_yaml_content = yaml.safe_load(ymlfile)

    server_params = [crazyflies] + [server_yaml_content['/crazyflie_server']['ros__parameters']]
    # robot description
    urdf = os.path.join(
        get_package_share_directory('crazyflie'),
        'urdf',
        'crazyflie_description.urdf')
    
    with open(urdf, 'r') as f:
        robot_desc = f.read()

    server_params[1]['robot_description'] = robot_desc
    
    launch_description = []
    launch_description.append(
        Node(
            package='crazyflie',
            executable='crazyflie_server.py',
            name='crazyflie_server',
            output='screen',
            parameters=server_params
        ))
    
    launch_description.append(
        Node(
            package='roboticsprojects_crazyflie',
            executable='TransformWorld2Odom.py',
            name='TransformWorld2Odom',
            output='screen'
        ))
        
    # Add vel_mux nodes dynamically based on the number parameter
    for i in range(1, int(os.environ.get('NUM_ROBOTS', '1')) + 1):
        namespace = f'cf_{i}'
        vel_mux_node = Node(
            package='crazyflie',
            executable='vel_mux.py',
            name=f'vel_mux{i}',
            output='screen',
            namespace=namespace,
            parameters=[
                {"hover_height": 1.0},
                {"incoming_twist_topic": "cmd_vel"},
                {"robot_prefix": f"/{namespace}"}
            ]
        )
        launch_description.append(vel_mux_node)
    
    launch_description.append(       
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory('roboticsprojects_crazyflie'), 'config', 'config.rviz')],
            parameters=[{
                "use_sim_time": True,
            }]
        )
        
    )
    return LaunchDescription(launch_description)
