import launch
import launch_ros.actions
import os
import pathlib
import yaml

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('passbutter_driver'), 'config')
    param_config = os.path.join(config_dir, "wheel_command.yaml")
    with open(param_config, 'r') as f:
        wheels_0_params = yaml.safe_load(f)["wheels_0"]["ros__parameters"]

    with open(param_config, 'r') as f:
        wheels_1_params = yaml.safe_load(f)["wheels_1"]["ros__parameters"]

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='passbutter_driver', executable='wheel_command', output='screen',
            name='axis_0',
            parameters=[
                wheels_0_params
            ]),
        launch_ros.actions.Node(
            package='passbutter_driver', executable='wheel_command', output='screen',
            name='axis_1',
            parameters=[
                wheels_1_params
            ])
    ])