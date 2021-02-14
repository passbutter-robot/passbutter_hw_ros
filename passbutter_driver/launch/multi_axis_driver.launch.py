import launch
import launch_ros.actions
import os
import pathlib
import yaml

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('passbutter_driver'), 'config')
    param_config = os.path.join(config_dir, "multi_axis_driver.yaml")
    with open(param_config, 'r') as f:
        params = yaml.safe_load(f)["multi_axis_driver"]["ros__parameters"]

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='passbutter_driver', executable='stepper_driver', output='screen',
            name='axis_ctrl',
            parameters=[
                params
            ])
    ])