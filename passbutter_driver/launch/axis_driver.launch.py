import launch
import launch_ros.actions
import os
import pathlib
import yaml

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('passbutter_driver'), 'config')
    param_config = os.path.join(config_dir, "axis_driver.yaml")
    with open(param_config, 'r') as f:
        axis_0_params = yaml.safe_load(f)["axis_0"]["ros__parameters"]

    with open(param_config, 'r') as f:
        axis_1_params = yaml.safe_load(f)["axis_1"]["ros__parameters"]

    with open(param_config, 'r') as f:
        axis_2_params = yaml.safe_load(f)["axis_2"]["ros__parameters"]

    with open(param_config, 'r') as f:
        axis_3_params = yaml.safe_load(f)["axis_3"]["ros__parameters"]

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='passbutter_driver', executable='single_stepper_driver', output='screen',
            name='axis_0',
            parameters=[
                axis_0_params
            ]),
        launch_ros.actions.Node(
            package='passbutter_driver', executable='single_stepper_driver', output='screen',
            name='axis_1',
            parameters=[
                axis_1_params
            ]),
        launch_ros.actions.Node(
            package='passbutter_driver', executable='single_stepper_driver', output='screen',
            name='axis_2',
            parameters=[
                axis_2_params
            ]),
        launch_ros.actions.Node(
            package='passbutter_driver', executable='single_stepper_driver', output='screen',
            name='axis_3',
            parameters=[
                axis_3_params
            ])
    ])