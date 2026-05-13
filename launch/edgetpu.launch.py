"""Standalone edgetpu_node launch (watchdog restart target)."""

from racecar_neo_ros2_driver.launch_common import single_node_launch


def generate_launch_description():
    return single_node_launch(
        arg_name='edgetpu_config',
        default_yaml='edgetpu.yaml',
        package='racecar_neo_ros2_driver',
        executable='edgetpu_node',
    )
