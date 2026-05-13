"""Standalone dotmatrix_node launch (watchdog restart target)."""

from racecar_neo_ros2_driver.launch_common import single_node_launch


def generate_launch_description():
    return single_node_launch(
        arg_name='dotmatrix_config',
        default_yaml='dotmatrix.yaml',
        package='racecar_neo_ros2_driver',
        executable='dotmatrix_node',
        description='MAX7219 dot matrix parameters',
    )
