"""Standalone pwm_node launch — owns /dev/maestro; watchdog TERM→KILL before restart."""

from racecar_neo_ros2_driver.launch_common import single_node_launch


def generate_launch_description():
    return single_node_launch(
        arg_name='pwm_config',
        default_yaml='pwm.yaml',
        package='racecar_neo_ros2_driver',
        executable='pwm_node',
    )
