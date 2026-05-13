"""Standalone backward camera launch (Arducam B0578 via gscam)."""

from racecar_neo_ros2_driver.launch_common import single_node_launch


def generate_launch_description():
    return single_node_launch(
        arg_name='camera_backward_config',
        default_yaml='camera_backward.yaml',
        package='gscam',
        executable='gscam_node',
        node_name='camera_backward',
        remappings=[
            ('camera/image_raw', '/camera/backward'),
            ('camera/camera_info', '/camera/backward/camera_info'),
        ],
    )
