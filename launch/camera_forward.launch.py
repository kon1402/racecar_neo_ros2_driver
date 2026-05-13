"""Standalone forward camera launch (Logitech BRIO via gscam)."""

from racecar_neo_ros2_driver.launch_common import single_node_launch


def generate_launch_description():
    return single_node_launch(
        arg_name='camera_forward_config',
        default_yaml='camera_forward.yaml',
        package='gscam',
        executable='gscam_node',
        node_name='camera_forward',
        remappings=[
            ('camera/image_raw', '/camera/forward'),
            ('camera/camera_info', '/camera/forward/camera_info'),
        ],
    )
