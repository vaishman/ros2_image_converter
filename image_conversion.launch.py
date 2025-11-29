from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[{
                'video_device': '/dev/video0',
                'framerate': 30.0,
                'pixel_format': 'yuyv',
                'image_width': 640,
                'image_height': 480
            }]
        ),
        Node(
            package='image_conversion_pkg',
            executable='image_conversion',
            name='image_conversion_node',
            output='screen',
            parameters=[{
                'input_topic': '/image_raw',
                'output_topic': '/image_converted'
            }]
        )
    ])
