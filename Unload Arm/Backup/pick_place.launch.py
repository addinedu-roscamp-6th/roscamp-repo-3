from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='usb_cam',
            parameters=[
                {'device': '/dev/video0'},
                {'image_size': [640, 480]},
                {'pixel_format': 'mjpeg'}
            ],
            output='screen'
        ),
        Node(
            package='yolov5_detector',
            executable='yolov5_node',      # setup.cfg에 등록한 이름
            name='yolov5_detector',
            parameters=[{'weights': '/home/jetcobot/weights/best.pt'},
                        {'image_topic': '/image_raw'}
                        ],
            output='screen'
        ),
        Node(
            package='jetcobot_pickplace',
            executable='pick_place_node',  # setup.cfg에 등록한 이름
            name='jetcobot_pickplace',
            parameters=[{'camera_device': '/dev/video0'},
                        {'speed': 30}],
            output='screen'
        )
    ])
