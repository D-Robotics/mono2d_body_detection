# Copyright (c) 2022，Horizon Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    camera_type = os.getenv('CAM_TYPE')
    print("camera_type is ", camera_type)

    camera_node = None
    camera_type_mipi = None
    camera_device_arg = None

    if camera_type == "fb":
        print("using feedback")
        # local image publish
        feedback_picture_arg = DeclareLaunchArgument(
            'picture',
            default_value='./config/test.jpg',
            description='feedback picture')
        fb_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('hobot_image_publisher'),
                    'launch/hobot_image_publisher.launch.py')),
            launch_arguments={
                'publish_image_source': LaunchConfiguration('picture'),
                'publish_image_format': 'jpg',
                'publish_output_image_w': '960',
                'publish_output_image_h': '544',
                'publish_message_topic_name': '/hbmem_img',
                'publish_fps': '10'
            }.items()
        )
        camera_node = fb_node
        camera_type_mipi = True
        camera_device_arg = feedback_picture_arg
    elif camera_type == "usb":
        print("using usb camera")
        # using usb cam publish image
        usb_cam_device_arg = DeclareLaunchArgument(
            'device',
            default_value='/dev/video8',
            description='usb camera device')

        usb_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('hobot_usb_cam'),
                    'launch/hobot_usb_cam.launch.py')),
            launch_arguments={
                'usb_image_width': '640',
                'usb_image_height': '480',
                'usb_video_device': LaunchConfiguration('device')
            }.items()
        )

        camera_node = usb_node
        camera_type_mipi = False
        camera_device_arg = usb_cam_device_arg
    else:
        print("using mipi cam")
        # using mipi cam publish image
        mipi_cam_device_arg = DeclareLaunchArgument(
            'device',
            default_value='F37',
            description='mipi camera device')

        mipi_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('mipi_cam'),
                    'launch/mipi_cam.launch.py')),
            launch_arguments={
                'mipi_image_width': '960',
                'mipi_image_height': '544',
                'mipi_io_method': 'shared_mem',
                'mipi_video_device': LaunchConfiguration('device')
            }.items()
        )

        camera_node = mipi_node
        camera_type_mipi = True
        camera_device_arg = mipi_cam_device_arg

    # nv12->jpeg
    jpeg_codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_codec'),
                'launch/hobot_codec_encode.launch.py')),
        launch_arguments={
            'codec_in_mode': 'shared_mem',
            'codec_out_mode': 'ros',
            'codec_sub_topic': '/hbmem_img',
            'codec_pub_topic': '/image'
        }.items()
    )

    # jpeg->nv12
    nv12_codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_codec'),
                'launch/hobot_codec_decode.launch.py')),
        launch_arguments={
            'codec_in_mode': 'ros',
            'codec_out_mode': 'shared_mem',
            'codec_sub_topic': '/image',
            'codec_pub_topic': '/hbmem_img'
        }.items()
    )

    # web
    web_smart_topic_arg = DeclareLaunchArgument(
        'smart_topic',
        default_value='/hobot_mono2d_body_detection',
        description='websocket smart topic')
    web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('websocket'),
                'launch/websocket.launch.py')),
        launch_arguments={
            'websocket_image_topic': '/image',
            'websocket_smart_topic': LaunchConfiguration('smart_topic')
        }.items()
    )

    # mono2d body detection
    mono2d_body_pub_topic_arg = DeclareLaunchArgument(
        'mono2d_body_pub_topic',
        default_value='/hobot_mono2d_body_detection',
        description='mono2d body ai message publish topic')
    mono2d_body_det_node = Node(
        package='mono2d_body_detection',
        executable='mono2d_body_detection',
        output='screen',
        parameters=[
            {"ai_msg_pub_topic_name": LaunchConfiguration(
                'mono2d_body_pub_topic')}
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    if camera_type_mipi:
        return LaunchDescription([
            camera_device_arg,
            # image publish
            camera_node,
            # image codec
            jpeg_codec_node,
            # body detection
            mono2d_body_pub_topic_arg,
            mono2d_body_det_node,
            # web display
            web_smart_topic_arg,
            web_node
        ])
    else:
        return LaunchDescription([
            camera_device_arg,
            # image publish
            camera_node,
            # image codec
            nv12_codec_node,
            # body detection
            mono2d_body_pub_topic_arg,
            mono2d_body_det_node,
            # web display
            web_smart_topic_arg,
            web_node
        ])
