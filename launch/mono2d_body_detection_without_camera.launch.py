# Copyright (c) 2024，D-Robotics.
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
    model_file_name_launch_arg = DeclareLaunchArgument(
        "kps_model_file_name", default_value=TextSubstitution(text="config/multitask_body_head_face_hand_kps_960x544.hbm")
    )
    model_type_launch_arg = DeclareLaunchArgument(
        "kps_model_type", default_value=TextSubstitution(text="0")
    )
    track_mode_launch_arg = DeclareLaunchArgument(
        "kps_track_mode", default_value=TextSubstitution(text="1")
    )
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
            {"model_file_name": LaunchConfiguration('kps_model_file_name')},
            {"model_type": LaunchConfiguration('kps_model_type')},
            {"track_mode": LaunchConfiguration('kps_track_mode')},
            {"ai_msg_pub_topic_name": LaunchConfiguration(
                'mono2d_body_pub_topic')}
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    return LaunchDescription([
        model_file_name_launch_arg,
        model_type_launch_arg,
        track_mode_launch_arg,
        # image codec
        jpeg_codec_node,
        # body detection
        mono2d_body_pub_topic_arg,
        mono2d_body_det_node,
        # web display
        web_smart_topic_arg,
        web_node
    ])
