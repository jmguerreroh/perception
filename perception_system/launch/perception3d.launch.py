# MIT License

# Copyright (c) 2024 José Miguel Guerrero Hernández

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    perception_system_dir = get_package_share_directory('perception_system')
    yolo_dir = get_package_share_directory('yolo_bringup')

    model = LaunchConfiguration('model')
    model_arg = DeclareLaunchArgument(
        'model', default_value=os.path.join(perception_system_dir, 'models',
                                            'yolov8m-pose.pt'),
        description='Model name or path'
    )

    ns = LaunchConfiguration('namespace')
    ns_arg = DeclareLaunchArgument(
        'namespace', default_value='perception_system',
        description='namespace'
    )

    target_frame = LaunchConfiguration('target_frame')
    target_frame_arg = DeclareLaunchArgument(
        'target_frame', default_value='oak-d-base-frame',
        description='Target frame to transform the 3D boxes'
    )

    debug = LaunchConfiguration('debug')
    debug_arg = DeclareLaunchArgument(
        'debug', default_value='True', description='Debug mode')

    input_image_topic = LaunchConfiguration('input_image_topic')
    input_image_topic_arg = DeclareLaunchArgument(
        'input_image_topic',
        default_value='/rgb/image',
        description='Input image topic')

    input_depth_topic = LaunchConfiguration('input_depth_topic')
    input_depth_topic_arg = DeclareLaunchArgument(
        'input_depth_topic',
        default_value='/stereo/depth',
        description='Input depth topic')

    input_depth_info_topic = LaunchConfiguration('input_depth_info_topic')
    input_depth_info_topic_arg = DeclareLaunchArgument(
        'input_depth_info_topic',
        default_value='/stereo/camera_info',
        description='Input depth info topic')

    depth_image_units_divisor = LaunchConfiguration('depth_image_units_divisor')
    depth_image_units_divisor_arg = DeclareLaunchArgument(
        'depth_image_units_divisor', default_value='1000',
        # 1 value for simulation, 1000 value in real robot
        description='Depth image units divisor')

    threshold = LaunchConfiguration("threshold")
    threshold_arg = DeclareLaunchArgument(
        "threshold",
        default_value="0.5",
        description="Minimum probability of a detection to be published")

    yolo3d_launch = os.path.join(yolo_dir, 'launch', 'yolo.launch.py')
    yolo3d = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(yolo3d_launch),
        launch_arguments={
            'namespace': ns,
            'model': model,
            'input_image_topic': input_image_topic,
            'input_depth_topic': input_depth_topic,
            'input_depth_info_topic': input_depth_info_topic,
            'depth_image_units_divisor': depth_image_units_divisor,
            'target_frame': target_frame,
            'threshold': threshold,
            'use_3d': 'True',
            }.items()
    )

    people_detection_node = Node(
        package='perception_system',
        namespace=ns,
        executable='dt_people',
        output='both',
        emulate_tty=True,
        parameters=[
            {'target_frame': target_frame},
            {'debug': debug},
        ],
    )

    objects_detection_node = Node(
        package='perception_system',
        namespace=ns,
        executable='dt_objects',
        output='both',
        emulate_tty=True,
        parameters=[
            {'target_frame': target_frame},
            {'classes': 'all'},
            {'debug': debug},
        ],
    )

    debug_node = Node(
        package='perception_system',
        namespace=ns,
        executable='dt_debug',
        output='both',
        emulate_tty=True,
        parameters=[
            {'target_frame': target_frame},
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(model_arg)
    ld.add_action(target_frame_arg)
    ld.add_action(ns_arg)
    ld.add_action(debug_arg)
    ld.add_action(input_image_topic_arg)
    ld.add_action(input_depth_topic_arg)
    ld.add_action(input_depth_info_topic_arg)
    ld.add_action(depth_image_units_divisor_arg)
    ld.add_action(threshold_arg)

    ld.add_action(yolo3d)

    ld.add_action(people_detection_node)
    ld.add_action(objects_detection_node)
    ld.add_action(debug_node)

    return ld
