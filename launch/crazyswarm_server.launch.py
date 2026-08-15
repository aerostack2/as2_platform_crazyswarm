# Copyright 2024 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Launch the Crazyswarm2 Python server (cflib backend).

Pass mocap:=true to also start the mocap_bridge node, which forwards
rigid-body poses from the mocap4r2 OptiTrack driver (/mocap/rigid_bodies)
to the Crazyswarm2 server via the /poses topic.

The mocap4r2 driver must be running separately (ground_station.yaml with
mocap4ros2:=true).  Rigid-body names in Motive must match the CF names in
crazyflies.yaml (e.g. "cf1").
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
import yaml


def launch_server(context):
    from launch.substitutions import LaunchConfiguration

    crazyflies_yaml = LaunchConfiguration('crazyflies_yaml_file').perform(context)
    with open(crazyflies_yaml, 'r') as f:
        crazyflies = yaml.safe_load(f)

    server_yaml = LaunchConfiguration('server_yaml_file').perform(context)
    with open(server_yaml, 'r') as f:
        server_params = yaml.safe_load(f)['/crazyflie_server']['ros__parameters']

    urdf = LaunchConfiguration('urdf_file').perform(context)
    with open(urdf, 'r') as f:
        server_params['robot_description'] = f.read()

    nodes = [
        Node(
            package='crazyflie_server_py',
            executable='crazyflie_server',
            name='crazyflie_server',
            output='screen',
            parameters=[crazyflies, server_params],
        )
    ]

    mocap_enabled = LaunchConfiguration('mocap').perform(context).lower() in ('true', '1', 'yes')
    if mocap_enabled:
        nodes.append(
            Node(
                package='as2_platform_crazyswarm',
                executable='mocap_bridge',
                name='mocap_bridge',
                output='screen',
                parameters=[{
                    'config_file':
                        LaunchConfiguration('mocap_config_file').perform(context)}],
            )
        )

    return nodes


def generate_launch_description():
    config_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'config')
    default_yaml = os.path.join(config_dir, 'crazyflies.yaml')
    default_server_yaml = os.path.join(config_dir, 'crazyswarm_server.yaml')
    default_urdf = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        '..', 'crazyswarm2', 'crazyflie', 'urdf', 'crazyflie_description.urdf')

    return LaunchDescription([
        DeclareLaunchArgument(
            'crazyflies_yaml_file',
            default_value=default_yaml,
            description='Path to the crazyflies configuration YAML'),
        DeclareLaunchArgument(
            'server_yaml_file',
            default_value=default_server_yaml,
            description='Path to the crazyflie server defaults YAML'),
        DeclareLaunchArgument(
            'urdf_file',
            default_value=default_urdf,
            description='Path to the Crazyflie URDF description'),
        DeclareLaunchArgument(
            'mocap',
            default_value='false',
            description='Set to true to launch the mocap_bridge node'),
        DeclareLaunchArgument(
            'mocap_config_file',
            default_value='',
            description='Path to config YAML with mocap_id/cf_name mapping '
                        '(required when mocap:=true)'),
        OpaqueFunction(function=launch_server),
    ])
