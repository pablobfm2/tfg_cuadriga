# Copyright (c) 2020, Open Source Robotics Foundation.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#    * Neither the name of the copyright holder nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGf, edfñeHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Launch the joy node with default configuration."""
import os

import ament_index_python.packages
import launch
import launch_ros.actions


def generate_launch_description():

    config_directory = os.path.join(
        ament_index_python.packages.get_package_share_directory('joy'),
        'config')
    params = os.path.join(config_directory, 'joy-params.yaml')
    joy_node = launch_ros.actions.Node(package='joy',
                                       executable='joy_node',
                                       output='both',
                                       parameters=[params])

    return launch.LaunchDescription([joy_node,

                                     launch.actions.RegisterEventHandler(
                                         event_handler=launch.event_handlers.OnProcessExit(
                                             target_action=joy_node,
                                             on_exit=[launch.actions.EmitEvent(
                                                 event=launch.events.Shutdown())],
                                         )),
                                     ])
