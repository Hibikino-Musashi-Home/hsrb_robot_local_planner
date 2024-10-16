#!/usr/bin/env python
'''
Copyright (c) 2024 TOYOTA MOTOR CORPORATION
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
'''
# -*- coding: utf-8 -*-

import collections
import math

from geometry_msgs.msg import Pose as RosPose

Vector3 = collections.namedtuple('Vector3', 'x y z')
Quaternion = collections.namedtuple('Quaternion', 'x y z w')
Pose = collections.namedtuple('Pose', 'pos ori')


def pose(x=0.0, y=0.0, z=0.0, ei=0.0, ej=0.0, ek=0.0, axes='sxyz'):
    """Create a new pose-tuple representation.

    Args:
        x, y, z: Linear translation.
        ei, ej, ek, axes: Rotation in euler form.
            By default, (ei, ej, ek) are correspond to (roll, pitch, yaw).

    Returns:
        Tuple[Vector3, Quaternion]: A new pose.
    """
    vec3 = (x, y, z)
    # https://github.com/dlu/tf_transformations also needs to install PIP dependent package installation
    # There is no method to complete with Rosdep (apt), so give up and implement it
    roll_2 = ei / 2.0
    pitch_2 = ej / 2.0
    yaw_2 = ek / 2.0
    qx = math.sin(roll_2) * math.cos(pitch_2) * math.cos(yaw_2) - math.cos(roll_2) * math.sin(pitch_2) * math.sin(yaw_2)
    qy = math.cos(roll_2) * math.sin(pitch_2) * math.cos(yaw_2) + math.sin(roll_2) * math.cos(pitch_2) * math.sin(yaw_2)
    qz = math.cos(roll_2) * math.cos(pitch_2) * math.sin(yaw_2) - math.sin(roll_2) * math.sin(pitch_2) * math.cos(yaw_2)
    qw = math.cos(roll_2) * math.cos(pitch_2) * math.cos(yaw_2) + math.sin(roll_2) * math.sin(pitch_2) * math.sin(yaw_2)
    return Pose(Vector3(*vec3), Quaternion(qx, qy, qz, qw))


def tuples_to_pose(tuples):
    """Convert a pose-tuple representation to a ``geometry_msgs/Pose``.

    Args:
        tuples (Tuple[Vector3, Quaternion]): A pose-tuple representation.

    Returns:
        geometry_msgs.msg.Pose: A result of conversion.
    """
    trans, rot = tuples
    pose = RosPose()
    pose.position.x = trans[0]
    pose.position.y = trans[1]
    pose.position.z = trans[2]
    pose.orientation.x = rot[0]
    pose.orientation.y = rot[1]
    pose.orientation.z = rot[2]
    pose.orientation.w = rot[3]
    return pose


def transform_to_tuples(transform):
    """Convert a ``geometry_msgs/Transform`` to a pose-tuple representation.

    Args:
        transform (geometry_msgs.msg.Transform): A transform message.

    Returns:
        tuples (Tuple[Vector3, Quaternion]): A result of conversion.
    """
    x = transform.translation.x
    y = transform.translation.y
    z = transform.translation.z
    qx = transform.rotation.x
    qy = transform.rotation.y
    qz = transform.rotation.z
    qw = transform.rotation.w
    return Pose(Vector3(x, y, z), Quaternion(qx, qy, qz, qw))
