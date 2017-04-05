# -*- coding: utf-8 -*-
#
# Software License Agreement (BSD License)

# Copyright (c) 2010-2011, Antons Rebguns.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of University of Arizona nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

__author__ = 'Antons Rebguns'
__copyright__ = 'Copyright (c) 2010-2011 Antons Rebguns'
__license__ = 'BSD'
___maintainer__ = 'Max Svetlik'
__email__ = 'max@svenzva.com'



import math

import rospy

from mx_driver.dynamixel_const import *

from mx_controllers.srv import SetSpeed
from mx_controllers.srv import TorqueEnable
from mx_controllers.srv import SetTorqueLimit

from std_msgs.msg import Float64
from mx_msgs.msg import MotorStateList
from mx_msgs.msg import JointState

class JointController:
    def __init__(self, dxl_io, controller_namespace, port_namespace):
        self.running = False
        self.dxl_io = dxl_io
        self.controller_namespace = controller_namespace
        self.port_namespace = port_namespace
        self.joint_name = rospy.get_param(self.controller_namespace + '/joint_name')
        self.joint_speed = rospy.get_param(self.controller_namespace + '/joint_speed', 1.0)
        self.torque_limit = rospy.get_param(self.controller_namespace + '/joint_torque_limit', None)

        self.speed_service = rospy.Service(self.controller_namespace + '/set_speed', SetSpeed, self.process_set_speed)
        self.torque_service = rospy.Service(self.controller_namespace + '/torque_enable', TorqueEnable, self.process_torque_enable)

        self.torque_limit_service = rospy.Service(self.controller_namespace + '/set_torque_limit', SetTorqueLimit, self.process_set_torque_limit)

        if self.torque_limit is not None:
            if self.torque_limit < 0: self.torque_limit = 0.0
            elif self.torque_limit > 1: self.torque_limit = 1.0

    def initialize(self):
        raise NotImplementedError

    def start(self):
        self.running = True
        self.joint_state_pub = rospy.Publisher(self.controller_namespace + '/state', JointState, queue_size=1)
        self.command_sub = rospy.Subscriber(self.controller_namespace + '/command', Float64, self.process_command)
        self.motor_states_sub = rospy.Subscriber('motor_states/%s' % self.port_namespace, MotorStateList, self.process_motor_states)

    def stop(self):
        self.running = False
        self.joint_state_pub.unregister()
        self.motor_states_sub.unregister()
        self.command_sub.unregister()
        self.speed_service.shutdown('normal shutdown')
        self.torque_service.shutdown('normal shutdown')

    def set_torque_enable(self, torque_enable):
        raise NotImplementedError

    def set_speed(self, speed):
        raise NotImplementedError

    def set_torque_limit(self, max_torque):
        raise NotImplementedError

    def process_set_speed(self, req):
        self.set_speed(req.speed)
        return [] # success

    def process_torque_enable(self, req):
        self.set_torque_enable(req.torque_enable)
        return []

    def process_set_torque_limit(self, req):
        self.set_torque_limit(req.torque_limit)
        return []

    def process_motor_states(self, state_list):
        raise NotImplementedError

    def process_command(self, msg):
        raise NotImplementedError

    def rad_to_raw(self, angle, initial_position_raw, flipped, encoder_ticks_per_radian):
        """ angle is in radians """
        angle_raw = angle * encoder_ticks_per_radian
        return int(round(initial_position_raw - angle_raw if flipped else initial_position_raw + angle_raw))
    def raw_to_rad(self, raw, initial_position_raw, flipped, radians_per_encoder_tick):
        return raw * radians_per_encoder_tick
