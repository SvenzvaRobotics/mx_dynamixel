# -*- coding: utf-8 -*-
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2017, Svenzva Robotics
# Copyright (c) 2010-2011, Cody Jorgensen, Antons Rebguns.
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


__author__ = 'Max Svetlik, Cody Jorgensen, Antons Rebguns'
__copyright__ = 'Copyright (c) 2017 Svenzva Robotics, Copyright (c) 2010-2011 Cody Jorgensen, Antons Rebguns'

__license__ = 'BSD'
__maintainer__ = 'Max Svetlik'
__email__ = 'max@svenzva.com'


"""
Dynamixel Constants
Note that for multi-address areas, the designated address is the starting address.
The number of bytes is then inferred by the set/get commands on a per-instruction basis
"""

#PERSISTANT MEMORY
MX_MODEL_NUMBER = 0
MX_MODEL_INFORMATION = 2
MX_VERSION = 6
MX_ID = 7
MX_BAUD_RATE = 8
MX_RETURN_DELAY_TIME = 9
MX_DRIVE_MODE = 10
MX_OPERATING_MODE = 11
MX_SHADOW_ID = 12
MX_PROTOCOL_VERSION = 13
MX_HOMING_OFFSET = 20
MX_MOVING_THRESHOLD = 24
MX_TEMPERATURE_LIMIT = 31
MX_MAX_VOLTAGE_LIMIT = 32
MX_MIN_VOLTAGE_LIMIT = 34
MX_PWM_LIMIT = 36
MX_CURRENT_LIMIT = 38
MX_ACCELERATION_LIMIT = 40
MX_VELOCITY_LIMIT = 44
MX_MAX_POSITION_LIMIT = 48
MX_MIN_POSITION_LIMIT = 52
MX_SHUTDOWN = 63
#RAM
MX_TORQUE_ENABLE = 64
MX_LED = 65
MX_STATUS_RETURN_LEVEL = 68
MX_REGISTERED_INSTRUCTION = 69
MX_HARDWARE_ERROR_STATUS = 70
MX_VELOCITY_I_GAIN = 76
MX_VELOCITY_P_GAIN = 78
MX_POSITION_D_GAIN = 80
MX_POSITION_I_GAIN = 82
MX_POSITION_P_GAIN = 84
MX_FEEDFORWARD_2_GAIN = 88
MX_FEEDFORWARD_1_GAIN = 90
MX_BUS_WATCHDOG = 98
MX_GOAL_PWM = 100
MX_GOAL_CURRENT = 102
MX_GOAL_VELOCITY = 104
MX_PROFILE_ACCELERATION = 108
MX_PROFILE_VELOCITY = 112
MX_GOAL_POSITION = 116
MX_REALTIME_TICK = 120
MX_MOVING = 122
MX_MOVING_STATUS = 123
MX_PRESENT_PWM = 124
MX_PRESENT_CURRENT = 126
MX_PRESENT_VELOCITY = 128
MX_PRESENT_POSITION = 130
MX_VELOCITY_TRAJECTORY = 136
MX_POSITION_TRAJECTORY = 140
MX_PRESENT_INPUT_VOLTAGE = 144
MX_PRESENT_TEMPERATURE = 146
#INDIRECT ADDRESS RANGE - 2 BYTES EACH
MX_INDIRECT_ADDRESS_START = 168
MX_INDIRECT_ADDRESS_END = 222
#TODO add 2nd and 3rd indir data ranges

#INDIRECT DATA RANGE- 1 BYTE EACH
MX_INDIRECT_DATA_START = 224
MX_INDIRECT_DATA_END = 251
#TODO: add 2nd and 3rd indir data ranges


# Status Return Levels
DXL_RETURN_NONE = 0
DXL_RETURN_READ = 1
DXL_RETURN_ALL = 2

# Instruction Set
DXL_PING = 1
DXL_READ_DATA = 2
DXL_WRITE_DATA = 3
DXL_REG_WRITE = 4
DXL_ACTION = 5
DXL_RESET = 6
DXL_SYNC_WRITE = 131

# Broadcast Constant
DXL_BROADCAST = 254

# Error Codes
DXL_INSTRUCTION_ERROR = 64
DXL_OVERLOAD_ERROR = 32
DXL_CHECKSUM_ERROR = 16
DXL_RANGE_ERROR = 8
DXL_OVERHEATING_ERROR = 4
DXL_ANGLE_LIMIT_ERROR = 2
DXL_INPUT_VOLTAGE_ERROR = 1
DXL_NO_ERROR = 0

MX_MAX_SPEED_TICK = 1023                    # maximum speed in encoder units
MX_MAX_TORQUE_TICK = 1023                   # maximum torque in encoder units

MX_CURRENT_UNITS = 3.36                     # units of current register in mA

KGCM_TO_NM = 0.0980665                      # 1 kg-cm is that many N-m
RPM_TO_RADSEC = 0.104719755                 # 1 RPM is that many rad/sec

DXL_MODEL_TO_PARAMS = \
{

    311: { 'name':               'MX-64',
           'encoder_resolution': 4096,
           'range_degrees':      360.0,
           'torque_per_volt':    6.0 / 12.0,                       #  6 NM @ 12V
           'velocity_per_volt':  (63 * RPM_TO_RADSEC) / 12.0,      #  63 RPM @ 12.0V
           'rpm_per_tick':       0.114,
           'features':           []
         },
}
