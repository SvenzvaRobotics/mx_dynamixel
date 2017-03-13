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
Constants for the MX-2.0 protocol of the Dynamixel control table

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
MX_PRESENT_POSITION = 132
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
MX_RETURN_NONE = 0
MX_RETURN_READ = 1
MX_RETURN_ALL = 2

# Instruction Set
MX_PING = 1
MX_READ_DATA = 2
MX_WRITE_DATA = 3
MX_REG_WRITE = 4
MX_ACTION = 5
MX_RESET = 6
MX_SYNC_WRITE = 131

# Broadcast Constant
MX_BROADCAST = 254

# Error Codes
MX_ACCESS_ERROR = 64
MX_DATA_LIMIT_ERROR = 32
MX_LENGTH_ERROR = 16
MX_RANGE_ERROR = 8
MX_CHECKSUM_ERROR = 4
MX_INSTRUCTION_ERROR = 2
MX_RESULT_FAIL = 1
MX_NO_ERROR = 0

MX_MAX_SPEED_TICK = 1023                    # maximum speed in encoder units
MX_MAX_TORQUE_TICK = 1023                   # maximum torque in encoder units

MX_CURRENT_UNITS = 3.36                     # units of current register in mA

KGCM_TO_NM = 0.0980665                      # 1 kg-cm is that many N-m
RPM_TO_RADSEC = 0.104719755                 # 1 RPM is that many rad/sec

MX_MODEL_TO_PARAMS = \
{

    311: { 'name':               'MX-64',
           'encoder_resolution': 4096,
           'range_degrees':      360.0,
           'torque_per_volt':    6.0 / 12.0,                       #  6 NM @ 12V
           'velocity_per_volt':  (63 * RPM_TO_RADSEC) / 12.0,      #  63 RPM @ 12.0V
           'rpm_per_tick':       0.114,
           'features':           []
         },
    321: { 'name':               'MX-106',
           'encoder_resolution': 4096,
           'range_degrees':      360.0,
           'torque_per_volt':    8.4 / 12.0,                       #  8.4 NM @ 12V
           'velocity_per_volt':  (45 * RPM_TO_RADSEC) / 12.0,      #  45 RPM @ 12.0V
           'rpm_per_tick':       0.114,
           'features':           []
         },
}


MX_CRC_TABLE = [
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,

        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,

        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,

        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,

        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,

        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,

        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,

        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,

        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,

        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,

        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,

        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,

        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,

        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,

        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,

        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,

        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,

        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,

        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,

        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,

        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,

        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,

        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,

        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,

        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,

        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,

        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,

        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,

        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,

        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,

        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,

        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202]
