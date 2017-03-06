#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2017 Svenzva Robotics
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

import rospy
import time
import serial
import ctypes
from array import array
from binascii import b2a_hex
from threading import Lock

from mx_driver.dynamixel_const import *

exception = None

class DynamixelIO(object):
    """ Provides low level IO with the Dynamixel servos through pyserial. Has the
    ability to write instruction packets, request and read register value
    packets, send and receive a response to a ping packet, and send a SYNC WRITE
    multi-servo instruction packet.
    """

    def __init__(self, port, baudrate, readback_echo=False):
        """ Constructor takes serial port and baudrate as arguments. """
        try:
            self.serial_mutex = Lock()
            self.ser = None
            self.ser = serial.Serial(port, baudrate, timeout=0.015)
            self.port_name = port
            self.readback_echo = readback_echo
        except SerialOpenError:
           raise SerialOpenError(port, baudrate)

    def __del__(self):
        """ Destructor calls DynamixelIO.close """
        self.close()

    def close(self):
        """
        Be nice, close the serial port.
        """
        if self.ser:
            self.ser.flushInput()
            self.ser.flushOutput()
            self.ser.close()

    def __write_serial(self, data):
        self.ser.flushInput()
        self.ser.flushOutput()
        self.ser.write(data)
        if self.readback_echo:
            self.ser.read(len(data))

    def __read_response(self, servo_id):
        data = []

        try:
            data.extend(self.ser.read(7))
            if not data[0:3] == ['\xff', '\xff', '\xfd']:
                rospy.logerr("Motor packet prefix incorrect. Dropping...")
                raise Exception('Wrong packet prefix %s' % data[0:3])
            length = (ord(data[5])) + ((ord(data[6])) << 8)
            data.extend(self.ser.read(length))
            data = array('B', ''.join(data)).tolist() # [int(b2a_hex(byte), 16) for byte in data]
        except Exception, e:
            raise DroppedPacketError('Invalid response received from motor %d. %s' % (servo_id, e))

        # verify checksum: use full packet length; which is length + 5 (bytes for header and etc)
        checksum = self.update_crc(0, data, length + 5)
        CRC_L = checksum & 0x00FF
        CRC_H = (checksum >> 8) & 0x00FF

        #Here however, need to account for length of the 'length' bytes. Add 7
        full_length = length + 7
        if not (CRC_H == data[full_length-1] and CRC_L == data[full_length-2]):
            raise ChecksumError(servo_id, data, checksum)
        return data

    def read(self, servo_id, address, size):
        """ Read "size" bytes of data from servo with "servo_id" starting at the
        register with "address". "address" is an integer between 0 and 57. It is
        recommended to use the constants in module dynamixel_const for readability.

        To read the position from servo with id 1, the method should be called
        like:
            read(1, DXL_GOAL_POSITION_L, 2)
        """
        length = 7  # instruction, address, size, checksum

        length_l = length & 0xff
        length_h = (length>>8) & 0xff

        addr_l = address & 0xff
        addr_h = (address>>8) & 0xff

        size_l = size & 0xff
        size_h = (size>>8) & 0xff

        packet = [0xFF, 0xFF, 0xFD, 0x00, servo_id, length_l, length_h, DXL_READ_DATA, addr_l, addr_h, size_l, size_h]
        checksum = self.update_crc(0,packet,len(packet))
        CRC_L = checksum & 0x00FF
        CRC_H = (checksum >> 8) & 0x00FF

        packet = [0xFF, 0xFF, 0xFD, 0x00, servo_id, length_l, length_h, DXL_READ_DATA, addr_l, addr_h, size_l, size_h, CRC_L, CRC_H]

        packetStr = array('B', packet).tostring() # same as: packetStr = ''.join([chr(byte) for byte in packet])

        with self.serial_mutex:
            self.__write_serial(packetStr)

            # wait for response packet from the motor
            timestamp = time.time()
            time.sleep(0.0013)

            # read response
            data = self.__read_response(servo_id)
            data.append(timestamp)

        return data

    def write(self, servo_id, address, data):
        """ Write the values from the "data" list to the servo with "servo_id"
        starting with data[0] at "address", continuing through data[n-1] at
        "address" + (n-1), where n = len(data). "address" is an integer between
        0 and 49. It is recommended to use the constants in module dynamixel_const
        for readability. "data" is a list/tuple of integers.

        To set servo with id 1 to position 276, the method should be called
        like:
            write(1, DXL_GOAL_POSITION_L, (20, 1))
        """
        # Number of bytes following standard header (0xFF, 0xFF, id, length)
        length = 3 + len(data)  # instruction, address, len(data), checksum

        # directly from AX-12 manual:
        # Check Sum = ~ (ID + LENGTH + INSTRUCTION + PARAM_1 + ... + PARAM_N)
        # If the calculated value is > 255, the lower byte is the check sum.
        checksum = 255 - ((servo_id + length + DXL_WRITE_DATA + address + sum(data)) % 256)

        # packet: FF  FF  ID LENGTH INSTRUCTION PARAM_1 ... CHECKSUM
        packet = [0xFF, 0xFF, servo_id, length, DXL_WRITE_DATA, address]
        packet.extend(data)
        packet.append(checksum)

        packetStr = array('B', packet).tostring() # packetStr = ''.join([chr(byte) for byte in packet])

        with self.serial_mutex:
            self.__write_serial(packetStr)

            # wait for response packet from the motor
            timestamp = time.time()
            time.sleep(0.0013)

            # read response
            data = self.__read_response(servo_id)
            data.append(timestamp)

        return data

    def sync_write(self, address, data):
        """ Use Broadcast message to send multiple servos instructions at the
        same time. No "status packet" will be returned from any servos.
        "address" is an integer between 0 and 49. It is recommended to use the
        constants in module dynamixel_const for readability. "data" is a tuple of
        tuples. Each tuple in "data" must contain the servo id followed by the
        data that should be written from the starting address. The amount of
        data can be as long as needed.

        To set servo with id 1 to position 276 and servo with id 2 to position
        550, the method should be called like:
            sync_write(DXL_GOAL_POSITION_L, ( (1, 20, 1), (2 ,38, 2) ))
        """
        # Calculate length and sum of all data
        flattened = [value for servo in data for value in servo]

        # Number of bytes following standard header (0xFF, 0xFF, id, length) plus data
        length = 4 + len(flattened)

        checksum = 255 - ((DXL_BROADCAST + length + \
                          DXL_SYNC_WRITE + address + len(data[0][1:]) + \
                          sum(flattened)) % 256)

        # packet: FF  FF  ID LENGTH INSTRUCTION PARAM_1 ... CHECKSUM
        packet = [0xFF, 0xFF, DXL_BROADCAST, length, DXL_SYNC_WRITE, address, len(data[0][1:])]
        packet.extend(flattened)
        packet.append(checksum)

        packetStr = array('B', packet).tostring() # packetStr = ''.join([chr(byte) for byte in packet])

        with self.serial_mutex:
            self.__write_serial(packetStr)


    """
    CRC checksum calculation taken from the specifications of Dynamixel 2.0 protocol from
    Robotis support docs. Converted to python.
    Returns an unsigned short that represents the full crc.
    """
    def update_crc(self, crc_accum, data_blk_ptr, data_blk_size):
        #crc table; 256 of unsigned shorts
        crc_table = [
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


        for j in range(0, data_blk_size):
            i = ctypes.c_ushort((crc_accum >> 8) ^ data_blk_ptr[j]).value & 0xFF;
            crc_accum = (crc_accum << 8) ^ crc_table[i];

        return ctypes.c_ushort(crc_accum).value;

    def ping(self, servo_id):
        """ Ping the servo with "servo_id". This causes the servo to return a
        "status packet". This can tell us if the servo is attached and powered,
        and if so, if there are any errors.
        """
        # Number of bytes following standard header (0xFF, 0xFF, id, length)
        length = 3  # instruction, #params, checksum_1, checksum_2

        length_l = length & 0xFF
        length_h = (length>>8) & 0xFF

        """
        Get checksum:
             crc_accum : set as ‘0’
             data_blk_ptr : Packet array pointer
             data_blk_size : number of bytes in the Packet excluding the CRC
             data_blk_size = Header(3) + Reserved(1) + Packet ID(1) + Packet Length(2) + Packet Length – CRC(2)

                           = 3+1+1+2+Packet Length-2 = 5 + Packet Length;

             Packet Length = (LEN_H << 8 ) + LEN_L;  //Little-endian
        """

        packet = [0xFF, 0xFF, 0xFD, 0x00, servo_id, length_l, length_h, DXL_PING]
        checksum = self.update_crc(0,packet,len(packet))
        CRC_L = checksum & 0x00FF
        CRC_H = (checksum >> 8) & 0x00FF

        packet = [0xFF, 0xFF, 0xFD, 0x00, servo_id, length_l, length_h, DXL_PING, CRC_L, CRC_H]
        packetStr = array('B', packet).tostring()
        with self.serial_mutex:
            self.__write_serial(packetStr)

            # wait for response packet from the motor
            timestamp = time.time()
            time.sleep(0.0013)

            try:
                response = self.__read_response(servo_id)
                response.append(timestamp)
            except Exception, e:
                response = []
        if response:
            self.exception_on_error(response[4], servo_id, 'ping')

        return response

    def test_bit(self, number, offset):
        mask = 1 << offset
        return (number & mask)


    ######################################################################
    # These function modify EEPROM data which persists after power cycle #
    ######################################################################

    def set_id(self, old_id, new_id):
        """
        Sets a new unique number to identify a motor. The range from 1 to 253
        (0xFD) can be used.
        """
        response = self.write(old_id, MX_ID, [new_id])
        if response:
            self.exception_on_error(response[4], old_id, 'setting id to %d' % new_id)
        return response

    def set_baud_rate(self, servo_id, baud_rate):
        """
        Sets servo communication speed. The range from 0 to 254.
        """
        response = self.write(servo_id, MX_BAUD_RATE, [baud_rate])
        if response:
            self.exception_on_error(response[4], servo_id, 'setting baud rate to %d' % baud_rate)
        return response

    def set_return_delay_time(self, servo_id, delay):
        """
        Sets the delay time from the transmission of Instruction Packet until
        the return of Status Packet. 0 to 254 (0xFE) can be used, and the delay
        time per data value is 2 usec.
        """
        response = self.write(servo_id, MX_RETURN_DELAY_TIME, [delay])
        if response:
            self.exception_on_error(response[4], servo_id, 'setting return delay time to %d' % delay)
        return response

    def set_position_limit_max(self, servo_id, max_angle):
        """
        Set the max angle of rotation limit.
        """
        #TODO: remove int casting!!
        loVal = int(max_angle % 256)
        hiVal = int(max_angle >> 8)

        response = self.write(servo_id, MX_MAX_POSITION_LIMIT, (loVal, hiVal))
        if response:
            self.exception_on_error(response[4], servo_id, 'setting max position limit to %d' % angle_ccw)
        return response

    def set_position_limit_min(self, servo_id, min_angle):
        """
        Set the min angle of rotation limit.
        """
        #TODO: remove int casting!!
        loVal = int(min_angle % 256)
        hiVal = int(min_angle >> 8)

        response = self.write(servo_id, MX_MIN_POSITION_LIMIT, (loVal, hiVal))
        if response:
            self.exception_on_error(response[4], servo_id, 'setting min position limit to %d' % angle_ccw)
        return response

    def set_drive_mode(self, servo_id, mode):
        """
        Sets the drive mode
        """
        response = self.write(servo_id, MX_DRIVE_MODE, [mode])
        if response:
            self.exception_on_error(response[4], servo_id, 'setting drive mode to %d' % drive_mode)
        return response

    def set_voltage_limit_min(self, servo_id, min_voltage):
        """
        Set the minimum voltage limit.
        NOTE: the absolute min is 5v
        """

        if min_voltage < 5: min_voltage = 5
        minVal = int(min_voltage * 10)

        response = self.write(servo_id, MX_MIN_VOLTAGE_LIMIT, [minVal])
        if response:
            self.exception_on_error(response[4], servo_id, 'setting minimum voltage level to %d' % min_voltage)
        return response

    def set_voltage_limit_max(self, servo_id, max_voltage):
        """
        Set the maximum voltage limit.
        NOTE: the absolute max is 25v
        """

        if max_voltage > 25: max_voltage = 25
        maxVal = int(max_voltage * 10)

        response = self.write(servo_id, MX_MAX_VOLTAGE_LIMIT, [maxVal])
        if response:
            self.exception_on_error(response[4], servo_id, 'setting maximum voltage level to %d' % max_voltage)
        return response

    def set_voltage_limits(self, servo_id, min_voltage, max_voltage):
        """
        Set the min and max voltage limits.
        NOTE: the absolute min is 5v and the absolute max is 25v
        """

        if min_voltage < 5: min_voltage = 5
        if max_voltage > 25: max_voltage = 25

        minVal = int(min_voltage * 10)
        maxVal = int(max_voltage * 10)

        response = self.write(servo_id, MX_MIN_VOLTAGE_LIMIT, (minVal, maxVal))
        if response:
            self.exception_on_error(response[4], servo_id, 'setting min and max voltage levels to %d and %d' %(min_voltage, max_voltage))
        return response


    #TODO CHECK IF TORQUE IS ENABLED FOR ALL EEPROM SETS!!


    ###############################################################
    # These functions can send a single command to a single servo #
    ###############################################################

    def set_torque_enabled(self, servo_id, enabled):
        """
        Sets the value of the torque enabled register to 1 or 0. When the
        torque is disabled the servo can be moved manually while the motor is
        still powered.
        """
        response = self.write(servo_id, MX_TORQUE_ENABLE, [enabled])
        if response:
            self.exception_on_error(response[4], servo_id, '%sabling torque' % 'en' if enabled else 'dis')
        return response

    def set_position_d_gain(self, servo_id, d_gain):
        """
        Sets the value of derivative action of PID controller.
        Gain value is in range 0 to 1024.
        """
        #TODO check if valid range
        response = self.write(servo_id, MX_POSITION_D_GAIN, [d_gain])
        self.exception_on_error(response[4], servo_id, 'setting D gain value of PID controller to %d' % d_gain)
        return response

    #TODO SET_POSITION_I

    #TODO SET_POSITION_P

    #TODO SET_VELOCITY_P

    #TODO SET_VELOCITY_I

    #TODO SET_FEEDFORWARD 1

    #TODO SET_FEEDFORWARD 2


    def set_acceleration_profile(self, servo_id, acceleration):
        """
        Sets acceleration profile. Units: 214.577[Rev/min2]
        0 - inifinite acceleration
        Range: 0 - val(MX_ACCELERATION_LIMIT)
        """

        response = self.write(servo_id, MX_ACCLERATION_PROFILE, (acceleration, ))
        if response:
            self.exception_on_error(response[4], servo_id, 'setting acceleration profile to %d' % acceleration)
        return response


    def set_homing_offset(self, servo_id, offset):
        """
        Set the servo with servo_id to the specified multiturn offset.
        Valid offset values depend on mode, ie joint mode -> [-1024, 1024]
        while multi-turn -> [-24576 to 24576]
        """
        #TODO remove int casting
        loVal = int(offset % 256)
        hiVal = int(offset >> 8)

        response = self.write(servo_id, MX_HOMING_OFFSET, (loVal, hiVal))
        if response:
            self.exception_on_error(response[4], servo_id, 'setting multiturn offset to %d' % offset)
        return response


    def set_position(self, servo_id, position):
        """
        position control range: 0 ~ 4098
        extended position range: -1,048,575 ~ 1,048,575

        implemented range: -24576 ~ 24576
        """
        #TODO get all bytes here
        loVal = ctypes.c_ubyte(position & 0xff).value
        hiVal = ctypes.c_ubyte(position >> 8).value

        response = self.write(servo_id, MX_GOAL_POSITION, (loVal, hiVal, 0, 0))
        if response:
            self.exception_on_error(response[4], servo_id, 'setting goal position to %d' % position)
        return response

    def set_speed(self, servo_id, speed):
        """
        Only used for velocity control mode.
        Valid ranges 0 - val(MX_MAX_VELOCITY_LIMIT)
        """
        #TODO instruction is now a 4byte command. Fix!
        # split speed into 2 bytes
        if speed >= 0:
            loVal = int(speed % 256)
            hiVal = int(speed >> 8)
        else:
            loVal = int((1023 - speed) % 256)
            hiVal = int((1023 - speed) >> 8)

        response = self.write(servo_id, MX_GOAL_VELOCITY, (loVal, hiVal))
        if response:
            self.exception_on_error(response[4], servo_id, 'setting moving speed to %d' % speed)
        return response

    def set_current_limit(self, servo_id, current):
        """
        Sets the value of the maximum current limit, where current is directly analagous to torque.
        Limiting current limits the maximim torque.

        unit: about  3.36[mA]
        range: 0 - 1,941
        """
        #TODO: use ctypes casting here
        loVal = int(current % 256)
        hiVal = int(current >> 8)

        response = self.write(servo_id, MX_CURRENT_LIMIT, (loVal, hiVal))
        if response:
            self.exception_on_error(response[4], servo_id, 'setting torque limit to %d' % torque)
        return response

    def set_goal_current(self, servo_id, current):
        """
        Fills in the goal current register.
        Only applies if in the Current-based position control mode.
        Valid range: 0 - val(MX_CURRENT_LIMIT)
        """

        #TODO: fix current mode
        loVal = int(torque % 256);
        hiVal = int(torque >> 8);
        response = self.write(servo_id, DXL_GOAL_TORQUE_L, (loVal, hiVal))
        if response:
            self.exception_on_error(response[4], servo_id, 'setting goal torque to %d' % torque)
        return response


    def set_position_and_speed(self, servo_id, position, speed):
        """
        Set the servo with servo_id to specified position and speed.

        Note that this method is depreciated; under 2.0 protocol, velocity is only set directly
        (when in velocity control mode) or indirectly through velocity profile (in all other
        control modes)
        """
        loPositionVal = ctypes.c_ubyte(position & 0xff).value
        hiPositionVal = ctypes.c_ubyte(position >> 8).value

        response = self.write(servo_id, MX_GOAL_POSITION, (loPositionVal, hiPositionVal, ))
        if response:
            self.exception_on_error(response[4], servo_id, 'setting goal position to %d and moving speed to %d' %(position, speed))
        return response

    def set_led(self, servo_id, led_state):
        """
        Turn the LED of servo motor on/off.
        Possible boolean state values:
            True - turn the LED on,
            False - turn the LED off.
        """
        response = self.write(servo_id, MX_LED, [led_state])
        if response:
            self.exception_on_error(response[4], servo_id,
                    'setting a LED to %s' % led_state)
        return response


    #################################################################
    # These functions can send multiple commands to multiple servos #
    # These commands are used in ROS wrapper as they don't send a   #
    # response packet, ROS wrapper gets motor states at a set rate  #
    #################################################################

    def set_multi_torque_enabled(self, valueTuples):
        """
        Method to set multiple servos torque enabled.
        Should be called as such:
        set_multi_servos_to_torque_enabled( (id1, True), (id2, True), (id3, True) )
        """
        self.sync_write(DXL_TORQUE_ENABLE, tuple(valueTuples))

    def set_compliance_d(self, valueTuple):
        """
        Sets the derivative value for MX series servos for a single id.
        valueTuple should be a tuple of the form (idx, value) where value is [0,254] inclusive
        """
        self.sync_write(DXL_D_GAIN, tuple(valueTuple))


    def set_multi_position(self, valueTuples):
        """
        Set different positions for multiple servos.
        Should be called as such:
        set_multi_position( ( (id1, position1), (id2, position2), (id3, position3) ) )
        """
        # prepare value tuples for call to syncwrite
        writeableVals = []

        for vals in valueTuples:
            sid = vals[0]
            position = vals[1]
            #TODO: ensure this 2s compliment conversion holds for non multi-turn motors
            loVal = ctypes.c_ubyte(position & 0x00FF).value
            hiVal =  ctypes.c_ubyte(position >> 8).value
            writeableVals.append( (sid, loVal, hiVal) )

        # use sync write to broadcast multi servo message
        self.sync_write(MX_GOAL_POSITION, writeableVals)

    def set_multi_speed(self, valueTuples):
        """
        Set different speeds for multiple servos.
        Should be called as such:
        set_multi_speed( ( (id1, speed1), (id2, speed2), (id3, speed3) ) )
        """

        rospy.logerr("Function not implemented.")
        return
        # prepare value tuples for call to syncwrite
        writeableVals = []

        for vals in valueTuples:
            sid = vals[0]
            speed = vals[1]

            # split speed into 2 bytes
            if speed >= 0:
                loVal = int(speed % 256)
                hiVal = int(speed >> 8)
            else:
                loVal = int((1023 - speed) % 256)
                hiVal = int((1023 - speed) >> 8)

            writeableVals.append( (sid, loVal, hiVal) )

        # use sync write to broadcast multi servo message
        self.sync_write(DXL_GOAL_SPEED_L, writeableVals)

    def set_multi_position_and_speed(self, valueTuples):
        """
        Set different positions and speeds for multiple servos.
        Should be called as such:
        set_multi_position_and_speed( ( (id1, position1, speed1), (id2, position2, speed2), (id3, position3, speed3) ) )
        """

        rospy.logerr("Function not implemented.")
        return
        # prepare value tuples for call to syncwrite
        writeableVals = []

        for vals in valueTuples:
            sid = vals[0]
            position = vals[1]
            speed = vals[2]


            #gear ratio
            if sid == 4 or sid == 5:
                speed = speed * 4
                position = position * 4
            elif sid == 2 or sid == 3:
                speed = speed * 6
                position = position * 6

            # split speed into 2 bytes
            if speed >= 0:
                loSpeedVal = int(speed % 256)
                hiSpeedVal = int(speed >> 8)
            else:
                loSpeedVal = int((1023 - speed) % 256)
                hiSpeedVal = int((1023 - speed) >> 8)

            # split position into 2 bytes
            loPositionVal = ctypes.c_ubyte(position & 0xff).value
            hiPositionVal = ctypes.c_ubyte(position >> 8).value
            writeableVals.append( (sid, loPositionVal, hiPositionVal, loSpeedVal, hiSpeedVal) )

        # use sync write to broadcast multi servo message
        self.sync_write(DXL_GOAL_POSITION_L, tuple(writeableVals))


    #################################
    # Servo status access functions #
    #################################

    def get_model_number(self, servo_id):
        response = self.read(servo_id, MX_MODEL_NUMBER, 2)
        if response:
            self.exception_on_error(response[8], servo_id, 'fetching model number')
        return response[9] + (response[10] << 8)

    def get_firmware_version(self, servo_id):
        """ Reads the servo's firmware version. """
        response = self.read(servo_id, MX_VERSION, 1)
        if response:
            self.exception_on_error(response[8], servo_id, 'fetching firmware version')
        return response[9]

    def get_return_delay_time(self, servo_id):
        """ Reads the servo's return delay time. """
        response = self.read(servo_id, MX_RETURN_DELAY_TIME, 1)
        if response:
            self.exception_on_error(response[8], servo_id, 'fetching return delay time')
        return response[9]

    def get_angle_limits(self, servo_id):
        """
        Returns the min and max angle limits from the specified servo.
        """
        response = self.read(servo_id, MX_MIN_POSITION_LIMIT, 4)
        response_m = self.read(servo_id, MX_MAX_POSITION_LIMIT, 4)
        if response or response_m:
            self.exception_on_error(response[8], servo_id, 'fetching position limits')
            self.exception_on_error(response_m[8], servo_id, 'fetching position limits')
        # extract data valus from the raw data
        cwLimit = response[9] + (response[10] << 8) + (response[11] << 16) + (response[12] << 24)
        ccwLimit = response_m[9] + (response_m[10] << 8) + (response_m[11] << 16) + (response_m[12] << 24)

        # return the data in a dictionary
        return {'min':cwLimit, 'max':ccwLimit}

    def get_drive_mode(self, servo_id):
        """ Reads the servo's drive mode. """
        response = self.read(servo_id, MX_DRIVE_MODE, 1)
        if response:
            self.exception_on_error(response[4], servo_id, 'fetching drive mode')
        return response[5]

    def get_voltage_limits(self, servo_id):
        """
        Returns the min and max voltage limits from the specified servo.
        """
        response_min = self.read(servo_id, MX_MIN_VOLTAGE_LIMIT, 2)
        response_max = self.read(servo_id, MX_MAX_VOLTAGE_LIMIT, 2)

        if response_min or response_max:
            self.exception_on_error(response_min[8], servo_id, 'fetching voltage limits')
            self.exception_on_error(response_max[8], servo_id, 'fetching voltage limits')
        # extract data valus from the raw data
        min_voltage = response_min[9] + (response_min[10] << 8) / 10.0
        max_voltage = response_max[9] + (response_max[10] << 8) / 10.0

        # return the data in a dictionary
        return {'min':min_voltage, 'max':max_voltage}

    def get_position(self, servo_id):
        """ Reads the servo's position value from its registers. """
        response = self.read(servo_id, MX_PRESENT_POSITION, 6)
        #TODO: accomodate the 6byte position vs the word sized pos as below
        if response:
            self.exception_on_error(response[4], servo_id, 'fetching present position')
        position = response[5] + (response[6] << 8)
        return ctypes.c_short(position)

    def get_speed(self, servo_id):
        """ Reads the servo's speed value from its registers. """
        response = self.read(servo_id, MX_PRESENT_VELOCITY, 6)
        #TODO accomodate 6byte velocity. also verify the maths below
        if response:
            self.exception_on_error(response[4], servo_id, 'fetching present speed')
        speed = response[5] + (response[6] << 8)
        if speed > 1023:
            return 1023 - speed
        return speed

    def get_torque_limit(self, servo_id):
        """ Reads the servo's speed value from its registers. """
        response = self.read(servo_id, DXL_TORQUE_LIMIT_L, 2)
        if response:
            self.exception_on_error(response[4], servo_id, 'fetching torque limit')
        torque = response[5] + (response[6] << 8)
        return torque

    def get_voltage(self, servo_id):
        """ Reads the servo's voltage. """
        response = self.read(servo_id, MX_PRESENT_INPUT_VOLTAGE, 2)
        if response:
            self.exception_on_error(response[8], servo_id, 'fetching supplied voltage')
        return (response[9] + (response[10] << 8)) / 10.0

    def get_current(self, servo_id):
        """ Reads the servo's current consumption (if supported by model) """
        model = self.get_model_number(servo_id)
        if not model in DXL_MODEL_TO_PARAMS:
            raise UnsupportedFeatureError(model, DXL_CURRENT_L)

        if DXL_CURRENT_L in DXL_MODEL_TO_PARAMS[model]['features']:
            response = self.read(servo_id, DXL_CURRENT_L, 2)
            if response:
                self.exception_on_error(response[4], servo_id, 'fetching sensed current')
            current = response[5] + (response[6] << 8)
            return 0.0045 * (current - 2048)

        if DXL_SENSED_CURRENT_L in DXL_MODEL_TO_PARAMS[model]['features']:
            response = self.read(servo_id, DXL_SENSED_CURRENT_L, 2)
            if response:
                self.exception_on_error(response[4], servo_id, 'fetching sensed current')
            current = response[5] + (response[6] << 8)
            return 0.01 * (current - 512)

        else:
            raise UnsupportedFeatureError(model, DXL_CURRENT_L)


    def get_feedback(self, servo_id):
        """
        Returns the id, goal, position, error, speed, load, voltage, temperature
        and moving values from the specified servo.
        """
        #TODO:
        # add temperature, true voltage (not pwm), and convert sensed current to torque
        response = self.read(servo_id, MX_GOAL_POSITION, 20)

        if response:
            self.exception_on_error(response[8], servo_id, 'fetching full servo status')
        # extract data values from the raw data
        goal = response[9] + (response[10] << 8) + (response[11] << 16) + (response[12] << 24)
        position = response[25] + (response[26] << 8) + (response[27] << 16) + (response[28] << 24)
        error = position - goal
        speed = response[21] + ( response[22] << 8) + (response[23] << 16) + (response[24] << 24)
        pwm = response[17] + (response[18] << 8)
        load_raw = ctypes.c_int16((response[19] + (response[20] << 8))).value
        moving = response[15]
        timestamp = response[-1]

        # return the data in a dictionary
        return { 'timestamp': timestamp,
                 'id': servo_id,
                 'goal': ctypes.c_int(goal).value,
                 'position': ctypes.c_int(position).value,
                 'error': ctypes.c_int(error).value,
                 'speed': ctypes.c_int(speed).value,
                 'load': load_raw,
                 'voltage': pwm,
                 'moving': bool(moving) }

    def get_led(self, servo_id):
        """
        Get status of the LED. Boolean return values:
            True - LED is on,
            False - LED is off.
        """
        response = self.read(servo_id, DXL_LED, 1)
        if response:
            self.exception_on_error(response[4], servo_id,
                'fetching LED status')

        return bool(response[5])


    def exception_on_error(self, error_code, servo_id, command_failed):
        global exception
        exception = None
        ex_message = '[servo #%d on %s@%sbps]: %s failed' % (servo_id, self.ser.port, self.ser.baudrate, command_failed)

        if not isinstance(error_code, int):
            msg = 'Communcation Error ' + ex_message
            exception = NonfatalErrorCodeError(msg, 0)
            return
        if not error_code & DXL_OVERHEATING_ERROR == 0:
            msg = 'Overheating Error ' + ex_message
            exception = FatalErrorCodeError(msg, error_code)
        if not error_code & DXL_OVERLOAD_ERROR == 0:
            msg = 'Overload Error ' + ex_message
            exception = FatalErrorCodeError(msg, error_code)
        if not error_code & DXL_INPUT_VOLTAGE_ERROR == 0:
            msg = 'Input Voltage Error ' + ex_message
            exception = NonfatalErrorCodeError(msg, error_code)
        if not error_code & DXL_ANGLE_LIMIT_ERROR == 0:
            msg = 'Angle Limit Error ' + ex_message
            exception = NonfatalErrorCodeError(msg, error_code)
        if not error_code & DXL_RANGE_ERROR == 0:
            msg = 'Range Error ' + ex_message
            exception = NonfatalErrorCodeError(msg, error_code)
        if not error_code & DXL_CHECKSUM_ERROR == 0:
            msg = 'Checksum Error ' + ex_message
            exception = NonfatalErrorCodeError(msg, error_code)
        if not error_code & DXL_INSTRUCTION_ERROR == 0:
            msg = 'Instruction Error ' + ex_message
            exception = NonfatalErrorCodeError(msg, error_code)

class SerialOpenError(Exception):
    def __init__(self, port, baud):
        Exception.__init__(self)
        self.message = "Cannot open port '%s' at %d bps" %(port, baud)
        self.port = port
        self.baud = baud
    def __str__(self):
        return self.message

class ChecksumError(Exception):
    def __init__(self, servo_id, response, checksum):
        Exception.__init__(self)
        self.message = 'Checksum received from motor %d does not match the expected one (%d != %d)' \
                       %(servo_id, response[-1], checksum)
        self.response_data = response
        self.expected_checksum = checksum
    def __str__(self):
        return self.message

class FatalErrorCodeError(Exception):
    def __init__(self, message, ec_const):
        Exception.__init__(self)
        self.message = message
        self.error_code = ec_const
    def __str__(self):
        return self.message

class NonfatalErrorCodeError(Exception):
    def __init__(self, message, ec_const):
        Exception.__init__(self)
        self.message = message
        self.error_code = ec_const
    def __str__(self):
        return self.message

class ErrorCodeError(Exception):
    def __init__(self, message, ec_const):
        Exception.__init__(self)
        self.message = message
        self.error_code = ec_const
    def __str__(self):
        return self.message

class DroppedPacketError(Exception):
    def __init__(self, message):
        Exception.__init__(self)
        self.message = message
    def __str__(self):
        return self.message

class UnsupportedFeatureError(Exception):
    def __init__(self, model_id, feature_id):
        Exception.__init__(self)
        if model_id in DXL_MODEL_TO_PARAMS:
            model = DXL_MODEL_TO_PARAMS[model_id]['name']
        else:
            model = 'Unknown'
        self.message = "Feature %d not supported by model %d (%s)" %(feature_id, model_id, model)
    def __str__(self):
        return self.message

