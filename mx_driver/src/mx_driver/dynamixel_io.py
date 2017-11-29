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
            self.baudrate = baudrate
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

        if len(data) != length + 7:
                raise DroppedPacketError('packet responded with error. dropping')

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
        register with "address".

        To read the position from servo with id 1, the method should be called
        like:
            read(1, MX_GOAL_POSITION, 2)
        """
        length = 7  # instruction, address, size, checksum

        length_l = length & 0xff
        length_h = (length>>8) & 0xff

        addr_l = address & 0xff
        addr_h = (address>>8) & 0xff

        size_l = size & 0xff
        size_h = (size>>8) & 0xff

        packet = [0xFF, 0xFF, 0xFD, 0x00, servo_id, length_l, length_h, MX_READ_DATA, addr_l, addr_h, size_l, size_h]
        checksum = self.update_crc(0,packet,len(packet))
        CRC_L = checksum & 0x00FF
        CRC_H = (checksum >> 8) & 0x00FF
        packet.append(CRC_L)
        packet.append(CRC_H)

        packetStr = array('B', packet).tostring()

        with self.serial_mutex:
            self.__write_serial(packetStr)

            # wait for response packet from the motor
            timestamp = time.time()
            #time.sleep(0.001)

            # read response
            data = self.__read_response(servo_id)
            data.append(timestamp)

        return data

    def write(self, servo_id, address, data):
        """ Write the values from the "data" list to the servo with "servo_id"
        starting with data[0] at "address", continuing through data[n-1] at
        "address" + (n-1), where n = len(data).
        "address" has a 2byte representation
        "data" is a list/tuple of integers.

        To set servo with id 1 to position 276, the method should be called
        eg:
            write(1, MX_GOAL_POSITION, (0,0,2, 0))
        """
        # Number of bytes following standard header (0xFF, 0xFF, id, length)
        length = 5 + len(data)  # instruction, address l+h, len(data), checksum l+h

        length_l = length & 0xff
        length_h = (length>>8) & 0xff

        addr_l = address & 0xff
        addr_h = (address>>8) & 0xff

        packet = [0xFF, 0xFF, 0xFD, 0x00, servo_id, length_l, length_h, MX_WRITE_DATA, addr_l, addr_h]
        packet.extend(data)
        checksum = self.update_crc(0,packet,len(packet))
        CRC_L = checksum & 0x00FF
        CRC_H = (checksum >> 8) & 0x00FF
        packet.append(CRC_L)
        packet.append(CRC_H)

        packetStr = array('B', packet).tostring() # packetStr = ''.join([chr(byte) for byte in packet])

        with self.serial_mutex:
            self.__write_serial(packetStr)

            # wait for response packet from the motor
            timestamp = time.time()
            #time.sleep(0.005)

            # read response
            data = self.__read_response(servo_id)
            data.append(timestamp)

        return data

    def sync_read(self, address, data_length, ids):
        """ Use Broadcast message to query multiple servos instructions at the
        same time.


        To read servos of id 1,2,3 at their current position, sync_read should be
        called as:
            sync_read(MX_PRESENT_POSITION, 4, (1,2,3))

        """
        flattened = ids #[value in ids]
        # Number of bytes following standard header ( instr, addr ) plus data
        length = 7 + len(flattened)
        length_l = length & 0xff
        length_h = (length>>8) & 0xff

        addr_l = address & 0xff
        addr_h = (address>>8) & 0xff

        data_l = data_length & 0xff
        data_h = (data_length>>8) & 0xff

        packet = [0xFF, 0xFF, 0xFD, 0x00, MX_BROADCAST, length_l, length_h, MX_SYNC_READ, addr_l, addr_h, data_l, data_h]
        packet.extend(flattened)
        checksum = self.update_crc(0,packet,len(packet))
        CRC_L = checksum & 0x00FF
        CRC_H = (checksum >> 8) & 0x00FF
        packet.append(CRC_L)
        packet.append(CRC_H)
        packetStr = array('B', packet).tostring()
        with self.serial_mutex:
            self.__write_serial(packetStr)

            # wait for response packet from the motor
            #timestamp = time.time()
            #time.sleep( float((length * 8.0 / int(self.baudrate) ) ))

            # read response
            status = []

            rx_length = 14
            for servo_id in ids:
                data = self.__read_response(servo_id)
                status.append(data)
                rx_length = data[5] + data[6] << 8
            #data.append(timestamp)

            for rx in status:
                if rx[5] + rx[6] << 8 != rx_length:
                    rospy.logdebug("Status returned with non-uniform packet. Dropping.")
                    #raise DroppedPacketError("Sync status read returned with non uniform packets.")

        return status



    def sync_write(self, address, data):
        """ Use Broadcast message to send multiple servos instructions at the
        same time. No "status packet" will be returned from any servos.
        "data" is a tuple of
        tuples. Each tuple in "data" must contain the servo id followed by the
        data that should be written from the starting address. The amount of
        data can be as long as needed.

        To set servo with id 1 to position 276 and servo with id 2 to position
        550, the method should be called like:
            sync_write(MX_GOAL_POSITION, ( (1, 20, 1), (2 ,38, 2) ))
        """
        # Calculate length of data
        flattened = [value for servo in data for value in servo]
        # Get the length of the data on a per-ID basis
        data_length = len(data[0][1:])
        # Number of bytes following standard header ( instr, addr ) plus data
        length = 7 + len(flattened)
        length_l = length & 0xff
        length_h = (length>>8) & 0xff

        addr_l = address & 0xff
        addr_h = (address>>8) & 0xff

        data_l = data_length & 0xff
        data_h = (data_length>>8) & 0xff

        packet = [0xFF, 0xFF, 0xFD, 0x00, MX_BROADCAST, length_l, length_h, MX_SYNC_WRITE, addr_l, addr_h, data_l, data_h]
        packet.extend(flattened)
        checksum = self.update_crc(0,packet,len(packet))
        CRC_L = checksum & 0x00FF
        CRC_H = (checksum >> 8) & 0x00FF
        packet.append(CRC_L)
        packet.append(CRC_H)

        packetStr = array('B', packet).tostring()
        with self.serial_mutex:
            self.__write_serial(packetStr)
            #time.sleep( float((length * 2.0 / int(self.baudrate) ) ))

    """
    CRC checksum calculation taken from the specifications of Dynamixel 2.0 protocol from
    Robotis support docs. Converted to python.
    Returns an unsigned short that represents the full crc.
    """
    def update_crc(self, crc_accum, data_blk_ptr, data_blk_size):
        for j in range(0, data_blk_size):
            i = ctypes.c_ushort((crc_accum >> 8) ^ data_blk_ptr[j]).value & 0xFF;
            crc_accum = (crc_accum << 8) ^ MX_CRC_TABLE[i];

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

        packet = [0xFF, 0xFF, 0xFD, 0x00, servo_id, length_l, length_h, MX_PING]
        checksum = self.update_crc(0,packet,len(packet))
        CRC_L = checksum & 0x00FF
        CRC_H = (checksum >> 8) & 0x00FF

        packet = [0xFF, 0xFF, 0xFD, 0x00, servo_id, length_l, length_h, MX_PING, CRC_L, CRC_H]
        packetStr = array('B', packet).tostring()
        with self.serial_mutex:
            self.__write_serial(packetStr)

            # wait for response packet from the motor
            timestamp = time.time()
            #time.sleep(0.00235)

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
            self.exception_on_error(response[8], old_id, 'setting id to %d' % new_id)
        return response

    def set_baud_rate(self, servo_id, baud_rate):
        """
        Sets servo communication speed. The range from 0 to 254.
        """
        response = self.write(servo_id, MX_BAUD_RATE, [baud_rate])
        if response:
            self.exception_on_error(response[8], servo_id, 'setting baud rate to %d' % baud_rate)
        return response

    def set_return_delay_time(self, servo_id, delay):
        """
        Sets the delay time from the transmission of Instruction Packet until
        the return of Status Packet. 0 to 254 (0xFE) can be used, and the delay
        time per data value is 2 usec.
        """
        response = self.write(servo_id, MX_RETURN_DELAY_TIME, [delay])
        if response:
            self.exception_on_error(response[8], servo_id, 'setting return delay time to %d' % delay)
        return response

    def set_position_limit_max(self, servo_id, max_angle):
        """
        Set the max angle of rotation limit.
        """
        b0 = max_angle & 0xff
        b1 = (max_angle >> 8) & 0xff
        b2 = (max_angle >> 16) & 0xff
        b3 = (max_angle >> 24) & 0xff

        response = self.write(servo_id, MX_MAX_POSITION_LIMIT, (b0, b1, b2, b3))
        if response:
            self.exception_on_error(response[8], servo_id, 'setting max position limit to %d' % max_angle)
        return response

    def set_moving_threshold(self, servo_id, val):
        """
        Set the speed of rotation limit to be considered 'moving' internally.
        """
        b0 = val & 0xff
        b1 = (val >> 8) & 0xff
        b2 = (val >> 16) & 0xff
        b3 = (val >> 24) & 0xff


        response = self.write(servo_id, MX_MOVING_THRESHOLD, (b0, b1, b2, b3))
        if response:
            self.exception_on_error(response[8], servo_id, 'setting moving limit to %d' % val)
        return response


    def set_position_limit_min(self, servo_id, min_angle):
        """
        Set the min angle of rotation limit.
        """
        b0 = min_angle & 0xff
        b1 = (min_angle >> 8) & 0xff
        b2 = (min_angle >> 16) & 0xff
        b3 = (min_angle >> 24) & 0xff


        response = self.write(servo_id, MX_MIN_POSITION_LIMIT, (b0, b1, b2, b3))
        if response:
            self.exception_on_error(response[8], servo_id, 'setting min position limit to %d' % min_angle)
        return response

    def set_drive_mode(self, servo_id, mode):
        """
        Sets the drive mode
        """
        response = self.write(servo_id, MX_DRIVE_MODE, [mode])
        if response:
            self.exception_on_error(response[8], servo_id, 'setting drive mode to %d' % drive_mode)
        return response

    def set_voltage_limit_min(self, servo_id, min_voltage):
        """
        Set the minimum voltage limit.
        NOTE: the absolute min is 9.5v
        """

        if min_voltage < 9.5: min_voltage = 9.5
        minVal = int(min_voltage * 10)

        response = self.write(servo_id, MX_MIN_VOLTAGE_LIMIT, (minVal,0))
        if response:
            self.exception_on_error(response[8], servo_id, 'setting minimum voltage level to %d' % min_voltage)
        return response

    def set_voltage_limit_max(self, servo_id, max_voltage):
        """
        Set the maximum voltage limit.
        NOTE: the absolute max is 16v
        """

        if max_voltage > 16: max_voltage = 16
        maxVal = int(max_voltage * 10)

        response = self.write(servo_id, MX_MAX_VOLTAGE_LIMIT, (maxVal,0))
        if response:
            self.exception_on_error(response[8], servo_id, 'setting maximum voltage level to %d' % max_voltage)
        return response

    def set_voltage_limits(self, servo_id, min_voltage, max_voltage):
        """
        Set the min and max voltage limits.
        NOTE: the absolute min is 5v and the absolute max is 25v
        """

        if min_voltage < 9.5: min_voltage = 9.5
        if max_voltage > 16: max_voltage = 16

        minVal = int(min_voltage * 10)
        maxVal = int(max_voltage * 10)

        response = self.write(servo_id, MX_MAX_VOLTAGE_LIMIT, (maxVal, 0, minVal, 0))
        if response:
            self.exception_on_error(response[8], servo_id, 'setting min and max voltage levels to %d and %d' %(min_voltage, max_voltage))
        return response


    #TODO CHECK IF TORQUE IS ENABLED FOR ALL EEPROM SETS!!


    ###############################################################
    # These functions can send a single command to a single servo #
    ###############################################################

    def set_torque_enabled(self, servo_id, enabled):
        """
        Sets the value of the torque enabled register to 1 or 0.
        Torque must be disabled to set any values in EEPROM memory area
        """
        response = self.write(servo_id, MX_TORQUE_ENABLE, (enabled,))
        if response:
            self.exception_on_error(response[8], servo_id, '%sabling torque' % 'en' if enabled else 'dis')
        return response

    def set_operation_mode(self, servo_id, mode):
        """
        Sets the operating mode {Torque, position control, velocity, etc)
        """
        response = self.write(servo_id, MX_OPERATING_MODE, (mode,))
        if response:
            self.exception_on_error(response[8], servo_id, 'setting operating mode')
        return response

    def set_velocity_i_gain(self, servo_id, i_gain):
        """
        Sets the value of integral action of PI velocity controller.
        Gain value is in range 0 to 2^15 - 1.
        """
        b0 = i_gain & 0xff
        b1 = (i_gain >> 8) & 0xff
        response = self.write(servo_id, MX_VELOCITY_I_GAIN, (b0, b1))
        if response:
            self.exception_on_error(response[8], servo_id, 'setting I gain value of velocity PI controller to %d' % i_gain)
        return response

    def set_velocity_p_gain(self, servo_id, p_gain):
        """
        Sets the value of proportional action of PI velocity controller.
        Gain value is in range 0 to 2^15 - 1.
        """
        b0 = p_gain & 0xff
        b1 = (p_gain >> 8) & 0xff
        response = self.write(servo_id, MX_VELOCITY_P_GAIN, (b0, b1))
        if response:
            self.exception_on_error(response[8], servo_id, 'setting P gain value of velocity PI controller to %d' % p_gain)
        return response

    def set_position_d_gain(self, servo_id, d_gain):
        """
        Sets the value of derivative action of PID controller.
        Gain value is in range 0 to 2^15 - 1.
        """
        b0 = d_gain & 0xff
        b1 = (d_gain >> 8) & 0xff
        response = self.write(servo_id, MX_POSITION_D_GAIN, (b0, b1))
        if response:
            self.exception_on_error(response[8], servo_id, 'setting D gain value of PID controller to %d' % d_gain)
        return response

    def set_position_i_gain(self, servo_id, i_gain):
        """
        Sets the value of integral action of PID controller.
        Gain value is in range 0 to 2^15 - 1.
        """
        b0 = i_gain & 0xff
        b1 = (i_gain >> 8) & 0xff
        response = self.write(servo_id, MX_POSITION_I_GAIN, (b0, b1))
        if response:
            self.exception_on_error(response[8], servo_id, 'setting I gain value of PID controller to %d' % i_gain)
        return response

    def set_position_p_gain(self, servo_id, p_gain):
        """
        Sets the value of proportional action of PID controller.
        Gain value is in range 0 to 2^15 - 1.
        """
        b0 = p_gain & 0xff
        b1 = (p_gain >> 8) & 0xff
        response = self.write(servo_id, MX_POSITION_P_GAIN, (b0, b1))
        if response:
            self.exception_on_error(response[8], servo_id, 'setting P gain value of PID controller to %d' % p_gain)
        return response

    def set_position_feedfwd1_gain(self, servo_id, gain):
        """
        Sets the value of 1st feedforward gain
        Gain value is in range 0 to 2^15 - 1.
        """
        b0 = gain & 0xff
        b1 = (gain >> 8) & 0xff
        response = self.write(servo_id, MX_FEEDFORWARD_1_GAIN, (b0, b1))
        if response:
            self.exception_on_error(response[8], servo_id, 'setting feedforward gain value of PID controller to %d' % gain)
        return response

    def set_position_feedfwd2_gain(self, servo_id, gain):
        """
        Sets the value of 2st feedforward gain
        Gain value is in range 0 to 2^15 - 1.
        """
        b0 = gain & 0xff
        b1 = (gain >> 8) & 0xff
        response = self.write(servo_id, MX_FEEDFORWARD_2_GAIN, (b0, b1))
        if response:
            self.exception_on_error(response[8], servo_id, 'setting feedforward gain value of PID controller to %d' % gain)
        return response

    def set_acceleration_profile(self, servo_id, acceleration):
        """
        Sets acceleration profile. Units: 214.577[Rev/min2]
        0 - inifinite acceleration
        Range: 0 - val(MX_ACCELERATION_LIMIT)
        """
        b0 = (acceleration) & 0xff
        b1 = (acceleration >> 8) & 0xff
        b2 = (acceleration >> 16) & 0xff
        b3 = (acceleration >> 24) & 0xff
        response = self.write(servo_id, MX_PROFILE_ACCELERATION, (b0,b1,b2,b3 ))
        if response:
            self.exception_on_error(response[8], servo_id, 'setting acceleration profile to %d' % acceleration)
        return response

    def set_velocity_profile(self, servo_id, velocity):
        """
        Sets acceleration profile. Units: .229 rpm
        0 - inifinite vel
        Range: 0 - val(MX_VELOCITY_LIMIT)
        """
        b0 = (velocity) & 0xff
        b1 = (velocity >> 8) & 0xff
        b2 = (velocity >> 16) & 0xff
        b3 = (velocity >> 24) & 0xff
        response = self.write(servo_id, MX_PROFILE_VELOCITY, (b0,b1,b2,b3 ))
        if response:
            self.exception_on_error(response[8], servo_id, 'setting velocity profile to %d' % velocity)
        return response


    def set_homing_offset(self, servo_id, offset):
        """
        Set the servo with servo_id to the specified multiturn offset.
        Valid offset values depend on mode, ie joint mode -> [-1024, 1024]
        while multi-turn -> [-24576 to 24576]
        """
        b0 = (offset) & 0xff
        b1 = (offset >> 8) & 0xff
        b2 = (offset >> 16) & 0xff
        b3 = (offset >> 24) & 0xff

        response = self.write(servo_id, MX_HOMING_OFFSET, (b0,b1,b2,b3))
        if response:
            self.exception_on_error(response[8], servo_id, 'setting multiturn offset to %d' % offset)
        return response


    def set_position(self, servo_id, position):
        """
        position control range: 0 ~ 4098
        extended position range: -1,048,575 ~ 1,048,575

        implemented range: -24576 ~ 24576
        """
        b0 = position & 0xff
        b1 = (position >> 8) & 0xff
        b2 = (position >> 16) & 0xff
        b3 = (position >> 24) & 0xff

        response = self.write(servo_id, MX_GOAL_POSITION, (b0,b1,b2,b3))
        if response:
            self.exception_on_error(response[8], servo_id, 'setting goal position to %d' % position)
        return response

    def set_speed(self, servo_id, speed):
        """
        Only used for velocity control mode.
        Valid ranges 0 - val(MX_MAX_VELOCITY_LIMIT)
        """
        b0 = speed & 0xff
        b1 = (speed >> 8) & 0xff
        b2 = (speed >> 16) & 0xff
        b3 = (speed >> 24) & 0xff


        response = self.write(servo_id, MX_GOAL_VELOCITY, (b0,b1,b2,b3))
        if response:
            self.exception_on_error(response[8], servo_id, 'setting moving speed to %d' % speed)
        return response

    def set_torque_goal(self, servo_id, current):
        """
        Only used for current control mode, or position+current control.
        This sets the current value directly, and not torque.
        To see equivalent torque value, check MX torque curves for the given motor
        Valid ranges 0 - val(MX_MAX_TORQUE_LIMIT)
        """
        b0 = current & 0xff
        b1 = (current >> 8) & 0xff
        response = self.write(servo_id, MX_GOAL_CURRENT, (b0,b1))
        if response:
            self.exception_on_error(response[8], servo_id, 'setting moving current (torque) to %d' % current)
        return response


    def set_current_limit(self, servo_id, current):
        """
        Sets the value of the maximum current limit, where current is directly analagous to torque.
        Limiting current limits the maximim torque.

        unit: about  3.36[mA]
        range: 0 - 1,941
        """
        b0 = (current & 0xff)
        b1 = (current >> 8) & 0xff

        response = self.write(servo_id, MX_CURRENT_LIMIT, (b0, b1))
        if response:
            self.exception_on_error(response[8], servo_id, 'setting torque limit to %d' % current)
        return response

    def set_goal_current(self, servo_id, current):
        """
        Fills in the goal current register.
        Only applies if in the Current-based position control mode.
        Valid range: 0 - val(MX_CURRENT_LIMIT)
        """
        b0 = current & 0xff
        b1 = (current >> 8) & 0xff

        response = self.write(servo_id, MX_GOAL_CURRENT, (b0,b1))
        if response:
            self.exception_on_error(response[8], servo_id, 'setting goal current to %d' % current)
        return response

    def set_goal_pwm(self, servo_id, pwm):
        """
        Fills in the goal pwm register.
        Valid range: 0 - val(MX_PWM_LIMIT)
        """
        b0 = pwm & 0xff
        b1 = (pwm >> 8) & 0xff

        response = self.write(servo_id, MX_GOAL_PWM, (b0,b1))
        if response:
            self.exception_on_error(response[8], servo_id, 'setting goal pwm to %d' % pwm)
        return response



    def set_position_and_speed(self, servo_id, position, speed):
        """
        Set the servo with servo_id to specified position and speed.

        Note that this method is depreciated; under 2.0 protocol, velocity is only set directly
        (when in velocity control mode) or indirectly through velocity profile (in all other
        control modes)
        """
        b0 = (position & 0xff)
        b1 = (position >> 8) & 0xff
        b2 = (position >> 16) & 0xff
        b3 = (position >> 24) & 0xff

        response = self.write(servo_id, MX_GOAL_POSITION, (b0,b1,b2,b3, ))
        if response:
            self.exception_on_error(response[8], servo_id, 'setting goal position to %d and moving speed to %d' %(position, speed))
        return response

    def set_led(self, servo_id, led_state):
        """
        Turn the LED of servo motor on/off.
        Possible boolean state values:
            True - turn the LED on,
            False - turn the LED off.
        """
        response = self.write(servo_id, MX_LED, (led_state,))
        if response:
            self.exception_on_error(response[8], servo_id,
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
        self.sync_write(MX_TORQUE_ENABLE, tuple(valueTuples))

    """
    The following send syncronous PID and PI values for position and velocity
    controllers. They should be called like:
        set_multi_pos_p((id1,value), (id2, value2))
    Note that each controller and value have particular value limits that are
    not checked here.
    """
    def set_multi_pos_p(self, tup):
        self.sync_write(MX_POSITION_P_GAIN, tuple(tup))
    def set_multi_pos_i(self, tup):
        self.sync_write(MX_POSITION_I_GAIN, tuple(tup))
    def set_multi_pos_d(self, tup):
        self.sync_write(MX_POSITION_D_GAIN, tuple(tup))
    def set_multi_vel_p(self, tup):
        self.sync_write(MX_VELOCITY_P_GAIN, tuple(tup))
    def set_multi_vel_i(self, tup):
        self.sync_write(MX_POSITION_D_GAIN, tuple(tup))

    def set_multi_current(self, valueTuples):
        """
        Set different positions for multiple servos.
        Should be called as such:
        set_multi_current( ( (id1, current1), (id2, current2), (id3, current3) ) )

        IMPORTANT: the units of current are raw values of 3.36mA.
        """
        # prepare value tuples for call to syncwrite
        writeableVals = []

        for vals in valueTuples:
            sid = vals[0]
            current = vals[1]
            b0 = current & 0xff
            b1 = (current >> 8) & 0xff
            writeableVals.append( (sid, b0, b1) )
        # use sync write to broadcast multi servo message
        self.sync_write(MX_GOAL_CURRENT, writeableVals)


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
            b0 = position & 0xff
            b1 = (position >> 8) & 0xff
            b2 = (position >> 16) & 0xff
            b3 = (position >> 24) & 0xff
            writeableVals.append( (sid, b0, b1, b2, b3) )
        # use sync write to broadcast multi servo message
        self.sync_write(MX_GOAL_POSITION, writeableVals)

    def set_multi_speed(self, valueTuples):
        """
        Set different speeds for multiple servos.
        Should be called as such:
        set_multi_speed( ( (id1, speed1), (id2, speed2), (id3, speed3) ) )
        """

        # prepare value tuples for call to syncwrite
        writeableVals = []

        for vals in valueTuples:
            sid = vals[0]
            speed = vals[1]

            b0 = speed & 0xff
            b1 = (speed >> 8) & 0xff
            b2 = (speed >> 16) & 0xff
            b3 = (speed >> 24) & 0xff
            writeableVals.append( (sid, b0,b1,b2,b3) )

        # use sync write to broadcast multi servo message
        self.sync_write(MX_GOAL_VELOCITY, writeableVals)

    def set_multi_position_and_speed(self, valueTuples):
        """
        Set different positions and speeds for multiple servos.
        Note that setting position and velocity may not have the intended effect in protocol 2.0:
        Control depends on the Control Mode set for the motor. However, setting both values
        ensures that e.g. trajectories are still played back despite the control method used.

        Should be called as such:
        set_multi_position_and_speed( ( (id1, position1, speed1), (id2, position2, speed2), (id3, position3, speed3) ) )
        """
        # prepare value tuples for call to syncwrite
        writeableValsPos = []
        writeableValsVel = []
        for vals in valueTuples:
            sid = vals[0]
            position = vals[1]
            speed = vals[2]

            p0 = position & 0xff
            p1 = (position >> 8) & 0xff
            p2 = (position >> 16) & 0xff
            p3 = (position >> 24) & 0xff

            b0 = speed & 0xff
            b1 = (speed >> 8) & 0xff
            b2 = (speed >> 16) & 0xff
            b3 = (speed >> 24) & 0xff

            writeableValsPos.append((sid, p0, p1, p2, p3))
            #writeableValsVel.append((sid, b0, b1, b2, b3))

        self.sync_write(MX_GOAL_POSITION, tuple(writeableValsPos))
        #self.sync_write(MX_GOAL_VELOCITY, tuple(writeableValsVel))


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
        response = self.read(servo_id, MX_PRESENT_POSITION, 4)
        if response:
            self.exception_on_error(response[8], servo_id, 'fetching present position')
        position = response[9] + (response[10] << 8) + (response[11] << 16) + (response[12] << 24)
        return ctypes.c_int(position).value

    def get_speed(self, servo_id):
        """ Reads the servo's speed value from its registers. """
        response = self.read(servo_id, MX_PRESENT_VELOCITY, 4)
        if response:
            self.exception_on_error(response[8], servo_id, 'fetching present speed')
        speed = response[9] + (response[10] << 8) + (response[11] << 16) + (response[12] << 24)
        return ctypes.c_int(speed).value

    def get_torque_limit(self, servo_id):
        """ Reads the servo's speed value from its registers. """
        response = self.read(servo_id, MX_CURRENT_LIMIT, 2)
        if response:
            self.exception_on_error(response[8], servo_id, 'fetching torque limit')
        torque = response[9] + (response[10] << 8)
        return ctypes.c_short(torque).value

    def get_voltage(self, servo_id):
        """ Reads the servo's voltage. """
        response = self.read(servo_id, MX_PRESENT_INPUT_VOLTAGE, 2)
        if response:
            self.exception_on_error(response[8], servo_id, 'fetching supplied voltage')
        return (response[9] + (response[10] << 8)) / 10.0

    def get_sync_feedback_reduced(self, servo_id_list):
        response2 = self.sync_read(MX_PRESENT_CURRENT, 10, servo_id_list)

        all_states = []
        for index, response in enumerate(response2):
            if response:
                self.exception_on_error(response[8], servo_id_list[index], 'fetching full servo status')

            position = response2[index][15] + (response2[index][16] << 8) + (response2[index][17] << 16) + (response2[index][18] << 24)
            error = 0
            speed = response2[index][11] + ( response2[index][12] << 8) + (response2[index][13] << 16) + (response2[index][14] << 24)
            load_raw = ctypes.c_int16((response2[index][9] + (response2[index][10] << 8))).value
            timestamp = 0 #response[-1]

            all_states.append({ 'timestamp': timestamp,
                 'id': servo_id_list[index],
                 'temperature': 0,
                 'goal': 0.0,
                 'position': ctypes.c_int(position).value,
                 'error': ctypes.c_int(error).value,
                 'speed': ctypes.c_int(speed).value,
                 'load': load_raw
                 })
        return all_states

    def get_sync_feedback_in_one(self, servo_id_list):
        """
        Returns state from multiple servos synchronously
        """
        response_ar = self.sync_read(MX_GOAL_POSITION, 20, servo_id_list)
        #response_ar = self.sync_read(MX_GOAL_POSITION, 7, servo_id_list)
        #response2 = self.sync_read(MX_PRESENT_CURRENT, 10, servo_id_list)

        all_states = []
        for index, response in enumerate(response_ar):
            if response:
                self.exception_on_error(response[8], servo_id_list[index], 'fetching full servo status')

            goal = response[9] + (response[10] << 8) + (response[11] << 16) + (response[12] << 24)
            moving = response[15]
            position = response[25] + (response[26] << 8) + (response[27] << 16) + (response[28] << 24)
            error = position - goal
            speed = response[21] + ( response[22] << 8) + (response[23] << 16) + (response[24] << 24)
            load_raw = ctypes.c_int16((response[19] + (response[20] << 8))).value
            timestamp = response[-1]

            all_states.append({ 'timestamp': timestamp,
                 'id': servo_id_list[index],
                 'temperature': 0,
                 'goal': ctypes.c_int(goal).value,
                 'position': ctypes.c_int(position).value,
                 'error': ctypes.c_int(error).value,
                 'speed': ctypes.c_int(speed).value,
                 'load': load_raw,
                 'moving': bool(moving) })
        return all_states



    def get_sync_feedback(self, servo_id_list):
        """
        Returns state from multiple servos synchronously
        and through several read commands. Fetching smaller sections of motor RAM greatly enhances
        packet transmission accuracy, but theoretically limits the throuput.

        For faster throughput, see get_sync_feeback_in_one(..)
        """
        response_ar = self.sync_read(MX_GOAL_POSITION, 7, servo_id_list)
        response2 = self.sync_read(MX_PRESENT_CURRENT, 6, servo_id_list)
        response3 = self.sync_read(MX_PRESENT_POSITION, 4, servo_id_list)

        all_states = []
        for index, response in enumerate(response_ar):
            if response:
                self.exception_on_error(response[8], servo_id_list[index], 'fetching full servo status')

            goal = response[9] + (response[10] << 8) + (response[11] << 16) + (response[12] << 24)
            moving = response[15]
            position = response3[index][9] + (response3[index][10] << 8) + (response3[index][11] << 16) + (response3[index][12] << 24)
            error = position - goal
            speed = response2[index][11] + ( response2[index][12] << 8) + (response2[index][13] << 16) + (response2[index][14] << 24)
            load_raw = ctypes.c_int16((response2[index][9] + (response2[index][10] << 8))).value
            timestamp = response[-1]

            all_states.append({ 'timestamp': timestamp,
                 'id': servo_id_list[index],
                 'temperature': 0,
                 'goal': ctypes.c_int(goal).value,
                 'position': ctypes.c_int(position).value,
                 'error': ctypes.c_int(error).value,
                 'speed': ctypes.c_int(speed).value,
                 'load': load_raw,
                 'moving': bool(moving) })
        return all_states

    def get_feedback_reduced(self, servo_id):
        """
        Returns the id, goal, position, error, speed, load, voltage,
        and moving values from the specified servo.

        This feedback method returns a smaller set of data in order to reduce the required number of
        read packets to 1.

        """
        response = self.read(servo_id, MX_PRESENT_CURRENT, 10)
        if response:
            self.exception_on_error(response[8], servo_id, 'fetching full servo status')

        position = response[15] + (response[16] << 8) + (response[17] << 16) + (response[18] << 24)
        error = position - goal
        speed = response[11] + ( response[12] << 8) + (response[13] << 16) + (response[14] << 24)
        load_raw = ctypes.c_int16((response[9] + (response[10] << 8))).value
        timestamp = response[-1]

        # return the data in a dictionary
        return { 'timestamp': timestamp,
                 'id': servo_id,
                 'position': ctypes.c_int(position).value,
                 'error': ctypes.c_int(error).value,
                 'speed': ctypes.c_int(speed).value,
                 'load': load_raw,
                 }



    def get_feedback(self, servo_id):
        """
        Returns the id, goal, position, error, speed, load, voltage,
        and moving values from the specified servo.

        Reads are broken into 2 separate packets due to the distance between values in the new protocol.
        This reduces possible throughput by 30%.
        See 'get_feedback_reduced' that fetches synchronous feedback in 1 command.
        """
        #break into 2 reads to increase reliability
        response = self.read(servo_id, MX_GOAL_POSITION, 7)
        response2 = self.read(servo_id, MX_PRESENT_CURRENT, 10)
        if response:
            self.exception_on_error(response[8], servo_id, 'fetching full servo status')

        goal = response[9] + (response[10] << 8) + (response[11] << 16) + (response[12] << 24)
        moving = response[15]
        position = response2[15] + (response2[16] << 8) + (response2[17] << 16) + (response2[18] << 24)
        error = position - goal
        speed = response2[11] + ( response2[12] << 8) + (response2[13] << 16) + (response2[14] << 24)
        load_raw = ctypes.c_int16((response2[9] + (response2[10] << 8))).value
        timestamp = response[-1]

        # return the data in a dictionary
        return { 'timestamp': timestamp,
                 'id': servo_id,
                 'goal': ctypes.c_int(goal).value,
                 'position': ctypes.c_int(position).value,
                 'error': ctypes.c_int(error).value,
                 'speed': ctypes.c_int(speed).value,
                 'load': load_raw,
                 'moving': bool(moving) }


    """
    get_is_moving simply returns whether or not the motor is executing a trajectory or satisfying a position goal condition
    get_moving status gives more detail into what type of movement is being followed, and whether or not it was successful
        consult the documentation for specific meanins of bit values
    """
    def get_is_moving(self, servo_id):
        response = self.read(servo_id, MX_MOVING, 1)
        if response:
            self.exception_on_error(response[8], servo_id, 'fetching full servo status')

        return bool(response[9])

    def get_moving_status(self, servo_id):
        response = self.read(servo_id, MX_MOVING_STATUS, 1)
        if response:
            self.exception_on_error(response[8], servo_id, 'fetching full servo status')

        return response[9]

    def get_multi_is_moving(self, servo_id_list):
        response_ar = self.sync_read(MX_MOVING, 1, servo_id_list)
        ret = []

        for index, response in enumerate(response_ar):
            if response:
                self.exception_on_error(response[8], servo_id_list[index], 'fetching full servo status')
            ret.append( (servo_id_list[index], response[9]))

        return ret


    def get_multi_moving_status(self, servo_id_list):
        response_ar = self.sync_read(MX_MOVING_STATUS, 1, servo_id_list)
        ret = []
        for index, response in enumerate(response_ar):
            if response:
                self.exception_on_error(response[8], servo_id_list[index], 'fetching full servo status')
            ret.append( (servo_id_list[index], response[9]))
        return ret

    def get_led(self, servo_id):
        """
        Get status of the LED. Boolean return values:
            True - LED is on,
            False - LED is off.
        """
        response = self.read(servo_id, MX_LED, 1)
        if response:
            self.exception_on_error(response[8], servo_id,
                'fetching LED status')

        return bool(response[9])


    def exception_on_error(self, error_code, servo_id, command_failed):
        global exception
        exception = None
        ex_message = '[servo #%d on %s@%sbps]: %s failed' % (servo_id, self.ser.port, self.ser.baudrate, command_failed)

        if not isinstance(error_code, int):
            msg = 'Communcation Error ' + ex_message
            exception = NonfatalErrorCodeError(msg, 0)
            return
        if not error_code & MX_ACCESS_ERROR == 0:
            msg = 'Access Error ' + ex_message
            exception = FatalErrorCodeError(msg, error_code)
        if not error_code & MX_DATA_LIMIT_ERROR == 0:
            msg = 'Data Limit Error ' + ex_message
            exception = FatalErrorCodeError(msg, error_code)
        if not error_code & MX_LENGTH_ERROR == 0:
            msg = 'Packet Length Error ' + ex_message
            exception = NonfatalErrorCodeError(msg, error_code)
        if not error_code & MX_CHECKSUM_ERROR == 0:
            msg = 'Bad Checksum Error ' + ex_message
            exception = NonfatalErrorCodeError(msg, error_code)
        if not error_code & MX_RANGE_ERROR == 0:
            msg = 'Range Error ' + ex_message
            exception = NonfatalErrorCodeError(msg, error_code)
        if not error_code & MX_RESULT_FAIL == 0:
            msg = 'Result Fail Error ' + ex_message
            exception = NonfatalErrorCodeError(msg, error_code)
        if not error_code & MX_INSTRUCTION_ERROR == 0:
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
        if model_id in MX_MODEL_TO_PARAMS:
            model = MX_MODEL_TO_PARAMS[model_id]['name']
        else:
            model = 'Unknown'
        self.message = "Feature %d not supported by model %d (%s)" %(feature_id, model_id, model)
    def __str__(self):
        return self.message

