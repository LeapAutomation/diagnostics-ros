#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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
#  * Neither the name of the Willow Garage nor the names of its
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

import socket
import sys
import threading
import traceback

import diagnostic_updater as DIAG
from diagnostic_updater.diagnostic_updater._diagnostic_updater import Updater
from diagnostic_msgs.msg import DiagnosticStatus
import ntplib
import rclpy
from rclpy.node import Node


class NTPTask(DiagnosticTask):
    """A diagnostic task that monitors the NTP offset of the system clock."""

    def __init__(self, ntp_hostname, ntp_port, warning_offset, error_offset):
        """Initialize the NTPMonitor."""

        self._ntp_hostname = ntp_hostname
        self._ntp_port = ntp_port
        self._warning_offset = warning_offset
        self._error_offset = error_offset

    def run(self, stat):
        ntp_client = ntplib.NTPClient()
        response = None
        exception_msg = ''
        try:
            response = ntp_client.request(
                self.ntp_hostname,
                port=self.ntp_port,
                version=3)
        except ntplib.NTPException as e:
            exception_msg = str(e)

        if response is not None:
            measured_offset = response.offset * 1e6
            stat.add('NTP offset to ' + self._ntp_hostname + \
            ':' + str(self._ntp_port), f'{measured_offset:.2f}')

            if (abs(measured_offset) > self._error_offset):
                stat.summary(DiagnosticStatus.ERROR,
                         f'NTP offset above error threshold: abs({measured_offset})>'\
                         f'{self._error_offset} us')
            elif (abs(measured_offset) > self._warning_offset):
                stat.summary(DiagnosticStatus.WARN,
                         f'NTP offset above threshold: abs({measured_offset})>'\
                         f'{self._warning_offset} us')
            else:
                stat.summary(DiagnosticStatus.OK,
                         f'NTP Offset abs({measured_offset}) us')
        else:
            stat.summary(DiagnosticStatus.ERROR,
                         f'NTP Error: {exception_msg}')

    def ntp_diag(self, st):
        """Add ntp diagnostics to the given status message `st` and return it."""

        def add_kv(stat_values, key, value):
            kv = DIAG.KeyValue()
            kv.key = key
            kv.value = value
            stat_values.append(kv)

        ntp_client = ntplib.NTPClient()
        response = None
        try:
            response = ntp_client.request(
                self.ntp_hostname,
                port=self.ntp_port,
                version=3)
        except ntplib.NTPException as e:
            self.get_logger().error(f'NTP Error: {e}')
            st.level = DIAG.DiagnosticStatus.ERROR
            st.message = f'NTP Error: {e}'
            add_kv(st.values, 'Offset (us)', 'N/A')
            add_kv(st.values, 'Errors', str(e))

        if response is not None:
            measured_offset = response.offset * 1e6
            if (abs(measured_offset) > self.offset):
                st.level = DIAG.DiagnosticStatus.WARN
                st.message = \
                    f'NTP offset above threshold: abs({measured_offset})>'\
                    f'{self.offset} us'
            if (abs(measured_offset) > self.error_offset):
                st.level = DIAG.DiagnosticStatus.ERROR
                st.message = \
                    f'NTP offset above error threshold: abs({measured_offset})>'\
                    f'{self.error_offset} us'
            if (abs(measured_offset) < self.offset):
                st.level = DIAG.DiagnosticStatus.OK
                st.message = f'NTP Offset OK: abs({measured_offset}) us'

        return st


def main(args=None):
    rclpy.init(args=args)

    # Create the node
    hostname = socket.gethostname()
    # Every invalid symbol is replaced by underscore.
    # isalnum() alone also allows invalid symbols depending on the locale
    cleaned_hostname = ''.join(
        c if (c.isascii() and c.isalnum()) else '_' for c in hostname)
    node = Node(f'ntp_monitor_{cleaned_hostname}')

    # Declare and get parameters
    node.declare_parameter('ntp_hostname', 'pool.ntp.org')
    node.declare_parameter('ntp_port', 123)
    node.declare_parameter('warning_offset_tolerance', 500)
    node.declare_parameter('error_offset_tolerance', 5000000)

    ntp_hostname = node.get_parameter(
        'ntp_hostname').get_parameter_value().string_value
    ntp_port = node.get_parameter(
        'ntp_port').get_parameter_value().integer_value
    warning_offset = node.get_parameter(
        'warning_offset_tolerance').get_parameter_value().integer_value
    error_offset = node.get_parameter(
        'error_offset_tolerance').get_parameter_value().integer_value

    # Create diagnostic updater with default updater rate of 1 hz
    updater = Updater(node)
    updater.setHardwareID(hostname)
    updater.add(NTPTask(ntp_hostname=ntp_hostname,
                        ntp_port=ntp_port,
                        warning_offset=warning_offset,
                        error_offset=error_offset))

    rclpy.spin(node)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    except Exception:
        traceback.print_exc()
