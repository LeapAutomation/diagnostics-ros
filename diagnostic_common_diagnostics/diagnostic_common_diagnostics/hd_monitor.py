#! /usr/bin/env python3
"""Hard Drive (or any other memory) monitor. Contains a the monitor node and its main function."""
# -*- coding: utf-8 -*-
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
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

# \author Kevin Watts
# \author Antoine Lima

from pathlib import Path
from shutil import disk_usage
from socket import gethostname
from typing import List
import traceback

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from diagnostic_updater import Updater
from diagnostic_updater.diagnostic_updater._diagnostic_updater import DiagnosticTask
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
import rclpy
from rclpy.node import Node


DICT_STATUS = {
    DiagnosticStatus.OK: 'OK',
    DiagnosticStatus.WARN: 'Warning',
    DiagnosticStatus.ERROR: 'Error',
}
DICT_USAGE = {
    DiagnosticStatus.OK: 'OK',
    DiagnosticStatus.WARN: 'Low Disk Space',
    DiagnosticStatus.ERROR: 'Very Low Disk Space',
}


class HDTask(DiagnosticTask):
    """
    Diagnostic task checking the remaining space on the specified hard drive.

    """

    def __init__(self, path, warning_percentage, error_percentage):
        self._path = path
        self._warning_percentage = warning_percentage
        self._error_percentage = error_percentage

    def run(self, stat):
        total, used, _ = disk_usage(self._path)
        percent = used / total * 100.0

        stat.add('HD Path', f'{self._path}')
        stat.add('HD Total (Gb)', f'{total // (1024 * 1024)}')
        stat.add('HD Usage (%)', f'{percent:.2f}')

        if percent >= self._error_percentage:
            stat.summary(DiagnosticStatus.ERROR,
                         f'HD usage exceeds {self._error_percentage} percent')
        elif percent >= self._warning_percentage:
            stat.summary(DiagnosticStatus.WARN,
                         f'HD usage exceeds {self._warning_percentage} percent')
        else:
            stat.summary(DiagnosticStatus.OK,
                         f'HD usage {percent:.2f} percent')

        return stat


def main(args=None):
    """Run the HDMonitor class."""
    rclpy.init(args=args)

    # Create the node
    hostname = gethostname()
    # Every invalid symbol is replaced by underscore.
    # isalnum() alone also allows invalid symbols depending on the locale
    cleaned_hostname = ''.join(
        c if (c.isascii() and c.isalnum()) else '_' for c in hostname)
    node = Node(f'hd_monitor_{cleaned_hostname}')

    # Declare and get parameters
    node.declare_parameter('warning_percentage', 90)
    node.declare_parameter('error_percentage', 99)

    warning_percentage = node.get_parameter(
        'warning_percentage').get_parameter_value().integer_value
    error_percentage = node.get_parameter(
        'error_percentage').get_parameter_value().integer_value
    path = node.get_parameter('path').get_parameter_value().string_value

    # Create diagnostic updater with default updater rate of 1 hz
    updater = Updater(node)
    updater.setHardwareID(hostname)
    updater.add(HDTask(path=path,
                        warning_percentage=warning_percentage,
                        error_percentage=error_percentage))

    rclpy.spin(node)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    except Exception:
        traceback.print_exc()