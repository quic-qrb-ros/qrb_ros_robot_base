# Copyright 2011 Brown University Robotics.
# Copyright 2017 Open Source Robotics Foundation, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
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

# Changes from Qualcomm Innovation Center, Inc. are provided under the following license:
# Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

import sys
import threading

import geometry_msgs.msg
import rclpy

from qrb_ros_robot_base_msgs.srv import SetControlMode
from qrb_ros_robot_base_msgs.srv import GetControlMode
from qrb_ros_robot_base_msgs.srv import SetMotionMode
from qrb_ros_robot_base_msgs.srv import EmergencyCmd
from qrb_ros_robot_base_msgs.srv import GetBatteryState
from qrb_ros_robot_base_msgs.msg import ChargerCmd

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


msg = """
This node takes keypresses from the keyboard and publishes them
as Twist/TwistStamped messages. It works best with a US keyboard layout.
---------------------------------------------------------------
Moving control:     Control mode:        Charger command:
   u    i    o       1   2   3   ?       8   9   0
   j    k    l      Motion mode:         Emergency command:
   m    ,    .       4   5   6   7       [   ]
---------------------------------------------------------------

k : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

Choose the control mode
1 : application control (RBx)
2 : charging-Pile control
3 : remote controller
? : query current mode

4 : speed mode (for test only)
5 : driver error (for test only)
6 : motion emergency enable (for test only, need to restart)
7 : motion emergency disable (for test only)

8 : start charging
9 : stop charging
0 : get battery state

[ : emergency enable
] : emergency disable

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0, 0, 0),
    'o': (1, 0, 0, -1),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
    'u': (1, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
    'm': (-1, 0, 0, -1),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}

controlModeBindings = {
    '1': (0, 'application control'),
    '2': (1, 'charger control'),
    '3': (2, 'remote controller'),
    '?': (3, 'query current mode'),
}

motionModeBindings = {
    '4': (1, 'speed mode'),
    '5': (4, 'driver error'),
    '6': (5, 'motion emergency brake enable'),
    '7': (6, 'motion emergency brake disable'),
}

emergencyModeBindings = {
    '[': (1, 'enable'),
    ']': (0, 'disable'),
}

chargerBindings = {
    '8': (0, 'start charging'),
    '9': (1, 'stop charging'),
    '0': (2, 'get battery state'),
}

def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(speed, turn):
    return 'currently:\tspeed %s\tturn %s ' % (speed, turn)


def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node('robot_base_keyboard')

    # parameters
    stamped = node.declare_parameter('stamped', False).value
    frame_id = node.declare_parameter('frame_id', '').value
    if not stamped and frame_id:
        raise Exception("'frame_id' can only be set when 'stamped' is True")

    if stamped:
        TwistMsg = geometry_msgs.msg.TwistStamped
    else:
        TwistMsg = geometry_msgs.msg.Twist

    namespace = ''

    speed_pub = node.create_publisher(TwistMsg, '{}cmd_vel'.format(namespace), 10)
    set_control_mode_client = node.create_client(SetControlMode, '{}set_control_mode'.format(namespace))
    get_control_mode_client = node.create_client(GetControlMode, '{}get_control_mode'.format(namespace))
    set_motion_mode_client = node.create_client(SetMotionMode, '{}set_motion_mode'.format(namespace))
    emergency_cmd_client = node.create_client(EmergencyCmd, '{}emergency_cmd'.format(namespace))
    charger_cmd_pub = node.create_publisher(ChargerCmd, '{}charger_cmd'.format(namespace), 10)
    get_battery_state_client = node.create_client(GetBatteryState, '{}get_battery_state'.format(namespace))

    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    speed = 0.2
    turn = 0.15
    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0
    status = 0.0

    twist_msg = TwistMsg()

    if stamped:
        twist = twist_msg.twist
        twist_msg.header.stamp = node.get_clock().now().to_msg()
        twist_msg.header.frame_id = frame_id
    else:
        twist = twist_msg

    try:
        print(msg)
        print(vels(speed, turn))
        while True:
            key = getKey(settings)
            if key in controlModeBindings.keys():
                # get control mode
                if key == '?':
                  print('get control mode start... ', end='', flush=True)
                  if not get_control_mode_client.wait_for_service(timeout_sec=1.0):
                      print('service not available, please check service status...')
                      continue
                  req = GetControlMode.Request()
                  resp = get_control_mode_client.call(req)

                  mode_str = 'unknown'
                  for v in controlModeBindings.values():
                      if v[0] == resp.mode:
                          mode_str = v[1]
                  print('success, mode: {}'.format(mode_str))
                  continue
                # set control mode
                mode = controlModeBindings[key]
                print('set control mode start, mode: {}... '.format(mode[1]), end='', flush=True)
                if not set_control_mode_client.wait_for_service(timeout_sec=1.0):
                    print('service not available, please check service status...')
                    continue
                req = SetControlMode.Request()
                req.mode = mode[0]
                resp = set_control_mode_client.call(req)
                print(resp.result == True and 'success' or 'failed')
                continue
            if key in motionModeBindings.keys():
                # set motion mode
                mode = motionModeBindings[key]
                print('set motion mode start, mode: {}... '.format(mode[1]), end='', flush=True)
                if not set_motion_mode_client.wait_for_service(timeout_sec=1.0):
                    print('service not available, please check service status...')
                    continue
                req = SetMotionMode.Request()
                req.mode = mode[0]
                resp = set_motion_mode_client.call(req)
                print(resp.result == True and 'success' or 'failed')
                continue
            if key in emergencyModeBindings.keys():
                # emergency cmd
                mode = emergencyModeBindings[key]
                print('set emergency mode start, mode: {}... '.format(mode[1]), end='', flush=True)
                if not emergency_cmd_client.wait_for_service(timeout_sec=1.0):
                    print('service not available, please check service status...')
                    continue
                req = EmergencyCmd.Request()
                req.mode = mode[0]
                resp = emergency_cmd_client.call(req)
                print(resp.result == True and 'success' or 'failed')
                continue
            if key in chargerBindings.keys():
                mode = chargerBindings[key]
                # charger cmd
                print('charger cmd: {}'.format(mode[1]))
                if key == '8' or key == '9':
                    charger_msg = ChargerCmd()
                    charger_msg.cmd = mode[0]
                    charger_cmd_pub.publish(charger_msg)
                # get battery state
                elif key == '0':
                    print('get battery state start... ', end='', flush=True)
                    if not get_battery_state_client.wait_for_service(timeout_sec=1.0):
                        print('service not available, please check service status...')
                        continue
                    req = GetBatteryState.Request()
                    resp = get_battery_state_client.call(req)
                    print('success, voltage: {}, current: {}, power_supply_status: {}'.format(resp.battery_state.voltage, resp.battery_state.current, resp.battery_state.power_supply_status))
                continue
            elif key == 'k':
                print('stop!!!')
                x = y = z = th = 0.0
            elif key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print(vels(speed, turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
                continue
            else:
                x = 0.0
                y = 0.0
                z = 0.0
                th = 0.0
                if (key == '\x03'):
                    break

            if stamped:
                twist_msg.header.stamp = node.get_clock().now().to_msg()

            twist.linear.x = x * speed
            twist.linear.y = y * speed
            twist.linear.z = z * speed
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = th * turn
            speed_pub.publish(twist_msg)

    except Exception as e:
        print(e)

    finally:
        if stamped:
            twist_msg.header.stamp = node.get_clock().now().to_msg()

        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        speed_pub.publish(twist_msg)
        rclpy.shutdown()
        spinner.join()

        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()
