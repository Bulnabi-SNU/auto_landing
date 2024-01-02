#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

__author__ = "Jaeyoung Lim"
__contact__ = "jalim@ethz.ch"

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus, VehicleOdometry
from std_msgs.msg import Int32, Float32MultiArray


class OffboardControl(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile
        )
        self.keycommand_sub = self.create_subscription(
            Int32,
            '/Key_Pressed',
            self.keyboard_command_callback,
            10
        )
        self.keycommand_sub

        self.command_sub = self.create_subscription(
            Float32MultiArray,
            '/point_to_go',
            self.point_command_callback,
            10
        )
        self.command_sub

        # TODO : Replace with GPS data
        self.odometry_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.vehicle_odometry_callback,
            qos_profile
        )
        self.odometry_sub

        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.command_num = 0

        #Way point
        self.global_waypoint = [-2., 0., -10.]  #[50., -14., -15.]
        self.vehicle_global_position = [0., 0., 0.]
 
    def vehicle_status_callback(self, msg):
        # TODO: handle NED->ENU transformation
        # print("NAV_STATUS: ", msg.nav_state)
        # print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.nav_state = msg.nav_state

    def keyboard_command_callback(self, msg):
        #Receives the Keyboard Inputs
        self.command_num = msg.data
        # print("Received", msg.data)

    def point_command_callback(self, msg):
        # TODO : Receive command from point clouds
        local_waypoint = msg.data
        q = self.vehicle_quad
        p = self.vehicle_global_position

        
        wp = local_to_global(local_waypoint,  p, q)

        if np.linalg.norm(np.array(self.vehicle_global_position) - np.array(self.global_waypoint)) < 10.0:
            self.global_waypoint = wp

        print("To go: ({}, {}, {})".format(self.global_waypoint[0], self.global_waypoint[1], self.global_waypoint[2]))

    def vehicle_odometry_callback(self, msg):
        self.vehicle_global_position = msg.position
        self.vehicle_quad = msg.q

        # print(f"Current Position: ({msg.position[0]}, {msg.position[1]}, {msg.position[2]})")

    def cmdloop_callback(self):
        # Publish offboard control modes
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position=True
        offboard_msg.velocity=True
        offboard_msg.acceleration=False
        self.publisher_offboard_mode.publish(offboard_msg)
        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:

            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)

            # trajectory_msg.position[0] = self.global_waypoint[0]
            # trajectory_msg.position[1] = self.global_waypoint[1]
            # trajectory_msg.position[2] = self.global_waypoint[2]

            trajectory_msg.position[0] = np.nan
            trajectory_msg.position[1] = np.nan
            trajectory_msg.position[2] = np.nan

            if self.command_num == 1:
                trajectory_msg.velocity[0] = 1
                trajectory_msg.velocity[1] = 0.0
                trajectory_msg.velocity[2] = 0.0
                #print("Forward Command")

            elif self.command_num == 2:
                trajectory_msg.velocity[0] = -1
                trajectory_msg.velocity[1] = 0.0
                trajectory_msg.velocity[2] = 0.0
                #print("Back Command")

            elif self.command_num == 3:
                trajectory_msg.velocity[0] = 0.0
                trajectory_msg.velocity[1] = -1
                trajectory_msg.velocity[2] = 0.0
                #print("Left Command")

            elif self.command_num == 4:
                trajectory_msg.velocity[0] = 0.0
                trajectory_msg.velocity[1] = 1
                trajectory_msg.velocity[2] = 0.0
                #print("Right Command")

            elif self.command_num == 5:
                trajectory_msg.velocity[0] = 0.0
                trajectory_msg.velocity[1] = 0.0
                trajectory_msg.velocity[2] = -1
                #print("Up Command")

            elif self.command_num == 6:
                trajectory_msg.velocity[0] = 0.0
                trajectory_msg.velocity[1] = 0.0
                trajectory_msg.velocity[2] = 1
                #print("Down Command")

            elif self.command_num == 0:
                trajectory_msg.velocity[0] = 0.0
                trajectory_msg.velocity[1] = 0.0
                trajectory_msg.velocity[2] = 0.0
                #print("Stop Command")

            self.publisher_trajectory.publish(trajectory_msg)

def local_to_global(p_b, p, q):
    q0, q1, q2, q3 = q[0], q[1], q[2], q[3]

    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    ps0 = r00 * p_b[0] + r01 * p_b[1] + r02 * p_b[2] + p[0]
    ps1 = r10 * p_b[0] + r11 * p_b[1] + r12 * p_b[2] + p[1]
    ps2 = r20 * p_b[0] + r21 * p_b[1] + r22 * p_b[2] + p[2]
    
    return [ps0, ps1, ps2]

def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
