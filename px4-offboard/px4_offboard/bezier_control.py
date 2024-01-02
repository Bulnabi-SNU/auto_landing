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

__author__ = "Jaeyoung Lim, Modified by Jongann Lee"
__contact__ = "jalim@ethz.ch, johnny3357@snu.ac.kr"

import rclpy
import copy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition
from std_msgs.msg import Float32MultiArray

hz = 50 #system hz, must be synchronized to the main callback frequency

class points():
    def __init__(self,xi,xf,vi,vf,hz):
        self.xi = xi
        self.xf = xf
        self.vi = vi
        self.vf = vf
        self.hz = hz
        self.vmax = 0.5 ##TBD
        self.amax = 2 ##TBD
        (self.t,self.point1,self.point2,self.point3,self.point4) = self.time_calibrate()
        self.count = int(self.t * self.hz)
        self.timecount = 1/(self.count+1e-10)
    def bezier_x(self):
        bezx= np.zeros(self.count)
        for count in range(self.count):
            bezx[count] = (self.point4[0]*(count * self.timecount)**3 +
            3 * self.point3[0]*(count * self.timecount)**2 * (1-count * self.timecount)+
            3 * self.point2[0]*(count * self.timecount)**1 * (1-count * self.timecount)**2+
            1 * self.point1[0]*(1-count * self.timecount)**3
            )
        return bezx
    def bezier_y(self):
        bezy= np.zeros(self.count)
        for count in range(self.count):
            bezy[count] = (self.point4[1]*(count * self.timecount)**3 +
            3 * self.point3[1]*(count * self.timecount)**2 * (1-count * self.timecount)+
            3 * self.point2[1]*(count * self.timecount)**1 * (1-count * self.timecount)**2+
            1 * self.point1[1]*(1-count * self.timecount)**3
            )
        return bezy
    def bezier_z(self):
        bezz= np.zeros(self.count)
        for count in range(self.count):
            bezz[count] = (self.point4[2]*(count * self.timecount)**3 +
            3 * self.point3[2]*(count * self.timecount)**2 * (1-count * self.timecount)+
            3 * self.point2[2]*(count * self.timecount)**1 * (1-count * self.timecount)**2+
            1 * self.point1[2]*(1-count * self.timecount)**3
            )
        return bezz
    def bezier_vx(self):
        bezvx = np.zeros(self.count)
        for count in range(self.count):
            bezvx[count] = (self.hz * self.timecount) * (3 * self.point4[0] * (count * self.timecount) ** 2 +
                                               6 * self.point3[0] * (count * self.timecount) * (1 - count * self.timecount) +
                                               3 * self.point2[0] * (1 - count * self.timecount) ** 2 +
                                               -3 * self.point3[0] * (count * self.timecount) ** 2 +
                                               -6 * self.point2[0] * (count * self.timecount) * (1 - count * self.timecount) +
                                               -3 * self.point1[0] * (1 - count* self.timecount)**2)
        return bezvx
    def bezier_vy(self):
        bezvy = np.zeros(self.count)
        for count in range(self.count):
            bezvy[count] = (self.hz * self.timecount) * (3 * self.point4[1] * (count * self.timecount) ** 2 +
                                               6 * self.point3[1] * (count * self.timecount) * (1 - count * self.timecount) +
                                               3 * self.point2[1] * (1 - count * self.timecount) ** 2 +
                                               -3 * self.point3[1] * (count * self.timecount) ** 2 +
                                               -6 * self.point2[1] * (count * self.timecount) * (1 - count * self.timecount) +
                                               -3 * self.point1[1] * (1 - count* self.timecount)**2)
        return bezvy
    def bezier_vz(self):
        bezvz = np.zeros(self.count)
        for count in range(self.count):
            bezvz[count] = (self.hz * self.timecount) * (3 * self.point4[2] * (count * self.timecount) ** 2 +
                                               6 * self.point3[2] * (count * self.timecount) * (1 - count * self.timecount) +
                                               3 * self.point2[2] * (1 - count * self.timecount) ** 2 +
                                               -3 * self.point3[2] * (count * self.timecount) ** 2 +
                                               -6 * self.point2[2] * (count * self.timecount) * (1 - count * self.timecount) +
                                               -3 * self.point1[2] * (1 - count* self.timecount)**2)
        return bezvz
    def time_calibrate(self):
        t_calculated= np.linalg.norm(self.xf - self.xi)/self.vmax
        t = np.linalg.norm(self.xf - self.xi)/self.vmax
        #print("t_init :",t)
        flag = 0
        a_max = 0
        while flag == 0:
            point1 = self.xi
            point4 = self.xf
            point3 = self.xf - self.vf*t/3
            # if np.linalg.norm(self.vi) < 1:
            #     point2 = self.xi + 1*(self.xf - self.xi)*t/(3*np.linalg.norm((self.xf - self.xi)))
            # else:
            point2 = self.xi + (self.vi + 0.05 * self.vi / np.linalg.norm(self.vi))*t/3  ##catastrophic.
            # point2 = [point1[0]/3+2*point4[0]/3, point1[1]/3+2*point4[1]/3, 1.5*point1[2]]
            point2[2] = (2 * self.xi[2] + self.xf[2])/3
            flag = 1#
            # A1 = 6* (point1[0] + point2[0] - point3[0] + point4[0])
            # A2 = 6*point1[0] - 2*point2[0] - 2*point3[0] + 2*point4[0]
            # B1 = 6* (-point1[1] + point2[1] - point3[1] + point4[1])
            # B2 = 6*point1[1] - 2*point2[1] - 2*point3[1] + 2*point4[1]
            # C1 = 6* (-point1[2] + point2[2] - point3[2] + point4[2])
            # C2 = 6*point1[2] - 2*point2[2] - 2*point3[2] + 2*point4[2]
            # t_maxaccel = - (A1*A2 + B1*B2 + C1*C2) / (A1**2+B1**2+C1**2)
            # print("t max accel:",t_maxaccel)

            # if t_maxaccel>0 and t_maxaccel<1:
            #     a_max = np.sqrt((A1*t_maxaccel+A2)**2+
            #                     (B1*t_maxaccel+B2)**2+
            #                     (C1*t_maxaccel+C2)**2) #/T_TOTAL^2
            # elif t_maxaccel < 0:
            #     a_max = np.sqrt((A2)**2+(B2)**2+(C2)**2)
            # elif t_maxaccel > 1:
            #     a_max = np.sqrt((A1+A2)**2 + (B1+B2)**2 + (C1+C2)**2)
            # t_calculated = np.sqrt(a_max/self.amax)
            # print("t calculated:",t_calculated)


            # if (t_calculated <= t):
            #     t_calibrated = t
            #     flag = 1 #end loop
            # if (t_calculated > t):
            #     t = t_calculated
        calibrated = ([t,point1,point2,point3,point4])#t_calibrated
        return calibrated


class BezierControl(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.count = 0
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

        self.command_sub = self.create_subscription(
            Float32MultiArray,
            '/bezier_waypoint',
            self.point_command_callback,
            10
        )
        self.command_sub

        # TODO : Replace with GPS data
        self.odometry_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.vehicle_position_callback,
            qos_profile
        )

        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.timer_period = 0.02  # seconds
        self.timer = self.create_timer(self.timer_period, self.cmdloop_callback)

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.delta_t = 0
        self.trigger = 0

        #Way point
        self.vehicle_position = np.zeros(3)
        self.vehicle_velocity = np.zeros(3)
        self.xf = np.zeros(3)
        self.vf = np.zeros(3)
 
    def vehicle_status_callback(self, msg):
        # TODO: handle NED->ENU transformation
        print("NAV_STATUS: ", msg.nav_state)
        print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.nav_state = msg.nav_state

    def point_command_callback(self, msg):
        self.xf = np.asfarray(msg.data[0:3])
        self.vf = np.asfarray(msg.data[3:6])
        self.init_position = copy.deepcopy(self.vehicle_position)
        #Execute the bezier curve class
        # if np.linalg.norm(self.xf - self.init_position) > 1:  # stop updating the bezier curve if 1m or less from the goal
        bezier_points = points(self.init_position, self.xf, self.vehicle_velocity, self.vf, hz)
        self.x = bezier_points.bezier_x()
        self.y = bezier_points.bezier_y()
        self.z = bezier_points.bezier_z()
        self.vx = bezier_points.bezier_vx()
        self.vy = bezier_points.bezier_vy()
        self.vz = bezier_points.bezier_vz()
        self.count = bezier_points.count
        self.t = bezier_points.t#
        self.point2 = bezier_points.point2#
        self.point1 = bezier_points.point1#
        self.point3 = bezier_points.point3#
        self.point4 = bezier_points.point4#
        self.trigger = 1 # T reset trigger


    def vehicle_position_callback(self, msg):
        self.vehicle_position[0] = msg.x
        self.vehicle_position[1] = msg.y
        self.vehicle_position[2] = msg.z
        if self.delta_t > 0 and self.delta_t < self.count-1 :
            self.vehicle_velocity[0] = self.vx[self.delta_t]
            self.vehicle_velocity[1] = self.vy[self.delta_t]
            self.vehicle_velocity[2] = self.vz[self.delta_t]
        else:
            self.vehicle_velocity[0] = msg.vx
            self.vehicle_velocity[1] = msg.vy
            self.vehicle_velocity[2] = msg.vz
        #print(f"Current Position: ({msg.x}, {msg.y}, {msg.z})")


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
            
            if self.delta_t + int(1/self.timer_period) < self.count-1 and np.linalg.norm(self.vehicle_position-self.xf) > 0.5:   # if receiving command from the bezier curve
                # if np.linalg.norm(self.vehicle_velocity) < 2 and np.linalg.norm(self.xf-self.vehicle_position) > 1:  # for an initial start in case the vehicle velocity is too low
                #     trajectory_msg.position[0] = np.nan
                #     trajectory_msg.position[1] = np.nan
                #     trajectory_msg.position[2] = np.nan
                #     trajectory_msg.velocity[0] = (self.xf[0]-self.vehicle_position[0])/np.linalg.norm(self.xf-self.vehicle_position)
                #     trajectory_msg.velocity[1] = (self.xf[1]-self.vehicle_position[1])/np.linalg.norm(self.xf-self.vehicle_position)
                #     trajectory_msg.velocity[2] = (self.xf[2]-self.vehicle_position[2])/np.linalg.norm(self.xf-self.vehicle_position)
                  # main part receiving commands from the bezier curve, currently using position control only
                trajectory_msg.position[0] = self.x[self.delta_t + int(1/self.timer_period)]#np.nan
                trajectory_msg.position[1] = self.y[self.delta_t + int(1/self.timer_period)]#np.nan
                trajectory_msg.position[2] = self.z[self.delta_t + int(1/self.timer_period)]#np.nan
                trajectory_msg.velocity[0] = np.nan #self.vx[self.delta_t] 
                trajectory_msg.velocity[1] = np.nan #self.vy[self.delta_t]
                trajectory_msg.velocity[2] = np.nan #self.vz[self.delta_t]
                #yaw sending, to match coordinates.
                if np.arctan2(self.vx[self.delta_t],self.vy[self.delta_t]) < np.pi/2 and np.arctan2(self.vx[self.delta_t],self.vy[self.delta_t]) > -3*np.pi/2 :
                    trajectory_msg.yaw = -np.arctan2(self.vx[self.delta_t],self.vy[self.delta_t]) + np.pi/2
                elif np.arctan2(self.vx[self.delta_t],self.vy[self.delta_t]) > np.pi/2:
                    trajectory_msg.yaw = -np.arctan2(self.vx[self.delta_t],self.vy[self.delta_t]) - 3 * np.pi/2
                else :
                    trajectory_msg.yaw = -np.arctan2(self.vx[self.delta_t],self.vy[self.delta_t]) + 5 * np.pi/2

                print(self.point1,self.point2,self.point3,self.point4,trajectory_msg.yaw,"\n",
                      self.t, self.delta_t ,self.vx[self.delta_t], self.vy[self.delta_t], self.vz[self.delta_t],"\n",
                      self.vehicle_velocity
                      )
                self.delta_t += 1
                
            else:  # if there are no commands, just stay still
                trajectory_msg.position[0] = self.xf[0]
                trajectory_msg.position[1] = self.xf[1]
                trajectory_msg.position[2] = self.xf[2]
                trajectory_msg.velocity[0] = np.nan
                trajectory_msg.velocity[1] = np.nan
                trajectory_msg.velocity[2] = np.nan
                #yaw sending, to match coordinates.
                if np.arctan2(self.vf[0],self.vf[1]) < np.pi/2 and np.arctan2(self.vf[0],self.vf[1]) > -3*np.pi/2 :
                    trajectory_msg.yaw = np.arctan2(self.vf[0],self.vf[1]) - np.pi/2
                elif np.arctan2(self.vf[0],self.vf[1]) > np.pi/2:
                    trajectory_msg.yaw = np.arctan2(self.vf[0],self.vf[1]) + 3 * np.pi/2
                else :
                    trajectory_msg.yaw = np.arctan2(self.vf[0],self.vf[1]) - 5 * np.pi/2
                
            if self.trigger == 1:  # delta_t reset 
                self.delta_t = 0
                self.trigger = 0

            self.publisher_trajectory.publish(trajectory_msg)



def main(args=None):
    rclpy.init(args=args)

    bezier_control = BezierControl()

    rclpy.spin(bezier_control)

    bezier_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
