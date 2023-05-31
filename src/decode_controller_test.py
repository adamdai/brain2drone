#!/usr/bin/env python3

import rospy
from typing import Tuple
import numpy as np
from geometry_msgs.msg import PointStamped, QuaternionStamped, PoseStamped
import os 
import redis
import copy

# Parameters
X_BOUNDS = [-3.0, 3.0]
Y_BOUNDS = [-2.0, 2.0]
Z_BOUNDS = [0.0, 1.5]

# Define trajectory
RATE = 10    # hz 
DT = 1/RATE  # seconds
T = 10.0     # seconds
N = int(T/DT)  # number of points
tspan = np.linspace(0, T, N)

# Circle trajectory (centered at (1,0), radius 1)
V = 0.2
traj_x = 1 + np.cos(tspan)
traj_y = np.sin(tspan)
traj_z = np.ones(N)
traj_vx = V * -np.sin(tspan)
traj_vy = V * np.cos(tspan)
traj_vz = np.zeros(N)
traj_vr = np.zeros(N)



class DecodeControllerTest:
    def __init__(self):
        rospy.init_node('decode_controller_test_node', disable_signals=True)
        self.rate = rospy.Rate(RATE)  # GCS rate is 20 Hz
        self.dt = DT  # seconds

        self.update_dt = 1.0  # seconds

        print("Testing decode controller")

        self.curr_pos = np.array([0.0, 0.0, 1.0])
        self.curr_att = np.zeros(4)

        self.ref_pos = np.array([0.0, 0.0, 1.0])

        self.count = 0

        # Publishers
        self.pos_pub = rospy.Publisher("gcs/setpoint/position", PointStamped, queue_size=1)
        self.att_pub = rospy.Publisher("gcs/setpoint/attitude", QuaternionStamped, queue_size=1)

        # Subscribers
        mocap_sub = rospy.Subscriber("drone2/mavros/local_position/pose", PoseStamped, self.mocap_cb)

    
    def mocap_cb(self, data):
        # Unpack Data
        self.curr_pos[0] = data.pose.position.x
        self.curr_pos[1] = data.pose.position.y
        self.curr_pos[2] = data.pose.position.z

        self.curr_att[0] = data.pose.orientation.w
        self.curr_att[1] = data.pose.orientation.x
        self.curr_att[2] = data.pose.orientation.y
        self.curr_att[3] = data.pose.orientation.z


    def update_ref_pos(self, event=None):
        self.ref_pos = copy.deepcopy(self.curr_pos)


    def run(self, event=None):

        # Up and down motion
        if self.count < N:
            # Up and down trajectory
            #vx = 0.0
            #vy = 0.0
            #vz = 0.05  # TODO: try increasing this (projected velocity vector)
            vx = traj_vx[self.count]
            vy = traj_vy[self.count]
            vz = traj_vz[self.count]
            print(f"v: {vx}, {vy}, {vz}")
        else:
            vz = 0.0
            print("stopped")
        self.count += 1

        t_now = rospy.Time.now()

        # Variables to publish
        pos_msg = PointStamped()
        att_msg = QuaternionStamped()

        # Position
        pos_msg.header.stamp = t_now
        pos_msg.header.frame_id = "map"

        self.ref_pos[0] += vx
        self.ref_pos[1] += vy
        self.ref_pos[2] += vz
        pos_msg.point.x = self.ref_pos[0] 
        pos_msg.point.y = self.ref_pos[1] 
        pos_msg.point.z = self.ref_pos[2]

        # Bound commanded position to be within the arena
        pos_msg.point.z = np.clip(pos_msg.point.z, Z_BOUNDS[0], Z_BOUNDS[1])
        pos_msg.point.y = np.clip(pos_msg.point.y, Y_BOUNDS[0], Y_BOUNDS[1])
        pos_msg.point.x = np.clip(pos_msg.point.x, X_BOUNDS[0], X_BOUNDS[1])

        # Attitude
        att_msg.header.stamp = t_now
        att_msg.header.frame_id = "map"
        
        att_msg.quaternion.w = 1.0
        att_msg.quaternion.x = 0.0
        att_msg.quaternion.y = 0.0
        att_msg.quaternion.z = 0.0

        # Publish
        self.pos_pub.publish(pos_msg)
        self.att_pub.publish(att_msg)


if __name__ == '__main__':
    
    dc = DecodeControllerTest()

    rospy.Timer(rospy.Duration(dc.dt), dc.run)
    rospy.Timer(rospy.Duration(dc.update_dt), dc.update_ref_pos)
    rospy.spin()

