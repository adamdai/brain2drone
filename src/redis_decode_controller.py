#!/usr/bin/env python3

import rospy
from typing import Tuple
import numpy as np
from geometry_msgs.msg import Vector3Stamped, PoseStamped
import os 
import redis
import copy

# Parameters
X_BOUNDS = [-3.0, 3.0]
Y_BOUNDS = [-2.0, 2.0]
Z_BOUNDS = [0.0, 1.5]

RATE = 20    # hz 
DT = 1/RATE  # seconds

"""
Takeoff at origin (0, 0, 1)

"""


class RedisDecodeController:
    def __init__(self):
        rospy.init_node('redis_decode_controller_node', disable_signals=True)
        self.rate = rospy.Rate(RATE)  # GCS rate is 20 Hz
        self.dt = DT  # seconds

        print("starting Redis connection")
        self.rc = redis.Redis(host='localhost', port=6379)
        info = self.rc.xinfo_stream('drone:decode:stream')
        self.newMsgId = info['first-entry'][0].decode('utf-8')

        self.curr_pos = np.array([0.0, 0.0, 1.0])
        self.curr_att = np.zeros(4)

        # Publishers
        self.vel_pub = rospy.Publisher("drone2/setpoint/velocity", Vector3Stamped, queue_size=1)

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

    
    def in_bounds(self):
        return (self.curr_pos[0] >= X_BOUNDS[0] and self.curr_pos[0] <= X_BOUNDS[1] and
                self.curr_pos[1] >= Y_BOUNDS[0] and self.curr_pos[1] <= Y_BOUNDS[1] and
                self.curr_pos[2] >= Z_BOUNDS[0] and self.curr_pos[2] <= Z_BOUNDS[1])


    def run(self, event=None):
        # Read off data from decoder outputs
        stream = self.rc.xread({'drone:decode:stream': '$'}, count=1, block=1000)
        # self.newMsgId = stream[0][1][0][0].decode('utf-8')
        vx = stream[0][1][0][1][b'vel_x:float32']
        vy = stream[0][1][0][1][b'vel_y:float32']
        vz = stream[0][1][0][1][b'vel_z:float32']
        vr = stream[0][1][0][1][b'vel_r:float32']
        
        vx = np.frombuffer(vx, dtype=np.float32)
        vy = np.frombuffer(vy, dtype=np.float32)
        vz = np.frombuffer(vz, dtype=np.float32)
        vr = np.frombuffer(vr, dtype=np.float32)
        print(f'{vx} {vy} {vz} {vr}')

        t_now = rospy.Time.now()

        vel_msg = Vector3Stamped()
        vel_msg.header.stamp = t_now
        if self.in_bounds():
            vel_msg.vector.x = vx
            vel_msg.vector.y = vy
            vel_msg.vector.z = vz
        else:
            vel_msg.vector.x = 0.0
            vel_msg.vector.y = 0.0
            vel_msg.vector.z = 0.0

        # Publish
        self.vel_pub.publish(vel_msg)


if __name__ == '__main__':
    
    rdc = RedisDecodeController()

    rospy.Timer(rospy.Duration(rdc.dt), rdc.run)
    rospy.spin()

