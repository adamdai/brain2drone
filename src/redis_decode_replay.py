#!/usr/bin/env python3

"""
Replays velocity commands from redis dump file and publishes to drone

"""



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


class RedisDecodeReplay:
    def __init__(self):
        rospy.init_node('redis_decode_offline_node', disable_signals=True)
        self.rate = rospy.Rate(RATE)  # GCS rate is 20 Hz
        self.dt = DT  # seconds 

        self.tstart = 1689199189370 # exemplar
        self.tend = '1689199359295' # exemplar

        # self.tstart = 1687215526099
        # self.tend = '1687216360188'

        self.t_idx = self.tstart
        self.loop_idx = 0
        self.max_loop_idx = 300  # 100 * 50 ms = 5 s

        print("starting Redis connection")
        self.rc = redis.Redis(host='localhost', port=6379, password='H9@zGz4!mKts')
        print("ping: ", self.rc.ping())
        # info = self.rc.xinfo_stream('drone:decode:stream')
        # self.newMsgId = info['first-entry'][0].decode('utf-8')

        self.curr_pos = np.array([0.0, 0.0, 1.0])
        self.curr_att = np.zeros(4)

        # Publishers
        self.vel_pub = rospy.Publisher("drone2/setpoint/velocity", Vector3Stamped, queue_size=1)
        self.bdr_pub = rospy.Publisher("drone2/setpoint/bodyrate", Vector3Stamped, queue_size=1)

        # Subscribers
        mocap_sub = rospy.Subscriber("drone2/mavros/local_position/pose", PoseStamped, self.mocap_cb)

        self.vlog_raw = []
        self.vlog_scaled = []

        rospy.on_shutdown(self.shutdown)

    
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
        self.loop_idx += 1
        if self.loop_idx > self.max_loop_idx:
            rospy.signal_shutdown("Finished replaying data")
            return

        # Read off data from decoder outputs
        stream = self.rc.xread({'drone:stream': str(self.t_idx)}, count=1, block=1000)
        self.t_idx += 50
        # self.newMsgId = stream[0][1][0][0].decode('utf-8')
        vel_bytes = stream[0][1][0][1][b'drone_latest_vel_mod:float32']
        vel = np.frombuffer(vel_bytes, dtype=np.float32)
        self.vlog_raw.append(vel)

        AIRSIM_MAX_VEL = 8
        AIRSIM_MAX_YAWRATE = 7.5
        FR_MAX_VEL = 0.5
        FR_MAX_YAWRATE = 0.5
        v_scale = FR_MAX_VEL / AIRSIM_MAX_VEL
        r_scale = FR_MAX_YAWRATE / AIRSIM_MAX_YAWRATE
        #print(v_scale * vel)

        t_now = rospy.Time.now()

        vel_msg = Vector3Stamped()
        bdr_msg = Vector3Stamped()
        vel_msg.header.stamp = t_now
        bdr_msg.header.stamp = t_now
        if self.in_bounds():
            # NOTE: switch x and y 
            # vel_msg.vector.x = v_scale * vel[0]
            # vel_msg.vector.y = v_scale * vel[1]
            vel_msg.vector.x = v_scale * vel[1]
            vel_msg.vector.y = v_scale * vel[0]
            # vel_msg.vector.z = v_scale * vel[2]
            #bdr_msg.vector.z = r_scale * vel[3]
            bdr_msg.vector.z = 0.0
        else:
            vel_msg.vector.x = 0.0
            vel_msg.vector.y = 0.0
            vel_msg.vector.z = 0.0
            bdr_msg.vector.z = 0.0

        print(f"{self.loop_idx} / {self.max_loop_idx}: {vel_msg.vector.x:2f}, {vel_msg.vector.y:2f}, {vel_msg.vector.z:2f}")
        self.vlog_scaled.append([vel_msg.vector.x, vel_msg.vector.y, vel_msg.vector.z, bdr_msg.vector.z])

        # Publish
        self.vel_pub.publish(vel_msg)
        self.bdr_pub.publish(bdr_msg)

    
    def shutdown(self):
        print("Shutting down")
        np.savez('src/brain2drone/notebooks/log', raw=np.array(self.vlog_raw), scaled=np.array(self.vlog_scaled))


if __name__ == '__main__':
    
    rdc = RedisDecodeReplay()

    rospy.Timer(rospy.Duration(rdc.dt), rdc.run)
    rospy.spin()

