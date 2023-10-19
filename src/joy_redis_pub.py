#!/usr/bin/env python3

"""
Subscribes to joystick and writes velocity commands to redis streams

"""

import rospy
import numpy as np
import os 
import redis
import copy

from sensor_msgs.msg import Joy

RATE = 20    # hz 
DT = 1/RATE  # seconds

print("connecting to server")
r = redis.Redis(host='localhost', port=6379, password='H9@zGz4!mKts')


class JoyRedisPub:
    def __init__(self):
        rospy.init_node('joy_redis_pub_node', disable_signals=True)
        self.rate = rospy.Rate(RATE)  
        self.dt = DT  # seconds

        self.vel_cmd = np.zeros(4)

        print("Initializing joystick redis pub stream")

        # Subscribers
        joy_sub = rospy.Subscriber("joy", Joy, self.joy_cb)


    def joy_cb(self, data):
        # Unpack Data
        self.joy_buttons = data.buttons
        self.joy_axes = data.axes

        # Buttons (0 if not pressed, 1 if pressed)
        # 0: A
        # 1: B
        # 3: X
        # 4: Y
        # 6: LB
        # 7: RB
        # 13: button stick left
        # 14: button stick right

        # Axes (-1 to 1)
        # 0: left stick right/left
        # 1: left stick down/up
        # 2: right stick right/left
        # 3: right stick down/up
        # 4: RT (1 if not pressed, -1 if pressed)
        # 5: LT (1 if not pressed, -1 if pressed)
        # 6: cross key right/left
        # 7: cross key down/up

        # Right stick - XY velocity command
        self.vel_cmd[0] = -1.0 * self.joy_axes[2]
        self.vel_cmd[1] = self.joy_axes[3]

        # Left stick - Z velocity and yaw command
        self.vel_cmd[2] = self.joy_axes[1]
        self.vel_cmd[3] = self.joy_axes[0]


    def run(self, event=None):

        # Get velocity command
        print(f"vel_cmd: {self.vel_cmd}")
        vx = np.float32(self.vel_cmd[0])
        vy = np.float32(self.vel_cmd[1])
        vz = np.float32(self.vel_cmd[2])
        vr = np.float32(self.vel_cmd[3])

        t_now = rospy.Time.now()

        r.xadd('drone:decode:stream', 
               {'vel_x:float32': vx.tobytes(),
                'vel_y:float32': vy.tobytes(),
                'vel_z:float32': vz.tobytes(),
                'vel_r:float32': vr.tobytes()
                })


if __name__ == '__main__':
    
    jrb = JoyRedisPub()

    rospy.Timer(rospy.Duration(jrb.dt), jrb.run)
    rospy.spin()

