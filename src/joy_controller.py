#!/usr/bin/env python3

import rospy
import numpy as np
import os 
import redis
import copy

from geometry_msgs.msg import PointStamped, QuaternionStamped, PoseStamped, Twist
from sensor_msgs.msg import Joy


# Parameters
X_BOUNDS = [-3.0, 3.0]
Y_BOUNDS = [-2.0, 2.0]
Z_BOUNDS = [0.5, 1.5]

RATE = 100    # hz 
DT = 1/RATE  # seconds


class JoyController:
    def __init__(self):
        rospy.init_node('joy_controller_node', disable_signals=True)
        self.rate = rospy.Rate(RATE)  # GCS rate is 20 Hz
        self.dt = DT  # seconds

        self.update_dt = 1.0  # seconds

        print("Initializing joy controller")

        self.curr_pos = np.array([0.0, 0.0, 1.0])
        self.curr_att = np.zeros(4)

        self.vel_cmd = np.zeros(3)

        # Publishers
        self.pos_pub = rospy.Publisher("gcs/setpoint/position", PointStamped, queue_size=1)
        self.att_pub = rospy.Publisher("gcs/setpoint/attitude", QuaternionStamped, queue_size=1)
        self.vel_pub = rospy.Publisher("gcs/setpoint/velocity", Twist, queue_size=1)

        # Subscribers
        mocap_sub = rospy.Subscriber("drone2/mavros/local_position/pose", PoseStamped, self.mocap_cb)
        joy_sub = rospy.Subscriber("joy", Joy, self.joy_cb)
    
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
        self.vel_cmd[0] = -self.joy_axes[2]
        self.vel_cmd[1] = self.joy_axes[3]
        


    def update_ref_pos(self, event=None):
        self.ref_pos = copy.deepcopy(self.curr_pos)


    def run(self, event=None):

        # Get velocity command
        print(f"vel_cmd: {self.vel_cmd}")
        vx = self.vel_cmd[0]
        vy = self.vel_cmd[1]
        vz = self.vel_cmd[2]

        t_now = rospy.Time.now()

        # Variables to publish
        vel_msg = Twist()
        if self.in_bounds():
            vel_msg.linear.x = vx
            vel_msg.linear.y = vy
            vel_msg.linear.z = vz
        else:
            vel_msg.linear.x = 0.0
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0

        # Publish
        self.vel_pub.publish(vel_msg)


if __name__ == '__main__':
    
    jc = JoyController()

    rospy.Timer(rospy.Duration(jc.dt), jc.run)
    rospy.Timer(rospy.Duration(jc.update_dt), jc.update_ref_pos)
    rospy.spin()

