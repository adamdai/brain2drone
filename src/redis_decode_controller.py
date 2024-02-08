#!/usr/bin/env python3

"""
Reads velocity commands off redis stream and publishes to drone

Two modes: "AIRSIM" and "JOY"
"AIRSIM" is for neural decoded control and "JOY" is for joystick control

"""



import rospy
import numpy as np
import redis

from geometry_msgs.msg import Vector3Stamped, PoseStamped
from scipy.spatial.transform import Rotation


#%% =========== Parameters ========== ###

DRONE_NAME = "drone1"

LOCAL_FRAME = True     # Whether decoded velocities are in local frame (as opposed to global)
                       # mavros velocity setpoints are in global frame
JOY_CONTROL = True     # Using joystick or decoder
ENABLE_Z = True
ENABLE_YAW = True

# For flightroom
X_BOUNDS = [-3.0, 6.0]
Y_BOUNDS = [-1.5, 1.5]
Z_BOUNDS = [0.0, 2.5]

# For flightroom
MAX_VEL = 0.5  # m/s
MAX_YAWRATE = 0.78  # rad/s (45 deg/s)

# Scaling parameters (decoder outputs vs. joystick outputs)
AIRSIM_MAX_VEL = 8  # m/s
AIRSIM_MAX_YAWRATE = 20.0  # rad/s
JOY_MAX_VEL = 1.0
JOY_MAX_YAWRATE = 1.0

if JOY_CONTROL:
    v_scale = MAX_VEL / JOY_MAX_VEL
    vr_scale = MAX_YAWRATE / JOY_MAX_YAWRATE
else:
    v_scale = MAX_VEL / AIRSIM_MAX_VEL
    vr_scale = MAX_YAWRATE / AIRSIM_MAX_YAWRATE

RATE = 20    # hz 
DT = 1/RATE  # seconds


#%% =========== Helper Functions ========== ###

def saturate(v: float, v_max: float) -> float:
    if v > v_max:
        return v_max
    elif v < -v_max:
        return -v_max
    else:
        return v
    

def quat_to_R(q):
    r = Rotation.from_quat(q)
    return r.as_matrix()

"""
Takeoff at origin (0, 0, 1)

"""

#%% =========== Main Node ========== ###

class RedisDecodeController:
    def __init__(self):

        rospy.init_node('redis_decode_controller_node', disable_signals=True)
        self.rate = rospy.Rate(RATE)  # GCS rate is 20 Hz
        self.dt = DT  # seconds 

        print("starting Redis connection")
        self.rc = redis.Redis(host='localhost', port=6379, password='H9@zGz4!mKts')
        print("ping: ", self.rc.ping())
        info = self.rc.xinfo_stream('drone:decode:stream')
        self.newMsgId = info['first-entry'][0].decode('utf-8')

        self.curr_pos = np.array([0.0, 0.0, 1.0])
        self.curr_att = np.zeros(4)

        # Publishers
        self.vel_pub = rospy.Publisher(f"{DRONE_NAME}/setpoint/velocity", Vector3Stamped, queue_size=1)
        self.bdr_pub = rospy.Publisher(f"{DRONE_NAME}/setpoint/bodyrate", Vector3Stamped, queue_size=1)

        # Subscribers
        mocap_sub = rospy.Subscriber(f"{DRONE_NAME}/mavros/local_position/pose", PoseStamped, self.mocap_cb)

        self.last_read_time = rospy.Time.now()
        self.dts = []
        self.log = []

        rospy.on_shutdown(self.shutdown)

    
    def mocap_cb(self, data):
        # Unpack Data
        self.curr_pos[0] = data.pose.position.x
        self.curr_pos[1] = data.pose.position.y
        self.curr_pos[2] = data.pose.position.z

        self.curr_att[0] = data.pose.orientation.x
        self.curr_att[1] = data.pose.orientation.y
        self.curr_att[2] = data.pose.orientation.z
        self.curr_att[3] = data.pose.orientation.w

    
    def in_bounds(self):
        return (self.curr_pos[0] >= X_BOUNDS[0] and self.curr_pos[0] <= X_BOUNDS[1] and
                self.curr_pos[1] >= Y_BOUNDS[0] and self.curr_pos[1] <= Y_BOUNDS[1] and
                self.curr_pos[2] >= Z_BOUNDS[0] and self.curr_pos[2] <= Z_BOUNDS[1])
    

    def bounds_state(self):
        return [self.curr_pos[0] <= X_BOUNDS[0], self.curr_pos[0] >= X_BOUNDS[1],
                self.curr_pos[1] <= Y_BOUNDS[0], self.curr_pos[1] >= Y_BOUNDS[1],
                self.curr_pos[2] <= Z_BOUNDS[0], self.curr_pos[2] >= Z_BOUNDS[1]]


    def run(self, event=None):
        # Read off data from decoder outputs
        stream = self.rc.xread({'drone:decode:stream': '$'}, count=1, block=1000)

        # # Log/print time since last read
        # t_now = rospy.Time.now()
        # dt = t_now - self.last_read_time
        # self.dts.append(dt.to_sec())
        # #print(f"dt: {dt.to_sec()}")
        # self.last_read_time = t_now

        # self.newMsgId = stream[0][1][0][0].decode('utf-8')
        vx = stream[0][1][0][1][b'vel_x:float32']
        vy = stream[0][1][0][1][b'vel_y:float32']
        vz = stream[0][1][0][1][b'vel_z:float32']
        vr = stream[0][1][0][1][b'vel_r:float32']
        
        vx = np.frombuffer(vx, dtype=np.float32)[0]
        vy = np.frombuffer(vy, dtype=np.float32)[0]
        vz = np.frombuffer(vz, dtype=np.float32)[0]
        vr = np.frombuffer(vr, dtype=np.float32)[0]

        #print(f"Received vel: , {vx}, {vy}, {vz}, {vr}")

        # Transform for live data: Live data is in AirSim coordinate frame (NED) 
        # whereas joystick control is in flightroom frame (ENU)
        if not JOY_CONTROL:
            vx, vy = vy, -vx
            vz = -vz

        # Local to global transform
        if LOCAL_FRAME:
            # Get pose rotation matrix (local to global)
            R = quat_to_R(self.curr_att)

            # Rotate xyz velocity command from local to global frame
            vxyz_local = np.array([vx, vy, vz])
            vxyz_global = R @ vxyz_local
            vx = vxyz_global[0]
            vy = vxyz_global[1]
            vz = vxyz_global[2]

        # Scale and saturate
        vx = v_scale * vx
        vy = v_scale * vy
        vz = 2 * v_scale * vz
        vr = vr_scale * vr

        vx = saturate(vx, MAX_VEL)
        vy = saturate(vy, MAX_VEL)
        vz = saturate(vz, MAX_VEL)
        vr = saturate(vr, MAX_YAWRATE)

        self.log.append([vx, vy, vz, vr])
        t_now = rospy.Time.now()

        vel_msg = Vector3Stamped()
        bdr_msg = Vector3Stamped()
        vel_msg.header.stamp = t_now
        bdr_msg.header.stamp = t_now

        # Check bounds
        bounds_state = self.bounds_state()
        if bounds_state[0]:
            print("CROSSED X LOWER BOUND")
            vx = max(0, vx)
        elif bounds_state[1]:
            print("CROSSED X UPPER BOUND")
            vx = min(0, vx)
        if bounds_state[2]:
            print("CROSSED Y LOWER BOUND")
            vy = max(0, vy)
        elif bounds_state[3]:
            print("CROSSED Y UPPER BOUND")
            vy = min(0, vy)
        if bounds_state[4]:
            print("CROSSED Z LOWER BOUND")
            vz = max(0, vz)
        elif bounds_state[5]:
            print("CROSSED Z UPPER BOUND")
            vz = min(0, vz)

        # Set velocities
        vel_msg.vector.x = vx
        vel_msg.vector.y = vy

        if ENABLE_Z: 
            vel_msg.vector.z = vz
        else: 
            vel_msg.vector.z = 0.0

        if ENABLE_YAW:
            bdr_msg.vector.z = vr
        else:
            bdr_msg.vector.z = 0.0

        #print(f"sending {np.round(vx, 2)}, {np.round(vy, 2)}, {np.round(vz, 2)}, {np.round(vr, 2)}")
        # else:
        #     print("out of bounds")
        #     vel_msg.vector.x = 0.0
        #     vel_msg.vector.y = 0.0
        #     vel_msg.vector.z = 0.0
        #     bdr_msg.vector.z = 0.0

        print(f"sending:   {np.round(vel_msg.vector.x, 2)},   {np.round(vel_msg.vector.y, 2)},   {np.round(vel_msg.vector.z, 2)},   {np.round(bdr_msg.vector.z, 2)}")

        # Publish
        self.vel_pub.publish(vel_msg)
        self.bdr_pub.publish(bdr_msg)

    def shutdown(self):
        print("Shutting down")
        print(f"Average dt: {np.mean(self.dts)} \n \
               Max dt: {np.max(self.dts)} \n \
               Min dt: {np.min(self.dts)} \n \
               Std dt: {np.std(self.dts)} \n")
        np.save('redis_decode_controller_log.npy', self.log)


if __name__ == '__main__':
    
    rdc = RedisDecodeController()

    rospy.Timer(rospy.Duration(rdc.dt), rdc.run)
    rospy.spin()

