
import redis
import os
import numpy as np
import datetime

print("connecting to server")
r = redis.Redis(host='localhost', port=6379, db=0)

def main():
    print("running main")

    # Define trajectory
    RATE = 10    # hz 
    DT = 1/RATE  # seconds
    T = 10.0     # seconds
    N = int(T/DT)  # number of points
    tspan = np.linspace(0, T, N)

    # Circle trajectory (centered at (1,0), radius 1)
    V = 0.05
    traj_x = 1 + np.cos(tspan)
    traj_y = np.sin(tspan)
    traj_z = np.ones(N)
    traj_vx = V * -np.sin(tspan)
    traj_vy = V * np.cos(tspan)
    traj_vz = np.zeros(N)
    traj_vr = np.zeros(N)

    for i in range(N):
        vx = np.float32(traj_vx[i])
        vy = np.float32(traj_vy[i])
        vz = np.float32(traj_vz[i])
        vr = np.float32(traj_vr[i])
        print(f"vx: {vx}, vy: {vy}, vz: {vz}, vr: {vr}")

        r.xadd('drone:decode:stream', 
               {'vel_x:float32': vx.tobytes(),
                'vel_y:float32': vy.tobytes(),
                'vel_z:float32': vz.tobytes(),
                'vel_r:float32': vr.tobytes()})
    
    r.save()
    config = r.config_get()
    savePath = os.path.join(config['dir'], config['dbfilename'])
    print("Savepath: ", savePath)
    if not os.path.isdir('/home/navlab/Data'):
        os.makedirs('/home/navlab/Data')
    os.system('cp ' + savePath + ' "/home/navlab/Data/test.rdb"')

if __name__ == "__main__":
	main()