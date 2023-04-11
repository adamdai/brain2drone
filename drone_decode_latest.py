'''
Example for receiving and parsing Redis binned neural stream and converting to a numpy array.
XREAD waits for the latest count of entries, blocking the stream from other Redis clients.
XRANGE gets the latest count of entries, non-blocking read.

Author: DA
Last Edited: 8/10/21
'''

import redis
import sys
import numpy as np

print("starting Redis connection")
r = redis.Redis(host='localhost', port=6379, db=0)

print("running...")

while(1):

    vel = r.get('drone_latest_decode') 
    x, y, z, yaw_rate = vel.split() # encoded as a string, values separated by spaces
    print(vel)