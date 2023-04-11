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

info = r.xinfo_stream('drone:decode:stream')
newMsgId = info['first-entry'][0].decode('utf-8')

target_info = r.xinfo_stream('drone:stream')
newMsgId_target = info['first-entry'][0].decode('utf-8')
#print("First message ID: " + newMsgId)

print("running...")

decoded_data = []
target_positions = []
drone_positions = []

while(1):

    # XREAD: Blocking request which waits on a stream for 'block' ms maximum
    #stream = r.xread({'drone:decode:stream': '$'}, count=1, block=1000)    
    stream = r.xread({'drone:decode:stream': newMsgId}, count=1, block=1000)
    target_stream = r.xread({'drone:stream': newMsgId_target}, count=1, block=1000)
    
    # XREVRANGE: Non-blocking request for a range of entries
    # The returned stream is a list of entries of length 'count', where each entry contains a tuple of (log time (ms), dict of key:value pairs)  
    # stream = r.xrevrange('binned:neural:stream', '+', '-', count=10)
    #newMsgId = stream[0][1][0][0].decode('utf-8')

    if not (stream == [] or target_stream == []):

        newMsgId = stream[0][1][0][0].decode('utf-8')
        newMsgId_target = target_stream[0][1][0][0].decode('utf-8')

        target_pos = target_stream[0][1][0][1][b'target_pos:float32']
        drone_pos = target_stream[0][1][0][1][b'drone_pos:float32']
        target_pos = np.frombuffer(target_pos, dtype=np.float32)
        drone_pos = np.frombuffer(drone_pos, dtype=np.float32)
        target_positions.append(target_pos)
        drone_positions.append(drone_pos)

        # XREAD
        #print(stream[0][1][0][1])
        vx = stream[0][1][0][1][b'vel_x:float32']
        vy = stream[0][1][0][1][b'vel_y:float32']
        vz = stream[0][1][0][1][b'vel_z:float32']
        vr = stream[0][1][0][1][b'vel_r:float32']
        
        vx = np.frombuffer(vx, dtype=np.float32)
        vy = np.frombuffer(vy, dtype=np.float32)
        vz = np.frombuffer(vz, dtype=np.float32)
        vr = np.frombuffer(vr, dtype=np.float32)

        print(f'{newMsgId}: {vx} {vy} {vz} {vr}')
        decoded_data.append([vx, vy, vz, vr])
        #print("processing")
    else:
        break

#print("Last message ID: " + newMsgId)
decoded_data = np.array(decoded_data)
target_positions = np.array(target_positions)
drone_positions = np.array(drone_positions)
np.savez('decoded_data.npz', decoded_data=decoded_data)
np.savez('target_positions.npz', target_positions=target_positions)
np.savez('drone_positions.npz', drone_positions=drone_positions)

# Plot the decoded data
import matplotlib.pyplot as plt

fig, axs = plt.subplots(4, 1)
axs[0].plot(decoded_data[:,0])
axs[0].set_title('Velocity X')
axs[1].plot(decoded_data[:,1])
axs[1].set_title('Velocity Y')
axs[2].plot(decoded_data[:,2])
axs[2].set_title('Velocity Z')
axs[3].plot(decoded_data[:,3])
axs[3].set_title('Velocity R')
plt.show()

    