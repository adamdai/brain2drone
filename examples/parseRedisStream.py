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
import redis

print("starting Redis connection")
r = redis.Redis(host='localhost', port=6379, db=0)

print("running...")

while(1):

    # XREAD: Blocking request which waits on a stream for 'block' ms maximum
    stream = r.xread({'binned:neural:stream': '$'}, count=1, block=1000)    
    
    # XREVRANGE: Non-blocking request for a range of entries
    # The returned stream is a list of entries of length 'count', where each entry contains a tuple of (log time (ms), dict of key:value pairs)  
    # stream = r.xrevrange('binned:neural:stream', '+', '-', count=10)

    if not (stream == []):
       
        print(stream[0][1])

        # XREAD
        bytesData = stream[0][1][0][1][b'data']

        # XREVRANGE
        # bytesData = stream[0][1][b'data']

        npBinnedNeural = np.frombuffer(bytesData, dtype=np.uint8)
        # print(npBinnedNeural)