# Continuous linear decoder node

# 1. Load in .mat decoder file specified by block number, or something
# 2. Pull latest binned neural data from redis
# 3. Apply .mat linear decoder matrix to decoded data
# 4. Send latest decode to redis


# TO DO:
# - Test running this as continuous python script in a loop
# - Do we want to pull just the most recent data, or all of the data since the timestamp
# - Test Matlab decoder build as a command from PC1?
# - Run a CL launch script using VCS


import redis
import scipy.io as sio
import sys
import numpy as np
import time
# import argparse
import json

# parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
# parser.add_argument('--decoderBlock', type=int)
# args = parser.parse_args()
# args = vars(args)   

params = json.load(open('/home/nptl/drone_dev/lib/params.json'))

r = redis.Redis(host='localhost', port=6379)

def main():
	# 1. Load in .mat decoder file specified by block number, or something
	print("Loading decoder from block " + str(params['decoderBlock']))
	file_name = '/home/nptl/Session/Data/Decoders/drone/decoder_block({}).mat'.format(params['decoderBlock'])
	numch = params['numCh']
	binms = params['binMs']
	alpha = params['alpha']

	key = 'drone_latest_decode'

	mat = sio.loadmat(file_name)
	print(mat.keys())

	linFiltDecoder = mat['linFiltDecoder']
	linFiltNeuralMeans = mat['linFiltNeuralMeans']

	starttime = time.time()
	looptime = 0.02

	while (1):
		try:
			# 2. Pull latest binned neural data from redis
			stream = r.xrevrange('binned:neural:stream', '+', '-', count=1)
			
			if stream != []:
				bytesData = stream[0][1][b'data']
				npDecoderOutput = np.frombuffer(bytesData, dtype=np.uint8)

				# 3. Apply .mat linear decoder matrix to decoded data
				prev_xk = np.zeros(4);
				centeredSpikes = npDecoderOutput[0:numch,] - linFiltNeuralMeans.T
				newDecode = np.matmul(centeredSpikes, linFiltDecoder)
				newDecode.shape
				new_xk = alpha * prev_xk + (1-alpha)*newDecode
				new_xk = np.float32(new_xk)

				# 4. Send latest decode to redis stream
				r.xadd('drone:decode:stream', {'vel_x:float32': new_xk[0,0].tobytes(),
				                               'vel_y:float32': new_xk[0,1].tobytes(),
				                               'vel_z:float32': new_xk[0,2].tobytes(),
				                               'vel_r:float32': new_xk[0,3].tobytes()
				                              })

				# store latest decode as a redis key the old way
				# x, y, z, yaw_rate, trash, discrete_click = vel.split()
				msg = str("%.4f"%new_xk[0,0]) + ' ' + str("%.4f"%new_xk[0,1]) + ' ' + str("%.4f"%new_xk[0,2]) + ' ' + str("%.4f"%new_xk[0,3])
				r.set(key, msg)
		except:
			raise

		#stop program when the block is over
		if r.exists('blockOver'):
			r.close()
			print("Block over key found, shutting down.")
			exit()
		else:
			time.sleep(looptime - ((time.time() - starttime) % looptime))
			print(time.time())


if __name__ == "__main__":
	main()
