from serial import Serial
import struct
import time

format = struct.Struct(
	'<' +  # little-endian
	'ff' +        # times
	'fff' * 3  +  # 3-axis readings
	'ff' # pressure, temperature
)

with Serial('/dev/ttyACM0', baudrate=115200) as conn:
	# wait for board to send ready message
	print conn.readline()

	# request file
	conn.write('\n')

	# wait for start of response
	while conn.inWaiting() == 0:
		pass

	conn.timeout = 0.25

	with open('log.dat', 'w') as raw_log, open('log.csv', 'w') as csv:
		print >> csv, ', '.join([
			'Time',
			'Time for loop in ms',
			'Acc x',
			'Acc y',
			'Acc z',
			'Gx Rate',
			'Gy Rate',
			'GzRate',
			'Mx',
			'My',
			'Mz',
			'Temp',
			'Pressure'
		])
		while True:
			data = conn.read(format.size)
			if data:
				raw_log.write(data)
				print >> csv, ', '.join(str(f) for f in format.unpack(data))
			else:
				break

# with open('all.raw.log') as src, open('all.log', 'w') as dest:
# 	while True:
# 		data = src.read(format.size)
# 		if data:
# 			print format.unpack(data)