import struct
import time

format = struct.Struct(
	'<' +  # little-endian
	'ff' +        # times
	'fff' * 3  +  # 3-axis readings
	'ff' # pressure, temperature
)

with open('log00.csv', 'w') as csv:
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
	file = open('LOG00.DAT', 'r')
	while True:
		data = file.readline(format.size)
		if data:
			print >> csv, ', '.join(str(f) for f in format.unpack(data))
		else:
			break

# with open('all.raw.log') as src, open('all.log', 'w') as dest:
# 	while True:
# 		data = src.read(format.size)
# 		if data:
# 			print format.unpack(data)