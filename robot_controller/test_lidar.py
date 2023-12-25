#!/usr/bin/env python3
from rplidar import RPLidar
from time import sleep

lidar = RPLidar('/dev/ttyUSB0')
lidar.stop_motor()

sleep(10)
info = lidar.get_info()
print(info)

health = lidar.get_health()
print(health)

for i, scan in enumerate(lidar.iter_scans()):
    print('%d: Got %d measurments' % (i, len(scan)))
    if i > 10:
        break

lidar.stop()
lidar.stop_motor()
lidar.disconnect()

