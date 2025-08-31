import os
import time
import threading
from pymycobot.mycobot280 import MyCobot280
from pymycobot.genre import Angle, Coord

mc = MyCobot280('/dev/ttyJETCOBOT', 1000000)
mc.thread_lock = True

mc.send_angles([-0.43, 51.94, 0.43, -52.03, 11.07, -45.61], 50)
coords = mc.get_coords()
print("현재 좌표:", coords)

mc.set_gripper_value(100, 50)

time.sleep(2)

#coords[1] += 30
#mc.send_coords(coords, 30, 1)
#oords = mc.get_coords()
#print(" 이동된 좌표:", coords)
#time.sleep(2)

#mc.send_angles([-0.43, 51.94, 0.43, -52.03, 11.07, -45.61], 50)
