import os
import time
import threading
from pymycobot.mycobot280 import MyCobot280
from pymycobot.genre import Angle, Coord

mc = MyCobot280('/dev/ttyJETCOBOT', 1000000)
mc.thread_lock = True

coords = mc.get_coords()
print("현재 좌표:", coords)

# 현재 각도 읽기
angles = mc.get_angles()
print("현재 각도:", angles)