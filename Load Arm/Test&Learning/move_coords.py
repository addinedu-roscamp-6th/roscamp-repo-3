import os
import time
import threading
from pymycobot.mycobot280 import MyCobot280
from pymycobot.genre import Angle, Coord
import numpy as np

mc = MyCobot280('/dev/ttyJETCOBOT', 1000000)
mc.thread_lock = True

mc.send_angles([-0.43, 51.94, 0.43, -52.03, 11.07, -45.61], 50)
time.sleep(1)

base_coords = mc.get_coords()

goal_coords = [ 64.59368502, -40.81160064, 348.20348858]

goal_coords = np.hstack([goal_coords, base_coords[3:]]).tolist()

mc.send_coords(goal_coords, 50, 1)
time.sleep(2)
mc.send_angles([-0.43, 51.94, 0.43, -52.03, 11.07, -45.61], 50)
