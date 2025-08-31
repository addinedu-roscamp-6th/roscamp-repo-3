import os
import time
import threading
from pymycobot.mycobot280 import MyCobot280
from pymycobot.genre import Angle, Coord

mc = MyCobot280('/dev/ttyJETCOBOT', 1000000)
mc.thread_lock = True

coords = mc.get_coords()
coords[0] += 20 # Z를 50mm 내리기
print(f"y축을 {coords[0]}로 내립니다.")
mc.send_coords(coords, 30, 0)
time.sleep(2)

mc.send_angles([0, 0, 0, 0, 0,-30], 50)