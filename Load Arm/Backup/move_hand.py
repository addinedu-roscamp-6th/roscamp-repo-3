import os
import time
import threading
from pymycobot.mycobot280 import MyCobot280
from pymycobot.genre import Angle, Coord

import sys
import tty
import termios

def get_key():
    """터미널에서 한 글자씩 입력받기(리눅스, mac)"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

mc = MyCobot280('/dev/ttyJETCOBOT', 1000000)
mc.thread_lock = True

# 기본 자세
home_angles = [-0.43, 51.94, 0.43, -52.03, 11.07, -45.61]

print("모터 비활성화 상태에서 수동 조작하세요.")
mc.release_all_servos()
time.sleep(1)

print("s키: 현재 좌표 출력, q키: 모터 활성화 및 기본 자세로 복귀 후 종료")
while True:
    key = get_key()
    if key == 's':
        coords = mc.get_coords()
        print("현재 좌표:", coords)
    elif key == 'q':
        print("모터 활성화 및 복귀 동작 시작...")
        mc.focus_all_servos()
        time.sleep(1)
        mc.send_angles(home_angles, 50)
        time.sleep(2)
        print("기본 자세로 복귀 완료. 프로그램 종료.")
        break
    else:
        print("s(좌표 출력), q(복귀 및 종료)만 사용 가능.")
