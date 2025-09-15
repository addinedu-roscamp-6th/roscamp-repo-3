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

def clip_angles(angle_list, min_angle=-135.0, max_angle=135.0):
    """각도 리스트의 값을 -135 ~ 135 범위로 제한"""
    return [max(min(a, max_angle), min_angle) for a in angle_list]

# MyCobot 객체 생성
mc = MyCobot280('/dev/ttyJETCOBOT', 1000000)
mc.thread_lock = True

# 각도 범위 제한을 고려한 안전한 각도 리스트
home_angles = [-0.43, 51.94, 0.43, -52.03, 11.07, -45.61]
target_angles = clip_angles([61.17, -38.67, -0.35, -53.52, -1.05, -49.13])

print("모터 비활성화 상태에서 수동 조작하세요.")
# mc.release_all_servos()
time.sleep(1)

print("s: 현재 좌표 및 조인트 각도 출력")
print("w: 원하는 각도로 이동")
print("q: 모터 활성화 후 기본 자세로 복귀 후 종료")

while True:
    key = get_key()
    
    if key == 's':
        coords = mc.get_coords()
        angles = mc.get_angles()
        print("현재 좌표:", coords)
        print("현재 조인트 각도:", angles)

    elif key == 'w':
        print("모터 활성화 후 원하는 각도로 이동합니다...")
        mc.focus_all_servos()
        time.sleep(1)
        mc.send_angles(target_angles, 30)
        time.sleep(2)
        print("지정한 각도로 이동 완료.")

    elif key == 'q':
        print("모터 활성화 및 복귀 동작 시작...")
        mc.focus_all_servos()
        time.sleep(1)
        mc.send_angles(home_angles, 30)
        time.sleep(2)
        print("기본 자세로 복귀 완료. 프로그램 종료.")
        break

    else:
        print("사용 가능한 키: s(좌표/각도 출력), w(지정 각도 이동), q(복귀 및 종료)")
