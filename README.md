# About


# Surpping - 마트 진열 보조 시스템

## Project Scenario
이 프로젝트는 PinkyBot(Logistic Pinky, Mart Pinky), Jetcobot(Load Arm, Unload Arm)을 사용하여 진행했습니다.
Raspberry Pi 기반으로 Camera, LiDAR, Ultrasonic를 활용하여 구현하는 것을 목표로 하였습니다.
Order GUI -> Mian Server -> Robots -> 진열(ArUco Marker Following Driving)

## 👥 Team Roles

| 이름 | 역할 | 주요 담당 |
|------|------|-----------|
| 홍길동 | 팀장 | 전체 일정 관리, 하드웨어 설계 |
| 김철수 | 임베디드 개발 | Raspberry Pi 환경 세팅, 센서 제어(카메라, LiDAR) |
| 이영희 | 제어 알고리즘 | 모터 제어, 주행 알고리즘 설계 |
| 박민수 | 통신 & ROS | ROS 기반 통신, 데이터 처리 |
| 최지훈 | 디자인 & 문서화 | 차량 구조 디자인, 발표 자료 제작 |

## ⚙️ Tech Stack
- Raspberry Pi(PinkyBot, Jetcobot)
- Ubuntu(24.04) ROS2(Jazzy), OpenCV, YOLO5, PyTorch, Pandas, PyQt
- Python, SQL
- LiDAR, Camera, Ultrasonic, IR Sensor
- Socket(TCP, UDP), ROS2 Domain Bridge(topic)

## 🏆 Expected Outcomes
- 자율주행 기술 시연(차선 인식 및 장애물 회피) -> Logistic Pinky : Camera, Mark Pinky : IR
- 사람을 따라가는 자율주행 기술 구현(ArUco Marker)
- YOLO5를 사용하여 물체 정확히 인식 및 판단
- 2D -> 3D로 좌표계 변환(Jetcobot)
- 팀 협업 능력 및 프로젝트 관리 역량 강화
