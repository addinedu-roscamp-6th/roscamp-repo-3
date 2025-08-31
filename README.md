# Surpping - 마트 진열 보조 시스템

## Project Scenario
이 프로젝트는 PinkyBot(Logistic Pinky, Mart Pinky), Jetcobot(Load Arm, Unload Arm)을 사용하여 진행했습니다.
Raspberry Pi 기반으로 Camera, LiDAR, Ultrasonic를 활용하여 구현하는 것을 목표로 하였습니다.

Order GUI -> Mian Server -> Robots -> 진열(ArUco Marker Following Driving)

최종 영상 링크 : https://youtube.com/shorts/6FzRB1wkSno?feature=share

## 👥 Team Roles

| 이름 | 역할 | 주요 담당 |
|------|------|-----------|
| 이기준 | 팀장 | Logistic Pinky 알고리즘 제작, Main Server 구축, Socket(TCP, UDP)통신, ROS2 Domain Bridge 통신, Sequence Diagram, PID |
| 양지우 | 팀원 | Load Arm 알고리즘 제작, YOLO 라벨링 및 학습, 맵 설계(3D Design), State Diagram |
| 오희석 | 팀원 | Mart Pinky 알고리즘 제작, System Architecture, PID, 자료 관리 |
| 이재성 | 팀원 | DB 구축, DB TCP 통신 및 저장 |
| 정태규 | 팀원 | Unload Arm 알고리즘 제어 시스템 제작 |
| 정성균 | 팀원 | Unload Arm 알고리즘 물체 기울기 추정 시스템 제작 |
| 강설미 | 팀원 | Order GUI, Monitoring GUI 구축 |

## ⚙️ Tech Stack
- Raspberry Pi(PinkyBot, Jetcobot)
- Ubuntu(24.04) ROS2(Jazzy), OpenCV, YOLO5, PyTorch, Pandas, PyQt
- Python, SQL
- LiDAR, Camera, Ultrasonic, IR Sensor
- Socket(TCP, UDP), ROS2 Domain Bridge(topic)

## 📸 Project Gallery

| ![3D Map Design](https://github.com/addinedu-roscamp-6th/roscamp-repo-3/blob/main/Pictures%20of%20Project/map.png) |
|:---:|
| 맵 설계 |

| ![System Architure](https://github.com/addinedu-roscamp-6th/roscamp-repo-3/blob/main/Pictures%20of%20Project/System%20Architure.png) | ![State Diagram](https://github.com/addinedu-roscamp-6th/roscamp-repo-3/blob/main/Pictures%20of%20Project/State%20Diagram.png) | ![Sequence Diagram](https://github.com/addinedu-roscamp-6th/roscamp-repo-3/blob/main/Pictures%20of%20Project/Sequence%20Diagram.png) |
|:---:|:---:|:---:|
| System Architure | State Diagram | Sequence Diagram |


## 🏆 Expected Outcomes
- 자율주행 기술 시연(차선 인식 및 장애물 회피) -> Logistic Pinky : Camera, Mark Pinky : IR
- 사람을 따라가는 자율주행 기술 구현(ArUco Marker)
- YOLO5를 사용하여 물체 정확히 인식 및 판단
- 2D -> 3D로 좌표계 변환(Jetcobot)
- 팀 협업 능력 및 프로젝트 관리 역량 강화
