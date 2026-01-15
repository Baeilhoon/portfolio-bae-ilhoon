# 💻 기술 스택 (Technical Stack)

## 🏗️ 개발 환경 & 플랫폼

### Operating System
```
Primary:    Ubuntu 22.04 LTS (개발 환경)
Additional: Windows 10/11 (도구, 작업)
            Raspbian (Raspberry Pi)
```

### ROS2 & Robotics
```
ROS2 Distribution: Humble
Architecture:      Ubuntu 22.04 + ROS2 Humble
Middleware:        DDS (Data Distribution Service)
Package Manager:   Colcon
```

---

## 📝 프로그래밍 언어

### C++ (ROS2 메인 언어)
**주요 활용**
- ROS2 노드 개발
- 실시간 제어 시스템
- 성능이 중요한 모듈

**키워드**
- Modern C++ (C++17/20)
- 멀티스레드 프로그래밍 (`std::thread`, `std::mutex`)
- 메모리 관리 (`smart pointers`)
- STL 활용 (vector, queue, map)

**경험 프로젝트**
- SLAM 자율주행 로봇 (ROS2 노드)
- 컨베이어 벨트 로봇 제어 (MoveIt2)
- 카메라 데이터 파이프라인

---

### Python (프로토타이핑 & AI)
**주요 활용**
- 데이터 처리 및 분석
- 영상 처리
- AI 모델 추론

**키워드**
- NumPy, Pandas (데이터)
- OpenCV (영상 처리)
- TensorFlow Lite (임베디드 AI)
- Flask (웹 서버)

**경험 프로젝트**
- AI 자동분리수거 쓰레기통 (OpenCV + TFLite)
- ROS2 Python 노드

---

### Arduino C (MCU 펌웨어)
**주요 활용**
- MCU 기반 펌웨어 개발
- 센서 데이터 수집
- 하드웨어 제어

**키워드**
- 아날로그/디지털 I/O
- 시리얼 통신 (UART)
- 인터럽트 처리
- 타이머/카운터

**경험 프로젝트**
- ESP32 MQTT 스마트홈
- 수질 모니터링 시스템
- AI 쓰레기통 (MCU 연동)

---

## 📷 Vision & Image Processing

### OpenCV
```
설치:  C++: opencv-contrib-cpp, Python: opencv-python
버전:  4.5.0 이상
주요 기능:
  - 이미지 읽기/쓰기/디스플레이
  - 필터링 (Gaussian, Bilateral, Morphological)
  - 이미지 변환 (resize, rotate, warp)
  - 객체 인식 (contours, edge detection)
  - 특징 추출 (SIFT, ORB, FAST)
```

**사용 경험**
- SLAM 프로젝트: 카메라 이미지 전처리
- 쓰레기통 프로젝트: 영상 캡처 및 노이즈 제거

### ROS2 Image Transport & cv_bridge
```
목적:  ROS2 메시지 ↔ OpenCV Mat 변환
용도:  - 카메라 데이터 스트리밍
       - 이미지 메시지 압축
       - 멀티센서 동기화

구현:
  #include <cv_bridge/cv_bridge.h>
  #include <sensor_msgs/msg/image.hpp>
  
  cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
```

### TensorFlow Lite (임베디드 AI)
```
목적:  경량 AI 모델 추론 (엣지 디바이스)
환경:  Raspberry Pi, 임베디드 Linux
모델:  YOLO, MobileNet 등 경량 모델

사용 경험:
  - AI 쓰레기통: YOLO 모델 추론
  - 정확도: 92%, 응답 시간: 1.2초
```

---

## 🤖 Robotics & ROS2

### ROS2 Core Concepts
```
Node:        독립적인 실행 단위
Topic:       비동기 Pub/Sub 통신
Service:     동기식 요청/응답
Action:      장시간 작업 (목표, 피드백, 결과)
Message:     데이터 구조 (.msg 파일)
QoS:         메시지 신뢰성 정책
```

### MoveIt2 (로봇 동작 계획)
```
목적:  로봇 팔의 안전한 동작 계획 및 제어
주요 기능:
  - 경로 계획 (경로 최적화)
  - 충돌 감지 (Collision Avoidance)
  - 역기구학 (Inverse Kinematics)
  - 그리퍼 제어

프로젝트: 컨베이어 벨트 로봇 제어
```

### TF2 (Transform Library)
```
목적:  좌표계 변환
용도:  - 카메라 프레임 ↔ 로봇 베이스 프레임
       - 센서 융합 시 데이터 정렬
       - 로봇 팔의 엔드이펙터 위치 계산
```

### DDS (Data Distribution Service)
```
ROS2 기본 미들웨어
QoS 레벨:
  - Reliability: Reliable vs BestEffort
  - Durability: Persistent vs Volatile
  - History: KeepLast vs KeepAll
  
최적화: 
  - 카메라 데이터 → BestEffort (지연 중심)
  - 제어 명령 → Reliable (신뢰성 중심)
```

---

## 🌐 Communication & Networking

### MQTT (IoT 프로토콜)
```
프로토콜:    Pub/Sub 기반 메시징
포트:        1883 (표준), 8883 (TLS)
QoS 레벨:    0 (최대 1회), 1 (최소 1회), 2 (정확히 1회)

프로젝트: ESP32 MQTT 스마트홈
토픽 구조:
  home/sensor/dht22     → 온습도 데이터
  home/control/led      → LED 제어
  home/status/device    → 기기 상태
```

### UART / Serial Communication
```
보율:       9600 bps (표준)
프로토콜:   점-대-점 통신
활용:       MCU ↔ MCU, MCU ↔ 센서

프로젝트: 
  - AI 쓰레기통: Raspberry Pi ↔ Arduino
  - 수질 모니터링: PC ↔ Arduino
```

### TCP/UDP (일반 네트워크)
```
TCP:  신뢰성 중심 (연결 기반)
      - ROS2 분산 시스템
      
UDP:  속도 중심 (비연결성)
      - 실시간 스트림
      - 카메라 데이터
```

---

## 🔧 Build Systems & Tools

### CMake
```
목적:  C++ 프로젝트 빌드 자동화
주요 파일: CMakeLists.txt

ROS2 패키지 빌드:
  cmake_minimum_required(VERSION 3.5)
  project(my_project)
  
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  
  add_executable(my_node src/main.cpp)
  ament_target_dependencies(my_node rclcpp std_msgs)
```

### Colcon (ROS2 빌드 도구)
```
설치: sudo apt install python3-colcon-common-extensions
주요 명령:
  colcon build               # 전체 빌드
  colcon build --packages-select pkg  # 특정 패키지만
  colcon test                # 테스트 실행
  source install/setup.bash  # 환경 설정
```

---

## 🐧 Linux Development

### Command Line Tools
```
파일 관리:    ls, cd, mkdir, cp, mv, rm
텍스트 편집:  nano, vim, VSCode
권한 관리:    chmod, sudo, chown
프로세스:     ps, top, kill, jobs
네트워크:     ping, netstat, ifconfig, ssh
```

### Development Tools
```
컴파일러:     g++, clang++ (C++)
디버거:       gdb (GNU Debugger)
패키지 관리:  apt, apt-get
버전 관리:    git
```

### GDB 디버깅
```
실행:   gdb ./executable
명령:   
  run              - 프로그램 실행
  break main       - 메인에 브레이크포인트
  next/step        - 다음 줄 / 함수 진입
  print variable   - 변수 값 출력
  backtrace        - 스택 트레이스
```

---

## 🔨 Development Tools & IDE

### Visual Studio Code
```
확장:
  C/C++ Extension Pack (Intellisense, 디버깅)
  Python (Python 개발)
  ROS (ROS2 지원)
  CMake Tools (CMake 빌드 지원)
  Git Graph (Git 시각화)

원격 개발:
  Remote - SSH (원격 Linux 서버)
  Remote - Containers (Docker)
```

### Arduino IDE
```
목적:  MCU 펌웨어 개발
보드:  Arduino Mega, ESP32
라이브러리: DHT, OneWire, SD 등
```

### Git & GitHub
```
기본 명령:
  git clone <url>         - 저장소 복제
  git add <file>          - 스테이징
  git commit -m "message" - 커밋
  git push                - 원격 저장소로 업로드
  git pull                - 원격 저장소에서 다운로드

브랜치 관리:
  git branch <name>       - 브랜치 생성
  git checkout <branch>   - 브랜치 전환
  git merge <branch>      - 브랜치 병합
```

---

## 🎯 Development Workflow

### Typical ROS2 Development Cycle

```
1. 워크스페이스 생성
   mkdir -p ros2_ws/src
   cd ros2_ws

2. 패키지 생성
   ros2 pkg create my_package --build-type ament_cmake --dependencies rclcpp

3. 코드 작성 (src/main.cpp)

4. CMakeLists.txt 수정

5. 빌드
   colcon build

6. 환경 설정
   source install/setup.bash

7. 실행
   ros2 run my_package my_node

8. 모니터링
   ros2 topic list
   ros2 topic echo /topic_name
   ros2 node list
```

---

## 📊 역량 레벨 평가

### 매우 능숙 ⭐⭐⭐⭐⭐
- C++ (ROS2 노드 개발)
- Linux Command Line
- ROS2 기본 개념 (Pub/Sub, Node, Topic)
- OpenCV (기초 ~ 중급)

### 능숙 ⭐⭐⭐⭐
- Python (데이터 처리, 프로토타이핑)
- MCU 펌웨어 (Arduino)
- Git & Version Control
- CMake 빌드 시스템

### 기초 ⭐⭐⭐
- MoveIt2 (로봇 동작 계획)
- TensorFlow Lite (AI 추론)
- MQTT 네트워킹
- 분산 ROS2 시스템

### 학습 중 ⭐⭐
- GPU 프로그래밍 (CUDA)
- 딥러닝 모델 커스텀 학습
- Kubernetes (컨테이너 오케스트레이션)

---

## 🚀 향후 학습 계획

### 1순위 (3개월)
- [ ] YOLO 커스텀 모델 학습
- [ ] ROS2 고급 개념 (Action, Service)
- [ ] Gazebo 시뮬레이션

### 2순위 (6개월)
- [ ] 분산 ROS2 시스템 (Multi-robot)
- [ ] GPU 가속화 (CUDA)
- [ ] 클라우드 연동 (AWS, GCP)

### 3순위 (1년)
- [ ] 자율 주행 알고리즘
- [ ] 강화학습 (Reinforcement Learning)
- [ ] Real-time OS (RTOS)

---

## 📚 학습 자료 & 참고 문헌

### 공식 문서
- [ROS2 Documentation](http://docs.ros.org/en/humble/)
- [OpenCV Documentation](https://docs.opencv.org/)
- [Arduino Reference](https://www.arduino.cc/reference/en/)

### 온라인 강좌
- ROS2 Humble Tutorials
- OpenCV Python Tutorials
- TensorFlow Lite for Embedded Linux

### 커뮤니티
- ROS Answers (질문 & 답변)
- Stack Overflow
- GitHub Issues

---

**[← 포트폴리오로 돌아가기](../README.md)**
