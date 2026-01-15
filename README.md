# 배일훈 | Embedded Linux & ROS2 Developer

> **Embedded Linux 환경에서 실시간 데이터 처리 시스템을 구축하고, 센서·영상·통신 파이프라인의 안정성을 끝까지 검증하는 신입 개발자**

---

## 🎯 About Me

저는 Ubuntu Linux 환경에서 ROS2 기반 로봇 시스템을 개발하며, 카메라 데이터 수집부터 실시간 영상 처리, 네트워크 통신까지 전체 파이프라인을 C/C++로 구현해왔습니다.

MCU 펌웨어 개발 경험을 통해 하드웨어 수준의 동작 원리를 이해하고 있으며, 이를 바탕으로 Embedded Linux 환경에서도 안정적인 시스템을 설계합니다.

### 핵심 역량
- **Embedded Linux**: Ubuntu 22.04 기반 실시간 애플리케이션 개발
- **ROS2**: DDS 통신, 멀티노드 시스템, 데이터 동기화 최적화
- **영상 처리**: OpenCV, 카메라 제어, 실시간 데이터 파이프라인
- **문제 해결**: 성능 병목 분석, 프로파일링, 최적화 경험

---

## 📚 Major Projects

### 🥇 [1. SLAM 기반 자율주행 로봇](./projects/01-slam-robot.md)
**ROS2 기반 실시간 영상 처리 & 자율주행 시스템**
- 카메라 데이터 파이프라인 개발 (C++)
- 그리퍼 제어 (Arduino → TurtleBot → PC 연동)
- 실시간 영상 처리 및 성능 최적화
- Ubuntu Linux + ROS2 Humble 환경

**기술**: C++, ROS2, OpenCV, RealSense, cv_bridge, Arduino, 그리퍼 제어

---

### 🥈 [2. 멀티모달 LLM 협동로봇 지능제어](./projects/02-multimodal-llm-robot.md)
**Webcam 기반 멀티모달 영상 인식 & 로봇 제어**
- YOLOWorld 기반 Open-vocabulary 물체 인식
- Mediapipe Hand Detector를 활용한 손 제스처 인식
- 영상 인식 + 로봇 제어 통합
- 실시간 멀티모달 상호작용

**기술**: Python, OpenCV, YOLOWorld, Mediapipe, ROS2

---

### 🥉 [3. 디지털트윈 TIAGo 자율배달로봇](./projects/03-tiAGo-digital-twin.md)
**Isaac-Sim 기반 지능형 물류 자동화 시스템**
- YOLOv8n 파인튜닝으로 박스 감지 (94% 정확도)
- Tesseract OCR로 배송 정보 인식 (92% 정확도)
- QR코드 감지 및 디코딩 (99% 정확도)
- Isaac-Sim 디지털트윈 시뮬레이션

**기술**: Python, YOLOv8, Tesseract OCR, pyzbar, Isaac-Sim, ROS2

---

### 4️⃣ [4. 컨베이어 벨트 자동화 공정](./projects/05-conveyor-ros2.md)
**ROS2 기반 로봇 자동화 제어 시스템**
- 실시간 제어 파이프라인 (State Machine)
- 협동로봇(Doosan) 제어
- 멀티스레드 동기화 처리

**기술**: C++, ROS2, MoveIt2, Ubuntu Linux

---

## 🔧 Other Projects

- [4️⃣ AI 자동분리수거 쓰레기통](./projects/04-trash-ai.md) - 임베디드 AI, 하드웨어 제어
- [5️⃣ ESP32 MQTT 스마트홈 제어](./projects/06-smartHome-esp32.md) - IoT 프로토콜, 네트워크 통신
- [6️⃣ 수질 모니터링 시스템](./projects/07-water-monitoring.md) - MCU 센서 데이터 처리

---

## 💻 Technical Skills

### 🐧 Platform & Environment
```
- OS: Ubuntu 22.04 LTS
- ROS2: Humble (DDS, Message Passing)
- Real-Time: Linux Kernel, Thread Management
- Embedded: Raspberry Pi, ARM Architecture
```

### 📝 Languages & Frameworks
```
- C/C++: ROS2 노드, 시스템 프로그래밍, OpenCV 통합
- Python: 영상 처리, AI 추론, 프로토타이핑
- Arduino C: MCU 펌웨어 개발
```

### 📷 Vision & Image Processing
```
- OpenCV: 영상 전처리, 필터링, 객체 인식
- ROS2 Image Transport: 카메라 데이터 스트리밍
- cv_bridge: ROS2 ↔ OpenCV 변환
- TensorFlow Lite: 임베디드 AI 추론
```

### 🤖 Robotics & Control
```
- ROS2 Architecture: Node, Topic, Service, Action
- MoveIt2: 로봇 동작 계획 및 제어
- TF2: 좌표계 변환
- SLAM: 자율주행 및 환경 맵핑
```

### 🌐 Communication & Networking
```
- ROS2 DDS: QoS 설정, 지연 최소화
- MQTT: IoT 프로토콜
- UART/Serial: MCU 통신
- TCP/UDP: 네트워크 통신
```

### 🛠️ Development Tools
```
- Build: CMake, Colcon
- Version Control: Git (협업, 브랜치 전략)
- Debug: GDB, rqt_graph, RViz, rqt_plot
- IDE: VSCode, Arduino IDE
```

**[📄 자세한 기술 스택 보기](./skills/technical-stack.md)**

---

## 🎓 Education & Experience

### 두산 로보틱스 ROKEY 부트캠프 (5기)
**2025.07 - 2026.01 (6개월)**

ROS2 기반 로봇 제어 시스템 개발 집중 교육
- 환경: Ubuntu 22.04, ROS2 Humble, C++/Python
- 성과: 실시간 영상 처리, 협동 로봇 제어, SLAM 기반 자율주행 구현

[📄 자세히 보기](./education/rokey-bootcamp.md)

---

## 📊 역량 평가 (씨프로 기준)

| 요구사항 | 경험 | 평가 |
|---------|------|------|
| Embedded Linux | ROS2 (Ubuntu) | ✅ 우수 |
| C/C++ | ROS2 + MCU | ✅ 우수 |
| 영상 처리 | OpenCV + ROS2 | ✅ 좋음 |
| 네트워크 | MQTT, ROS2 DDS | ✅ 좋음 |
| 실시간 처리 | 센서 + 로봇 제어 | ✅ 우수 |

---

## 🎯 Quick Links

- **[프로젝트 상세](./projects/)** - 각 프로젝트의 구체적인 기술 내용
- **[기술 스택](./skills/technical-stack.md)** - 사용한 도구 및 라이브러리
- **[교육 경력](./education/rokey-bootcamp.md)** - ROKEY 부트캠프 상세 정보

---

**Last Updated**: 2026년 1월 15일
