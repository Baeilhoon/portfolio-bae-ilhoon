# 배일훈 | Embedded Software Developer

> **Embedded Linux 기반 실시간 시스템을 설계·구현하고, 센서·통신·하드웨어 제어 파이프라인의 안정성을 검증하는 신입 임베디드 소프트웨어 개발자**

---

## 🎯 About Me

저는 Ubuntu Linux 환경에서 C/C++를 사용한 저수준 프로그래밍으로 실시간 데이터 처리 시스템을 구축해왔습니다. 센서 데이터 수집부터 신호 처리, 네트워크 통신까지 전체 파이프라인의 성능 최적화와 안정성 검증에 집중합니다.

MCU 펌웨어 개발 경험(ESP32, Arduino)을 통해 하드웨어 수준의 동작 원리를 깊이 있게 이해하고 있으며, 이를 바탕으로 Embedded Linux 시스템과 MCU 기반 IoT를 통합한 안정적인 솔루션을 설계합니다.

### 핵심 역량
- **Embedded Linux**: Ubuntu 22.04 기반 실시간 애플리케이션, 시스템 프로그래밍, 성능 최적화
- **임베디드 C/C++**: 저수준 프로그래밍, 메모리 관리, 멀티스레드 동기화
- **MCU 펌웨어**: ESP32, Arduino 기반 센서 제어 및 통신 시스템
- **네트워크 통신**: MQTT, DDS, UART/Serial, TCP/UDP 프로토콜
- **실시간 시스템**: 성능 병목 분석, 프로파일링, 최적화 경험

---

## 📚 Major Projects

### 🥇 [1. ESP32 MQTT 스마트홈 제어 시스템](./projects/06-smartHome-esp32.md)
**IoT 네트워크 기반 임베디드 제어 시스템**
- ESP32 MCU 펌웨어 개발 (Arduino C)
- WiFi 통신 및 MQTT 프로토콜 구현
- 센서 데이터 수집 및 실시간 전송
- 모바일 앱과 하드웨어 통신 통합

**기술**: C, Arduino, ESP32, MQTT, WiFi, 센서 제어

---

### 🥈 [2. 수질 모니터링 시스템](./projects/07-water-monitoring.md)
**MCU 기반 센서 데이터 처리 & 로깅 시스템**
- 아날로그/디지털 센서 제어 (온습도, pH, 유량 등)
- 시리얼 통신 기반 데이터 수집
- 메모리 최적화 및 로깅 시스템 구현

**기술**: Arduino C, UART, 센서 프로토콜, 데이터 처리

---

### 🥉 [3. AI 자동분리수거 쓰레기통](./projects/04-trash-ai.md)
**Embedded Linux + AI 기반 하드웨어 제어 시스템**
- Raspberry Pi 기반 임베디드 시스템
- 카메라 제어 및 실시간 영상 처리 (Python + OpenCV)
- TensorFlow Lite를 사용한 경량 AI 추론
- MCU와의 통신으로 액추에이터 제어

**기술**: Python, C, OpenCV, TensorFlow Lite, Raspberry Pi, GPIO 제어

---

### 4️⃣ [4. SLAM 기반 자율주행 로봇](./projects/01-slam-robot.md)
**Embedded Linux 기반 실시간 제어 시스템** *(응용 사례)*
- ROS2 + Linux 환경의 멀티프로세스 시스템 설계
- 카메라 데이터 파이프라인 (C++, cv_bridge)
- 아두이노와 PC 간 실시간 통신 및 동기화
- 성능 최적화 및 레이턴시 개선

**기술**: C++, ROS2, OpenCV, Arduino, 실시간 동기화

---

## 🔧 Advanced Projects *(ROS2 응용 경험)*

- [멀티모달 LLM 협동로봇 지능제어](./projects/02-multimodal-llm-robot.md) - Embedded Linux + ROS2 기반 실시간 영상 인식
- [디지털트윈 TIAGo 자율배달로봇](./projects/03-tiAGo-digital-twin.md) - Embedded 시뮬레이션 환경
- [컨베이어 벨트 자동화 공정](./projects/05-conveyor-ros2.md) - ROS2 기반 실시간 제어 시스템

---

## 💻 Technical Skills

### 🐧 Platform & Environment
```
- OS: Ubuntu 22.04 LTS (시스템 프로그래밍, 실시간 애플리케이션)
- Embedded Systems: Raspberry Pi, ARM Architecture, STM32
- MCU: ESP32, Arduino (Firmware Development)
- Communication: MQTT, DDS, UART, I2C, SPI
```

### 📝 Languages & Frameworks
```
- C/C++: 저수준 프로그래밍, 멀티스레드, 메모리 관리, ROS2
- Arduino C: MCU 펌웨어 개발, GPIO 제어, 센서 통신
- Python: 데이터 처리, 시스템 테스트, 프로토타이핑
```

### 🔌 Hardware & Embedded
```
- GPIO 제어: Digital/Analog I/O
- 센서 인터페이스: UART, I2C, SPI, ADC
- 시리얼 통신: 직렬 통신 프로토콜, 데이터 동기화
- MCU 펌웨어: 인터럽트 처리, 타이머 제어, 메모리 최적화
```

### 📊 Real-Time & Performance
```
- 성능 최적화: Profiling, Bottleneck 분석
- 멀티스레드: std::thread, std::mutex, 동기화 메커니즘
- 실시간 처리: 레이턴시 < 50ms, 안정적인 프레임레이트
- 메모리 관리: Smart pointers, 메모리 누수 방지
```

### 🌐 Networking & Protocol
```
- IoT: MQTT (Publish/Subscribe), QoS 설정
- 중간웨어: ROS2 DDS, Pub/Sub 패턴
- Serial 통신: UART, 프로토콜 설계
- 신뢰성: 오류 처리, 재연결 메커니즘
```

### 🛠️ Development Tools
```
- Build: CMake, Colcon, Makefile
- Version Control: Git (협업, 브랜치 전략)
- Debug: GDB, Serial Monitor, Oscilloscope
- IDE: VSCode, Arduino IDE, STM32 CubeIDE
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

## 📊 역량 평가

| 분야 | 경험 | 평가 |
|------|------|------|
| Embedded Linux | Ubuntu 시스템 프로그래밍, 실시간 애플리케이션 | ✅ 우수 |
| C/C++ | 저수준 프로그래밍, 멀티스레드, 성능 최적화 | ✅ 우수 |
| MCU 펌웨어 | ESP32, Arduino 센서/통신 시스템 | ✅ 우수 |
| 네트워크 통신 | MQTT, DDS, UART, Serial 프로토콜 | ✅ 좋음 |
| 실시간 시스템 | 성능 분석, 병목 최적화, 동기화 | ✅ 우수 |
| ROS2 (응용) | 멀티프로세스 실시간 시스템 | ✅ 좋음 |

---

## 🎯 Quick Links

- **[프로젝트 상세](./projects/)** - 각 프로젝트의 구체적인 기술 내용
- **[기술 스택](./skills/technical-stack.md)** - 사용한 도구 및 라이브러리
- **[교육 경력](./education/rokey-bootcamp.md)** - ROKEY 부트캠프 상세 정보

---
