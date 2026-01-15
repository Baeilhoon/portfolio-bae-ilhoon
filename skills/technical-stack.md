# 💻 기술 스택 (Technical Stack)

## 🏗️ 개발 환경 & 플랫폼

### Operating System
```
Primary:    Ubuntu 22.04 LTS (임베디드 Linux, 시스템 프로그래밍)
Additional: Windows 10/11 (개발 도구, 빌드)
            Raspbian (Raspberry Pi, SBC)
```

### 임베디드 플랫폼
```
MCU:        ESP32, Arduino (IoT, 센서 시스템)
SBC:        Raspberry Pi (Embedded Linux + AI)
Arch:       ARM (Cortex-M, Cortex-A)
```

---

## 📝 프로그래밍 언어

### C++ 
**주요 활용**
- Embedded Linux 시스템 프로그래밍
- 실시간 데이터 처리 및 신호 처리
- 성능 최적화, 메모리 관리

**키워드**
- Modern C++ (C++17/20)
- 멀티스레드 프로그래밍 (`std::thread`, `std::mutex`, `std::condition_variable`)
- 메모리 관리 (`smart pointers`, RAII 패턴)
- STL 활용 (vector, queue, map, deque)
- 저수준 API (포인터, 메모리 레이아웃, 바이트 조작)

**경험 프로젝트**
- SLAM 로봇: 멀티스레드 데이터 동기화, 성능 최적화
- 쓰레기통: Raspberry Pi 기반 실시간 처리
- 컨베이어 벨트: 실시간 상태 머신 제어

---

### Python 
**주요 활용**
- 임베디드 시스템 테스트 및 자동화
- 데이터 분석 및 로깅
- 프로토타이핑

**키워드**
- 시스템 레벨: subprocess, serial, threading
- 데이터 처리: NumPy, Pandas
- 응용: OpenCV (이미지 처리), TensorFlow Lite (경량 AI)

**경험 프로젝트**
- AI 쓰레기통: Raspberry Pi 기반 영상 처리
- SLAM 로봇: 센서 데이터 수집 및 분석

---

### Arduino C
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

## � Hardware Interface & Low-Level Programming

### GPIO & Digital I/O
```
개념:  
  - GPIO (General Purpose Input/Output) 제어
  - 디지털 신호 읽기/쓰기
  - 극성 제어 (HIGH/LOW)

구현 (Arduino):
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  int reading = digitalRead(BUTTON_PIN);

구현 (Linux - sysfs):
  echo 17 > /sys/class/gpio/export
  echo out > /sys/class/gpio/gpio17/direction
  echo 1 > /sys/class/gpio/gpio17/value

프로젝트: ESP32 LED 제어, 릴레이 제어
```

### UART / Serial Communication
```
프로토콜:  비동기 시리얼 통신
보율:      9600, 115200 bps
응용:      MCU ↔ MCU, MCU ↔ PC 데이터 송수신

Arduino 구현:
  Serial.begin(115200);
  Serial.write(data, length);
  int data = Serial.read();

Linux C 구현:
  int fd = open("/dev/ttyUSB0", O_RDWR);
  tcgetattr(fd, &tio);
  tio.c_cflag = B115200;

프로젝트: 
  - 수질 모니터링: PC ↔ Arduino
  - AI 쓰레기통: Raspberry Pi ↔ Arduino
```

### I2C (Inter-Integrated Circuit)
```
프로토콜:  동기식 양방향 통신 (클록 + 데이터)
속도:      100kHz (standard), 400kHz (fast)
용도:      센서, LCD, EEPROM 등 주변 장치

Arduino 구현 (Wire 라이브러리):
  Wire.begin();
  Wire.beginTransmission(0x48);  // 7-bit slave address
  Wire.write(register_addr);
  Wire.write(value);
  Wire.endTransmission();
  
  Wire.requestFrom(0x48, 2);
  byte data1 = Wire.read();
  byte data2 = Wire.read();

Linux C 구현:
  int fd = open("/dev/i2c-1", O_RDWR);
  ioctl(fd, I2C_SLAVE, 0x48);
  write(fd, &buffer, 1);
  read(fd, &buffer, 1);

프로젝트:
  - 수질 모니터링: 온습도 센서 (DHT), pH 센서
  - 스마트홈: 조도 센서
```

### SPI
```
프로토콜:  동기식 4선 통신 (MOSI, MISO, SCK, CS)
속도:      1MHz ~ 10MHz 이상
용도:      SD 카드, 고속 센서, 디스플레이

Arduino 구현 (SPI 라이브러리):
  SPI.begin();
  digitalWrite(CS_PIN, LOW);
  byte result = SPI.transfer(data);
  digitalWrite(CS_PIN, HIGH);

Linux C 구현:
  int fd = open("/dev/spidev0.0", O_RDWR);
  spi_transfer(fd, tx_buffer, rx_buffer, length);
```

### ADC
```
목적:  아날로그 신호를 디지털 값으로 변환
분해능: 8-bit ~ 16-bit
응용:  온습도 센서, 조도 센서, 음성 신호 등

Arduino 구현:
  int value = analogRead(A0);  // 0-1023
  
선형화 (보정):
  voltage = (value / 1023.0) * 5.0;
  temperature = (voltage - 0.5) * 100;

프로젝트: 센서 데이터 수집
```

---

## 📡 Communication Protocols & Networking

### MQTT
```
프로토콜:    경량 Pub/Sub 메시징
포트:        1883 (표준), 8883 (TLS)
QoS 레벨:    
  0: 최대 1회 (at most once)
  1: 최소 1회 (at least once)
  2: 정확히 1회 (exactly once)

ESP32 구현 (Arduino):
  #include <PubSubClient.h>
  
  PubSubClient client(espClient);
  client.setServer("broker.example.com", 1883);
  
  client.publish("home/sensor/temp", "25.5");
  client.subscribe("home/control/led");

프로젝트: ESP32 MQTT 스마트홈 제어
```

### DDS
```
용도:  멀티프로세스 실시간 통신
QoS:   신뢰성(Reliability), 지연시간(Deadline) 등
응용:  ROS2 노드 간 통신 (응용 사례)
```

### TCP/UDP 
```
TCP:  신뢰성 중심 (연결 기반, 손실 없음)
UDP:  속도 중심 (비연결, 손실 가능)

Linux C 구현:
  socket(), connect(), send(), recv()
  bind(), listen(), accept()
```

---

## 💾 Linux System Programming

### 프로세스 및 멀티태스킹
```
개념:
  - fork(): 자식 프로세스 생성
  - execve(): 새 프로그램 실행
  - wait(): 자식 프로세스 종료 대기
  - exit(): 프로세스 종료

예제:
  pid_t pid = fork();
  if (pid == 0) {
    // 자식 프로세스
    execve("./program", args, env);
  } else {
    // 부모 프로세스
    waitpid(pid, &status, 0);
  }
```

### 메모리 관리
```
malloc/free: 동적 메모리 할당/해제
mmap():      파일을 메모리에 매핑
메모리 누수:  valgrind로 검출

예제:
  int *ptr = (int *)malloc(sizeof(int) * 100);
  if (!ptr) perror("malloc");
  free(ptr);
```

### 파일 I/O
```
open/close:  파일 열기/닫기
read/write:  데이터 읽기/쓰기
lseek():     파일 포인터 이동
ioctl():     하드웨어 제어

임베디드 응용:
  - /dev/ttyUSB0: 시리얼 포트
  - /dev/i2c-*: I2C 장치
  - /sys/class/gpio: GPIO 제어
```

### 멀티스레딩
```
pthread: POSIX 스레드 라이브러리
동기화: mutex, condition_variable, semaphore

예제:
  pthread_t thread;
  pthread_create(&thread, NULL, thread_func, NULL);
  pthread_join(thread, NULL);
  
  pthread_mutex_lock(&mutex);
  // critical section
  pthread_mutex_unlock(&mutex);
```

---

## 🔧 Build & Compilation

### Makefile
```
목적:  C/C++ 프로젝트 빌드 자동화 (간단한 프로젝트)

기본 구조:
  CC = gcc
  CFLAGS = -Wall -O2
  
  program: main.o utils.o
    $(CC) -o program main.o utils.o
  
  main.o: main.c
    $(CC) $(CFLAGS) -c main.c
  
  clean:
    rm -f *.o program

프로젝트: 센서 데이터 처리 프로그램
```

### CMake (중규모 프로젝트)
```
목적:  크로스플랫폼 빌드 도구

CMakeLists.txt:
  cmake_minimum_required(VERSION 3.5)
  project(my_project)
  
  add_executable(program main.cpp utils.cpp)
  
  # 라이브러리 링크
  target_link_libraries(program pthread m)

빌드 방법:
  mkdir build && cd build
  cmake ..
  make
```

### Colcon (ROS2 프로젝트)
```
설치: sudo apt install python3-colcon-common-extensions

빌드:
  colcon build
  colcon build --packages-select my_package
  
테스트:
  colcon test
  colcon test-result --verbose

환경 설정:
  source install/setup.bash
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
  break file.c:10  - 특정 줄에 브레이크포인트
  next/step        - 다음 줄 / 함수 진입
  print variable   - 변수 값 출력
  watch variable   - 변수 값 변화 감시
  backtrace        - 스택 트레이스
  info threads     - 스레드 정보
  thread 1         - 스레드 전환

원격 디버깅:
  gdb-server target:port
  (gdb) target remote host:port
```

### Serial Monitor
```
Arduino IDE 내장
또는 minicom, picocom 사용

minicom 사용:
  minicom -D /dev/ttyUSB0 -b 115200
  
데이터 송수신 확인용
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

---

## 🎯 Development Workflow

### 임베디드 시스템 개발 사이클

```
1. 요구사항 분석
   - 하드웨어 사양 검토
   - 성능 요구사항 (레이턴시, CPU, 메모리)
   - 통신 프로토콜 선택

2. 하드웨어 설계
   - 센서/액추에이터 선정
   - 통신 인터페이스 결정 (UART, I2C, SPI, WiFi)
   - 전원 설계

3. 펌웨어 개발
   MCU:
   - Arduino IDE/PlatformIO로 개발
   - 센서 라이브러리 통합
   - 테스트 (하드웨어 직결)
   
   Embedded Linux:
   - Ubuntu 환경에서 개발
   - Cross-compilation (ARM)
   - 타겟 보드에 배포

4. 통신 프로토콜 구현
   - 센서 데이터 수집
   - 시리얼/MQTT/DDS 통신
   - 오류 처리

5. 성능 최적화
   - 프로파일링 (성능 병목 분석)
   - 메모리 최적화
   - 레이턴시 감소

6. 테스트 및 검증
   - 단위 테스트 (Unit Test)
   - 통합 테스트 (Integration Test)
   - 스트레스 테스트

7. 배포 및 유지보수
   - 펌웨어 업데이트 자동화
   - 모니터링 및 로깅
   - 버그 픽스
```

---

## 📊 역량 레벨 평가

### 능숙 ⭐⭐⭐⭐
- **C/C++** (저수준 프로그래밍, 멀티스레드, 실시간)
- **MCU 펌웨어** (Arduino, ESP32)
- **임베디드 Linux** (시스템 프로그래밍)
- **Hardware Interface** (GPIO, UART, I2C, SPI)

### 능숙 ⭐⭐⭐⭐
- **Python** (시스템 테스트, 데이터 처리)
- **MQTT** (IoT 통신)
- **Git & Version Control**
- **CMake/Makefile** (빌드 시스템)

### 기초 ⭐⭐⭐
- **OpenCV** (이미지 처리, 응용)
- **TensorFlow Lite** (경량 AI, 엣지 디바이스)
- **ROS2** (분산 시스템, 응용)
- **DDS** (실시간 통신 미들웨어)

### 학습 중 ⭐⭐
- **RTOS** (FreeRTOS, µC/OS)
- **Device Driver** (Linux kernel)
- **Bluetooth/LoRaWAN** (무선 통신)

---

**[← 포트폴리오로 돌아가기](../README.md)**
