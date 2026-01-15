# 📱 ESP32 MQTT 스마트홈 제어 시스템

## 📋 프로젝트 개요

**작업 기간**: 2024.06 - 2024.08  
**프로젝트 분류**: 팀 프로젝트  
**참여 인원**: 4명    
**기여도**: (35%)

### 프로젝트 설명

ESP32 마이크로컨트롤러를 활용한 IoT 스마트홈 시스템입니다. MQTT 프로토콜을 통해 여러 센서 및 제어기를 네트워크로 연결하고, 모바일 앱에서 실시간 제어할 수 있습니다. 이 프로젝트는 **네트워크 통신, 임베디드 시스템, IoT 프로토콜**의 실무적 경험을 제공합니다.

---

## 🎯 구현 내용

### ✅ ESP32 펌웨어 개발

**담당 내용**
- WiFi 연결 및 MQTT 클라이언트 구현
- 센서 데이터 수집 (온습도, 조도, 동작감지)
- 실시간 데이터 MQTT 브로커로 전송

**성과**
- WiFi 연결 안정성: 99.5%
- MQTT 메시지 손실률: 0.2%
- 센서 업데이트 주기: 5초

---

### ✅ IoT 센서 통합

**담당 센서**
- **DHT22**: 온습도 센서
- **LDR**: 조도 센서
- **PIR**: 동작 감지 센서
- **릴레이 모듈**: 전자기기 제어

---

### ✅ MQTT 프로토콜 및 토픽 설계

**토픽 구조**
```
home/
├── sensor/
│   ├── dht22         → {"temp": 25.5, "humidity": 45.2}
│   ├── brightness    → {"value": 78}
│   └── motion        → {"detected": true}
├── control/
│   ├── led           → "ON" or "OFF"
│   ├── fan           → "ON" or "OFF"
│   └── ac            → {"mode": "cool", "temp": 22}
└── status/
    └── device        → {"online": true, "uptime": 3600}
```

**QoS 설정**
- 센서 데이터 (Telemetry): QoS 1 (최소 1회 전달)
- 제어 명령 (Commands): QoS 2 (정확히 1회 전달)

---

## 🛠️ 기술 스택

### 하드웨어
- **MCU**: ESP32 (WiFi + Bluetooth)
- **센서**: DHT22, LDR, PIR, 토양수분센서
- **액추에이터**: 릴레이 모듈, 서보모터
- **전원**: USB 5V

### 소프트웨어
- **펌웨어**: Arduino IDE (C++)
- **프로토콜**: MQTT
- **라이브러리**: WiFi, PubSubClient, DHT, ArduinoJson

### 네트워킹
- **WiFi**: 2.4GHz IEEE 802.11 b/g/n
- **MQTT**: Mosquitto 브로커
- **포트**: 1883 (표준 MQTT)

### 개발 도구
- **IDE**: Arduino IDE, VSCode
- **Debug**: Serial Monitor, MQTT Explorer
- **Version Control**: Git

---

## 📊 시스템 아키텍처

```
┌──────────────────────────────────────────────────────┐
│           WiFi Network (2.4GHz)                      │
│                                                      │
│  ┌─────────────────────┐     ┌──────────────────┐  │
│  │   ESP32 Device 1    │     │  ESP32 Device 2  │  │
│  │  - DHT22 Sensor     │     │  - Motion Sensor │  │
│  │  - LED Control      │     │  - Brightness    │  │
│  └──────────┬──────────┘     └────────┬─────────┘  │
│             │                         │              │
│             └──────────┬──────────────┘              │
│                        │                            │
│                    WiFi                             │
│                        │                            │
└────────────────────────┼────────────────────────────┘
                         │
                    MQTT Broker
                  (Mosquitto)
                    :1883
                         │
        ┌────────────────┼────────────────┐
        │                │                │
        ▼                ▼                ▼
┌────────────────┐  ┌────────────────┐  ┌────────────────┐
│  Mobile App    │  │  Web Dashboard │  │  Home Assistant│
│  (iOS/Android) │  │  (Node-Red)    │  │  Integration   │
└────────────────┘  └────────────────┘  └────────────────┘
```

---

## 📊 성능 지표

| 항목 | 수치 | 평가 |
|------|------|------|
| WiFi 연결 안정성 | 99.5% | ✅ 우수 |
| MQTT 메시지 손실률 | 0.2% | ✅ 양호 |
| 센서 업데이트 주기 | 5초 | ✅ 양호 |
| 제어 응답 시간 | < 500ms | ✅ 양호 |
| 전력 소비 | 150mA (avg) | ✅ 양호 |
| 메모리 사용량 | 220KB / 520KB | ✅ 양호 |

---

## 🎓 배운 점

### 1. 네트워크 통신 프로토콜 (MQTT)
- Pub/Sub 패턴의 원리
- QoS 레벨 선택 및 최적화
- 토픽 설계 및 계층 구조

### 2. IoT 시스템 설계
- 여러 디바이스의 조율
- 센서 데이터 수집 및 전송
- 실시간 제어 로직

### 3. 임베디드 WiFi 프로그래밍
- WiFi 연결 관리
- 안정적인 네트워크 재연결
- 저전력 모드 운영

### 4. 이벤트 기반 프로그래밍
- 콜백 함수 활용
- 비동기 메시지 처리
- 에러 처리 및 복구

---

## 💡 개선 기회

### 단기 개선
- [ ] MQTT 암호화 (TLS/SSL)
- [ ] 디바이스 인증 (인증서)
- [ ] 웹 대시보드 개선

### 장기 개선
- [ ] LoRaWAN 통신 추가
- [ ] 블루투스 메시 네트워크
- [ ] AI 기반 자동화

---

## 🔗 관련 자료

- **GitHub**: [github.com/Baeilhoon/iot-teamproject](https://github.com/Baeilhoon/iot-teamproject)
- **Arduino**: [arduino.cc](https://www.arduino.cc/)
- **MQTT.org**: [mqtt.org](https://mqtt.org/)
- **Mosquitto**: [https://mosquitto.org/]

---

## ✅ 최종 평가

### 강점
✅ MQTT 네트워크 프로토콜 경험  
✅ 멀티센서 통합  
✅ 임베디드 WiFi 프로그래밍  
✅ IoT 시스템 설계 경험  

### 기술적 성과
- 안정적인 네트워크 연결
- 신뢰할 수 있는 메시지 전달
- 확장 가능한 토픽 구조

---

**[← 포트폴리오로 돌아가기](../README.md)**  
**[← 이전 프로젝트](./05-conveyor-ros2.md) | [다음 프로젝트 →](./07-water-monitoring.md)**
