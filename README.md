# 🗑️ LiDAR 기반 구역 호출형 자율주행 스마트 쓰레기통 (캡스톤 디자인 동상 수상)

> STM32F411 펌웨어 레포지토리 | 전체 시스템: RPi 4B + STM32 + RPLiDAR C1

## 프로젝트 개요

Blynk 앱에서 구역(A/B/C)을 호출하면 로봇이 자율주행으로 해당 구역까지 이동하고, 도착 시 STM32가 서보모터로 뚜껑을 자동 개폐한 뒤 홈 위치로 복귀하는 **스마트 쓰레기통 로봇**이다.

- 동적 장애물 회피 포함 시연 성공
- 캡스톤 디자인 **동상 수상**
- 지도교수 극찬

## 시연 영상

[![시연 영상 1](https://img.youtube.com/vi/nU25p0-28sU/maxresdefault.jpg)](https://youtu.be/nU25p0-28sU)
[![시연 영상 2](https://img.youtube.com/vi/Q-DqqO-98y0/maxresdefault.jpg)](https://youtu.be/Q-DqqO-98y0)

## 시스템 아키텍처

```
[Blynk 앱]
    ↓ MQTT
[Raspberry Pi 4B]  ←→  RPLiDAR C1
  ROS2 / Nav2
  brain.cpp (상태머신)
  motor.cpp (역기구학 + UART)
  odom.cpp  (엔코더 오도메트리)
    ↓ UART1 (115200 bps)
[STM32F411 Nucleo]
  듀얼 모터 PID 제어
  엔코더 피드백
  서보 뚜껑 개폐
    ↓ PWM
[MDD10A 모터드라이버]
    ↓
[JGB37-520 엔코더 모터 × 2]
```

## 하드웨어 스펙

| 항목 | 사양 |
|------|------|
| MCU | STM32F411RE (Nucleo-64) |
| 모터 | JGB37-520 (12V, 감속비 1/180, 13PPR) |
| 모터드라이버 | MDD10A |
| 서보 | MG995 (뚜껑 개폐) |
| 상위 컴퓨터 | Raspberry Pi 4B |
| LiDAR | RPLiDAR C1 |

## STM32 펌웨어 주요 기능

### 1. 듀얼 모터 PI 제어 (50ms 주기)
- TIM10 인터럽트 (50ms) 기반 주기적 제어 루프
- 좌/우 독립 PI 제어기 (Kp=2.2, Ki=2.05)
- 쿼드라처 엔코더 4체배 모드: 13 PPR × 180(감속비) × 4 = **9,360 counts/rev**
- I항 windup 방지 (±2000 클램핑)
- PWM 출력 0~999 클램핑 후 MDD10A에 인가

### 2. UART 통신 프로토콜 (RPi ↔ STM32)
```
RPi → STM32:  "M{L_rpm},{R_rpm}\n"     예) "M30.50,-25.30\n"
STM32 → RPi:  "F{L_rpm},{R_rpm},{tick}\n"  예) "F29.83,-24.91,102\n"
```
- 인터럽트 기반 1바이트씩 수신 → 버퍼 파싱 → `sscanf`로 추출
- **500ms 명령 타임아웃**: 마지막 명령 수신 후 500ms 초과 시 자동 정지

### 3. 역기구학 (RPi 측 motor.cpp)
RPi의 `motor.cpp`에서 목표 선속도(v)·각속도(ω)를 각 바퀴 RPM으로 변환하여 STM32에 전송:
```
RPM_L = (v - ω × L/2) × (60 / (π × d))
RPM_R = (v + ω × L/2) × (60 / (π × d))
```

## 주요 타이머 자원 배분

| 타이머 | 용도 |
|--------|------|
| TIM1 CH1/CH4 | 좌/우 모터 PWM 출력 |
| TIM3 CH1/CH2 | 좌 엔코더 쿼드라처 입력 |
| TIM4 CH1/CH2 | 우 엔코더 쿼드라처 입력 |
| TIM10 | 50ms 제어 주기 인터럽트 |

## 소프트웨어 스택 (전체 시스템)

| 계층 | 기술 |
|------|------|
| 자율주행 | ROS2 Humble, Nav2, SLAM Toolbox |
| 경로계획기 | MPPI Planner + RotationShimController |
| 통신 미들웨어 | CycloneDDS (WSL ↔ RPi via Tailscale) |
| IoT 연동 | Blynk IoT (MQTT) |
| 펌웨어 | STM32 HAL, C |

## 담당 역할 (SW 전담)

- **STM32 펌웨어**: UART 파싱, PI 제어, 엔코더(TIM3/TIM4), 50ms 타이머 인터럽트
- **ROS2 노드 설계·구현**: `motor.cpp` (역기구학 + UART), `odom.cpp` (엔코더 오도메트리), `brain.cpp` (IDLE/MOVING/ARRIVED/RETURNING 상태머신)
- **SLAM Toolbox** 맵 생성 + Nav2 파라미터 튜닝
- **Blynk MQTT** 연동 (구역 호출 → brain 상태머신 트리거)
- **CycloneDDS** 크로스머신 통신 구성 (WSL ↔ RPi via Tailscale)

## 파일 구조

```
capstone/
├── Core/
│   ├── Inc/
│   │   └── main.h          # 핀 정의 (모터 DIR, 액추에이터 등)
│   └── Src/
│       └── main.c          # 메인 루프, PID 제어, UART 파싱, 타이머 콜백
├── Drivers/                # STM32 HAL 드라이버 (CubeMX 자동생성)
└── capstone.ioc           