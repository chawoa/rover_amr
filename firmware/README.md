# Firmware (STM32 NUCLEO-F401RE + micro-ROS)

## 빌드 준비

micro-ROS 정적 라이브러리(`libmicroros.a`)는 용량 문제로 레포에 포함하지 않습니다.
처음 클론한 뒤 한 번 생성해야 합니다.

```bash
cd firmware
git submodule update --init          # micro_ros_stm32cubemx_utils
```
STM32CubeIDE에서 프로젝트를 빌드하면 pre-build step(Docker)이 라이브러리를 생성합니다.
Docker가 설치되어 있어야 합니다.

## 핀 배치 요약

| 기능 | 타이머/핀 |
|---|---|
| 좌측 엔코더 | TIM3 (Encoder Mode TI1 and TI2, 내부 풀업) |
| 우측 엔코더 | TIM4 (Encoder Mode TI1 and TI2, 내부 풀업) |
| MOTOR_R_IN4 | PB8 (TIM4_CH1과 충돌해 PB6에서 이동) |

## micro-ROS 통신
- 시리얼 115200 baud, Jetson에서 `/dev/ttyACM0`
- 구독: `/cmd_vel`
- 발행: 휠 오도메트리 (Jetson의 `wheel_odom_relay.py`가 EKF 입력으로 변환)
