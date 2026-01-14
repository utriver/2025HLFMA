# PD Controller 패키지

이 패키지는 GPS 속도 데이터를 기반으로 한 PD 제어기를 구현합니다.

## 기능

- `/gps/velocity` 토픽에서 속도 데이터를 구독
- PD 제어 알고리즘을 사용하여 제어 신호 생성
- 0~100 범위의 제어 신호를 `/control_signal` 토픽으로 발행

## 제어 구조

```
Target Velocity (+) → Error Sum → PD Controller → Control Signal (0~100)
                 ↑                                      ↓
                 |                                   PWM Signal → Motor
                 |                                      ↓
                 |                                  Actual Speed
                 |                                      ↓
                 ← Feedback (-) ← GPS Sensor ← Measured Current Speed
```

## 사용법

### 1. 패키지 빌드
```bash
cd ~/ros2_ws
colcon build --packages-select pd_control
source install/setup.bash
```

### 2. 노드 실행
```bash
# 단일 노드 실행
ros2 run pd_control pd_controller_node

# Launch 파일로 실행
ros2 launch pd_control pd_controller_launch.py
```

### 3. 토픽 모니터링
```bash
# 제어 신호 확인
ros2 topic echo /control_signal

# 현재 속도 확인
ros2 topic echo /gps/velocity

# 목표 속도 설정
ros2 topic pub /target_velocity std_msgs/Float32 "data: 3.0"
```

## 파라미터

- `kp`: 비례 게인 (기본값: 2.0)
- `kd`: 미분 게인 (기본값: 0.5)
- `target_velocity`: 목표 속도 [m/s] (기본값: 5.0)

## 토픽

### 구독 토픽
- `/gps/velocity` (geometry_msgs/TwistWithCovarianceStamped): GPS 속도 데이터
- `/target_velocity` (std_msgs/Float32): 목표 속도 설정

### 발행 토픽
- `/control_signal` (std_msgs/Float32): PD 제어 신호 (0~100)

## 예제

```bash
# 1. sensor_fusion_node 실행 (GPS 데이터 제공)
ros2 run gps sensor_fusion_node

# 2. PD 제어기 실행
ros2 run pd_control pd_controller_node

# 3. 목표 속도 변경 (3 m/s로 설정)
ros2 topic pub /target_velocity std_msgs/Float32 "data: 3.0"
```
