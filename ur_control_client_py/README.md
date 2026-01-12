# UR Control Client (Python)

Python 기반 UR 로봇 제어 라이브러리 (`URRobotController` 클래스)

## 📋 개요

이 패키지는 UR 로봇을 제어하기 위한 Python 라이브러리(`URRobotController`)와 예제 프로그램을 제공합니다.

**주요 특징:**

- 🎯 **Non-Node 클래스**: `URRobotController`는 ROS2 Node를 상속하지 않고, Node 인스턴스를 생성자에서 받아 사용
- 🔄 **재사용 가능**: 여러 노드에서 동일한 컨트롤러 인스턴스를 공유 가능
- 🚀 **간편한 API**: 복잡한 ROS2 Action/Service 호출을 간단한 메서드로 추상화

**제공 기능:**

- ✅ **MoveJ**: 관절 공간 모션 제어
- ✅ **MoveL**: 직교 공간 직선 모션 제어
- ✅ **Speed Control**: 속도 슬라이더 제어 및 모니터링
- ✅ **Digital I/O**: 디지털 입출력 제어 (18 pins)
- ✅ **State Monitoring**: 실시간 로봇 상태 모니터링
- ✅ **TCP Pose Tracking**: TF2를 통한 TCP 포즈 추적 (4x4 변환 행렬)

## 📦 패키지 정보

- **이름**: `ur_control_client_py`
- **버전**: `1.0.0`
- **빌드 타입**: `ament_python`
- **라이선스**: BSD-3-Clause
- **Maintainer**: hhanoo (woo980711@gmail.com)

---

## 🎯 제공 예제

### 1. `example_movej` - MoveJ 모션 제어

관절 공간에서 로봇을 제어하는 예제

**실행:**

```bash
ros2 run ur_control_client_py example_movej
```

**기능:**

- Home 위치로 이동
- Target 위치로 이동
- Home으로 복귀

---

### 2. `example_io_speed` - I/O 및 속도 제어

디지털 I/O와 속도 슬라이더를 제어하는 예제

**실행:**

```bash
ros2 run ur_control_client_py example_io_speed
```

**기능:**

- 속도 슬라이더 제어 (50%, 100%)
- 디지털 입력 읽기 (DI 0-17)
- 디지털 출력 제어 (DO 0-17)
- 출력 토글 시퀀스

---

### 3. `example_complete` - 통합 예제 (Pick & Place)

모든 기능을 사용하는 완전한 Pick & Place 시퀀스

**실행:**

```bash
ros2 run ur_control_client_py example_complete
```

**기능:**

1. 속도를 30%로 설정 (안전 운전)
2. MoveJ로 Home 위치 이동
3. Digital Output 제어 (DO[0] ON)
4. MoveJ로 Pre-Pick 위치 이동
5. MoveL로 직선 하강 (Pick 위치)
6. 그리퍼 닫기 (DO[1] ON)
7. MoveL로 직선 상승
8. MoveJ로 Place 위치 이동
9. 그리퍼 열기 (DO[1] OFF)
10. MoveJ로 Home 복귀
11. 모든 출력 OFF, 속도 100% 복원

---

## 🚀 사용 방법

### 1. 빌드

```bash
cd /ros2_ws
colcon build --packages-select ur_control_client_py
source install/setup.bash
```

### 2. 사전 준비

클라이언트를 실행하기 전에 UR 로봇 제어 시스템이 실행되어 있어야 합니다.

#### 방법 1: 통합 Launch 파일 사용 (추천) ⭐

**가장 간단한 방법**: 하나의 명령으로 모든 노드 실행

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# 실제 로봇 사용
ros2 launch src/Universal_Robots_ROS2/ur_control.launch.py \
    robot_ip:=192.168.1.25 \
    ur_type:=ur10e \
    use_fake_hardware:=false \
    launch_rviz:=true

# 시뮬레이션 모드
ros2 launch src/Universal_Robots_ROS2/ur_control.launch.py \
    robot_ip:=127.0.0.1 \
    ur_type:=ur10e \
    use_fake_hardware:=true \
    launch_rviz:=true
```

**파라미터 설명:**

- `robot_ip`: 로봇의 IP 주소 (시뮬레이션: `127.0.0.1`, 실제 로봇: `192.168.1.25` 또는 실제 IP)
- `ur_type`: 로봇 모델 (ur3, ur3e, ur5, ur5e, ur10, ur10e, ur16e, ur20, ur30)
- `use_fake_hardware`: 시뮬레이션 모드 (`true`: 시뮬레이션, `false`: 실제 로봇)
- `launch_rviz`: RViz 실행 여부 (`true`: RViz 실행, `false`: RViz 없이 실행)

**이 방법은 다음을 자동으로 실행합니다:**

- ✅ UR Robot Driver
- ✅ MoveIt
- ✅ Motion Action Server

---

#### 방법 2: 개별 실행 (디버깅용)

각 노드를 별도 터미널에서 실행하는 방법입니다.

**터미널 1: UR Driver 실행**

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch ur_robot_driver_wrapper bringup.launch.py \
    robot_ip:=192.168.1.25 \
    ur_type:=ur10e \
    use_fake_hardware:=false
```

**터미널 2: MoveIt 실행**

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch ur_moveit_config_wrapper ur_moveit.launch.py \
    ur_type:=ur10e \
    launch_rviz:=true
```

**터미널 3: Motion Action Server 실행**

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run ur_motion motion_action_server
```

**참고:**

- Planning group은 자동으로 감지됩니다
- 수동 지정이 필요한 경우:
  ```bash
  ros2 run ur_motion motion_action_server \
      --ros-args \
      -p planning_group:=ur10e_manipulator
  ```

### 3. 예제 실행

```bash
# MoveJ 예제
ros2 run ur_control_client_py example_movej

# I/O & Speed 예제
ros2 run ur_control_client_py example_io_speed

# 통합 예제 (Pick & Place)
ros2 run ur_control_client_py example_complete
```

---

## 📝 API 사용 예제

### 기본 사용법

```python
import rclpy
from rclpy.node import Node
from ur_control_client_py import URRobotController

def main(args=None):
    rclpy.init(args=args)
    
    # Create ROS2 node
    node = Node('my_robot_controller')
    
    # Create robot controller
    robot = URRobotController(node)

    # Wait for robot connection
    node.get_logger().info('Waiting for robot connection...')
    while rclpy.ok() and not robot.is_connected():
        rclpy.spin_once(node, timeout_sec=0.1)

    if not robot.is_connected():
        node.get_logger().error('Failed to connect to robot')
        node.destroy_node()
        rclpy.shutdown()
        return

    node.get_logger().info('✅ Robot connected!')

    # Your code here...

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### MoveJ 사용

```python
# Move to home position
home = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
success = robot.move_j(home, velocity=0.5, wait=True)

if success:
    node.get_logger().info('✅ Motion succeeded!')
```

### MoveL 사용

```python
import math

# Helper function to create T-matrix
def create_tmatrix(x, y, z, rx, ry, rz):
    """Create 4x4 transformation matrix from XYZ and RPY (radians)"""
    cr, sr = math.cos(rx), math.sin(rx)
    cp, sp = math.cos(ry), math.sin(ry)
    cy, sy = math.cos(rz), math.sin(rz)

    return [
        cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, x,
        sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, y,
        -sp, cp * sr, cp * cr, z,
        0.0, 0.0, 0.0, 1.0
    ]

# Create T-matrix and move
tmatrix = create_tmatrix(0.4, 0.2, 0.3, math.pi, 0.0, 0.0)
success = robot.move_l(tmatrix, velocity=0.3, wait=True)
```

### Speed Slider 제어

```python
# Set speed slider to 50%
robot.set_speed_slider(0.5)

# Get user-set speed slider value
slider_value = robot.get_speed_slider()
node.get_logger().info(f'Speed slider: {slider_value * 100:.1f}%')

# Get actual robot speed scaling
speed_scaling = robot.get_speed_scaling()
node.get_logger().info(f'Actual speed: {speed_scaling * 100:.1f}%')

# Change speed during motion (async mode)
robot.set_speed_slider(0.3, wait=False)  # Non-blocking call
```

**Speed Slider vs Speed Scaling:**

- **`speed_slider`**: 사용자가 설정한 속도 값 (제어 입력)
- **`speed_scaling`**: 로봇이 실제로 사용하는 속도 값 (실제 출력)
- **관계식**: `speed_scaling = speed_slider × target_speed_fraction`
- **정상 동작**: `target_speed_fraction = 1.0`일 때 두 값이 동일
- **비동기 모드**: `wait=False` 사용 시 모션 중에도 속도 변경 가능

### Digital I/O 제어

```python
# Set digital output
robot.set_digital_out(0, True)   # DO[0] = HIGH
robot.set_digital_out(0, False)  # DO[0] = LOW

# Read digital input
di_state = robot.get_digital_in(0)  # Read DI[0]
node.get_logger().info(f'DI[0]: {"HIGH" if di_state else "LOW"}')

# Read digital output
do_state = robot.get_digital_out(0)  # Read DO[0]
```

### 상태 모니터링

```python
# Check connection
if robot.is_connected():
    node.get_logger().info('✅ Robot connected!')

# Get joint positions
joints = robot.get_joint_positions()
if joints:
    for i, j in enumerate(joints):
        node.get_logger().info(f'Joint[{i}]: {j:.4f} rad')

# Get TCP pose (4x4 transformation matrix)
tcp_pose = robot.get_tcp_pose()
if robot.is_tcp_pose_available():
    position = tcp_pose[:3, 3]  # Extract [x, y, z]
    node.get_logger().info(f'TCP position: [{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]')

# Monitor speed values
slider = robot.get_speed_slider()     # User-set value
scaling = robot.get_speed_scaling()   # Actual robot speed
node.get_logger().info(f'Speed - Slider: {slider:.2f}, Scaling: {scaling:.2f}')
```

---

## 🔧 자신만의 프로그램 만들기

### 1. 새 Python 파일 생성

```bash
cd /ros2_ws/src/Universal_Robots_ROS2/ur_control_client_py/ur_control_client_py
touch my_program.py
chmod +x my_program.py
```

### 2. 기본 구조 작성

```python
#!/usr/bin/env python3
"""
My Custom UR Control Program
"""

import rclpy
from rclpy.node import Node
from ur_control_client_py import URRobotController

def main(args=None):
    rclpy.init(args=args)
    
    # Create ROS2 node
    node = Node('my_robot_controller')
    
    # Create robot controller
    robot = URRobotController(node)

    # Wait for robot connection
    node.get_logger().info('Waiting for robot connection...')
    while rclpy.ok() and not robot.is_connected():
        rclpy.spin_once(node, timeout_sec=0.1)

    if not robot.is_connected():
        node.get_logger().error('Failed to connect to robot')
        node.destroy_node()
        rclpy.shutdown()
        return

    node.get_logger().info('✅ Robot connected!')

    # ========== Your custom logic here ==========

    home = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
    robot.move_j(home, 0.5, True)

    # More motions...

    # ============================================

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3. setup.py에 추가

```python
entry_points={
    'console_scripts': [
        'example_movej = ur_control_client_py.example_movej:main',
        'example_io_speed = ur_control_client_py.example_io_speed:main',
        'example_complete = ur_control_client_py.example_complete:main',
        'my_program = ur_control_client_py.my_program:main',  # ← 추가
    ],
},
```

### 4. 빌드 및 실행

```bash
cd /ros2_ws
colcon build --packages-select ur_control_client_py
source install/setup.bash
ros2 run ur_control_client_py my_program
```

---

## 📚 API 참고

### URRobotController 클래스

`URRobotController`는 non-Node 클래스로, ROS2 Node 인스턴스를 생성자에서 받아 사용합니다.

**구조:**

```python
class URRobotController:
    def __init__(self, node: Node):
        """
        Initialize UR Robot Controller.
        
        Args:
            node: ROS2 node instance for creating subscriptions, clients, etc.
        """
```

**내부 상태 변수:**

```python
# Robot connection
self.connected: bool              # Robot connection status

# Joint states
self.latest_joint_state: JointState | None  # Latest joint state message

# Speed control
self.speed_slider: float          # User-set speed slider [0.01 ~ 1.0]
self.speed_scaling: float         # Actual robot speed [0.0 ~ 1.0]

# Digital I/O
self.digital_in_states: list[bool]   # 18 digital inputs
self.digital_out_states: list[bool]  # 18 digital outputs

# TCP pose (via TF2)
self.tcp_pose_matrix: np.ndarray  # 4x4 transformation matrix
self.tcp_pose_available: bool     # TF availability status
```

---

#### State Monitoring (상태 조회)

로봇의 현재 상태를 조회하는 메서드들입니다.

```python
# Check robot connection
is_connected() -> bool

# Get joint positions (6 joints in radians)
get_joint_positions() -> list[float] | None

# Get TCP pose as 4x4 transformation matrix
get_tcp_pose() -> np.ndarray  # 4x4 matrix (base -> tool0_controller)

# Check if TCP pose is available from TF
is_tcp_pose_available() -> bool

# Get user-set speed slider value
get_speed_slider() -> float  # [0.01 ~ 1.0]

# Get actual robot speed scaling
get_speed_scaling() -> float  # [0.0 ~ 1.0]

# Get digital input state
get_digital_in(pin: int) -> bool  # pin: 0-17

# Get digital output state
get_digital_out(pin: int) -> bool  # pin: 0-17
```

**사용 예시:**

```python
# Robot connection check
if robot.is_connected():
    node.get_logger().info('✅ Robot connected!')

# Joint positions
joints = robot.get_joint_positions()
if joints:
    node.get_logger().info(f'Joints: {joints}')

# TCP pose (NumPy array)
tcp_pose = robot.get_tcp_pose()
position = tcp_pose[:3, 3]  # Extract position [x, y, z]
node.get_logger().info(f'TCP position: {position}')

# Speed monitoring
slider = robot.get_speed_slider()    # User-set value
scaling = robot.get_speed_scaling()  # Actual robot speed
node.get_logger().info(f'Slider: {slider:.2f}, Scaling: {scaling:.2f}')

# Digital I/O states
for i in range(8):
    di_state = robot.get_digital_in(i)
    do_state = robot.get_digital_out(i)
    node.get_logger().info(f'DI[{i}]: {di_state}, DO[{i}]: {do_state}')
```

---

#### Motion Control (모션 제어)

관절 공간 및 직교 공간에서 로봇을 제어합니다.

```python
move_j(joints: list, velocity: float = 0.5, wait: bool = True) -> bool
move_l(tmatrix: list, velocity: float = 0.5, wait: bool = True) -> bool
```

**Parameters:**

- `joints`: 6개 관절 위치 (라디안)
- `tmatrix`: 4x4 변환 행렬 (16개 요소, row-major) 또는 numpy ndarray
- `velocity`: 속도 스케일링 [0.01 ~ 1.0]
- `wait`: 모션 완료 대기 여부

**Returns:**

- `True`: 성공
- `False`: 실패

---

#### Speed Control (속도 제어)

로봇의 속도 슬라이더를 제어합니다.

```python
set_speed_slider(slider_value: float, wait: bool = True) -> bool | Future
```

**Parameters:**

- `slider_value`: 속도 슬라이더 값 [0.01 ~ 1.0]
- `wait`: 서비스 응답 대기 여부
  - `True` (기본값): 동기 모드, 완료될 때까지 대기
  - `False`: 비동기 모드, 즉시 반환 (모션 중 속도 변경 시 사용)

**Returns:**

- `wait=True`: `bool` (성공 여부)
- `wait=False`: `Future` (비동기 호출 결과)

**사용 예시:**

```python
# Synchronous mode (wait for response)
success = robot.set_speed_slider(0.5, wait=True)
if success:
    node.get_logger().info('✅ Speed changed to 50%')

# Asynchronous mode (change speed during motion)
robot.set_speed_slider(0.3, wait=False)  # Non-blocking
# Motion continues while speed changes
```

**속도 값 조회:**

```python
get_speed_slider() -> float   # User-set value
get_speed_scaling() -> float  # Actual robot speed
```

---

#### I/O Control (디지털 I/O 제어)

디지털 출력을 제어합니다 (입력은 State Monitoring 참조).

```python
set_digital_out(pin: int, value: bool) -> bool  # pin: 0-17
```

---

## 🔌 Digital I/O Pin Mapping

| Pin Range | Type         | Description            |
| --------- | ------------ | ---------------------- |
| 0-7       | Standard     | 표준 디지털 I/O        |
| 8-15      | Configurable | 설정 가능한 디지털 I/O |
| 16-17     | Tool         | 툴 디지털 I/O          |

---

## ⚡ 속도 제어의 이해

### Speed Slider vs Speed Scaling

UR 로봇의 속도 제어는 두 가지 값으로 구성됩니다:

#### 1. **Speed Slider** (사용자 제어 값)
- 사용자가 `set_speed_slider()`로 설정하는 값
- `get_speed_slider()`로 조회 가능
- 범위: `[0.01 ~ 1.0]`
- **의미**: "로봇이 이 속도로 동작하도록 설정"

#### 2. **Speed Scaling** (실제 로봇 속도)
- 로봇이 실제로 사용하는 속도 값
- `get_speed_scaling()`로 조회 가능
- 범위: `[0.0 ~ 1.0]`
- **의미**: "로봇이 실제로 이 속도로 동작 중"

### 속도 관계식

```
speed_scaling = speed_slider × target_speed_fraction
```

**정상 동작 시:**
- `target_speed_fraction = 1.0` (UR 로봇 내부 값)
- ∴ `speed_scaling = speed_slider`
- 즉, 설정한 값과 실제 값이 동일

**예외 상황:**
- Teach Pendant에서 속도를 변경한 경우
- 안전 기능이 활성화된 경우
- 이런 경우 `speed_scaling ≠ speed_slider`

### 동기 vs 비동기 모드

#### 동기 모드 (`wait=True`)
```python
# 서비스 호출이 완료될 때까지 대기
success = robot.set_speed_slider(0.5, wait=True)
if success:
    print("✅ Speed changed!")
```

**특징:**
- 블로킹 호출 (완료될 때까지 대기)
- 반환값: `bool` (성공 여부)
- 사용 시점: 모션 전에 속도를 미리 설정

#### 비동기 모드 (`wait=False`)
```python
# 서비스 호출 후 즉시 반환
future = robot.set_speed_slider(0.8, wait=False)
# 모션이 계속 실행됨
```

**특징:**
- 논블로킹 호출 (즉시 반환)
- 반환값: `Future` (비동기 결과)
- 사용 시점: **모션 중에 속도를 실시간으로 변경**

### 실전 예제

#### 예제 1: 느린 접근 → 빠른 복귀
```python
# Slow approach (30%)
robot.set_speed_slider(0.3)
robot.move_j(approach_position, velocity=0.5, wait=True)

# Fast return (100%)
robot.set_speed_slider(1.0)
robot.move_j(home_position, velocity=0.8, wait=True)
```

#### 예제 2: 모션 중 속도 변경
```python
# Start slow motion
robot.set_speed_slider(0.3)
robot.move_j(target_position, velocity=0.8, wait=False)  # Non-blocking

# Wait 2 seconds
time.sleep(2.0)

# Speed up during motion (async mode)
robot.set_speed_slider(1.0, wait=False)

# Continue until motion completes
# (Robot will accelerate to 100% during motion)
```

#### 예제 3: 속도 모니터링
```python
import time

# Set speed slider
robot.set_speed_slider(0.5)

# Monitor actual speed
for i in range(10):
    slider = robot.get_speed_slider()    # 0.5 (사용자 설정값)
    scaling = robot.get_speed_scaling()  # ~0.5 (실제 로봇 속도)
    print(f"Slider: {slider:.2f}, Scaling: {scaling:.2f}")
    time.sleep(0.5)
```

---

## 🛠️ 문제 해결

### 1. Action 서버를 찾을 수 없음

```
MoveJ action server not available
```

**해결:**

```bash
# Motion Action Server가 실행 중인지 확인
ros2 action list
# /move_j 와 /move_l 이 있어야 함

# 없는 경우 실행
ros2 run ur_motion motion_action_server
```

---

### 2. 로봇 연결 실패

```
Failed to connect to robot
```

**해결:**

```bash
# UR Driver가 실행 중인지 확인
ros2 topic list | grep joint_states
# /joint_states 토픽이 있어야 함

# 없는 경우 UR Driver 실행
ros2 launch ur_robot_driver_wrapper bringup.launch.py \
    robot_ip:=192.168.1.25 ur_type:=ur10e
```

---

### 3. I/O 서비스 사용 불가

```
I/O service not available
```

**해결:**

```bash
# UR Driver의 I/O 컨트롤러 확인
ros2 service list | grep set_io
# /io_and_status_controller/set_io 서비스가 있어야 함
```

---

### 4. MoveL 실패

```
MoveL failed
```

**해결:**

- 목표 위치가 로봇의 작업 공간 내에 있는지 확인
- RViz에서 충돌이 발생하지 않는지 확인
- 속도를 낮춰서 다시 시도:
  ```python
  robot.set_speed_slider(0.3)
  robot.move_l(tmatrix, velocity=0.2)
  ```

---

### 5. Speed Slider와 Speed Scaling이 다름

```python
slider = robot.get_speed_slider()    # 1.0
scaling = robot.get_speed_scaling()  # 0.5  ← Why different?
```

**원인:**

- Teach Pendant에서 속도를 수동으로 변경함
- 안전 기능이 활성화됨 (Safety Stop, Reduced Mode 등)
- `target_speed_fraction` 값이 1.0이 아님

**해결:**

```python
# Check if values match
slider = robot.get_speed_slider()
scaling = robot.get_speed_scaling()

if abs(slider - scaling) > 0.01:
    node.get_logger().warn(
        f'Speed mismatch! Slider: {slider:.2f}, Scaling: {scaling:.2f}'
    )
    node.get_logger().warn('Check Teach Pendant or Safety settings')
```

---

### 6. TCP Pose를 가져올 수 없음

```python
tcp_pose = robot.get_tcp_pose()
# Returns identity matrix (no actual pose)
```

**원인:**

- TF 변환이 아직 준비되지 않음
- UR Driver가 TF를 브로드캐스트하지 않음

**해결:**

```python
# Wait for TF to be available
import time

while rclpy.ok() and not robot.is_tcp_pose_available():
    rclpy.spin_once(node, timeout_sec=0.1)
    time.sleep(0.1)

if robot.is_tcp_pose_available():
    tcp_pose = robot.get_tcp_pose()
    node.get_logger().info('✅ TCP pose available!')
```

---

### 7. 모션 중 속도 변경이 안됨

```python
# This will block and cause threading issues
robot.move_j(target, wait=False)
robot.set_speed_slider(0.5, wait=True)  # ← Blocking call!
```

**해결:**

```python
# Use async mode for speed change during motion
robot.move_j(target, velocity=0.8, wait=False)
time.sleep(2.0)
robot.set_speed_slider(1.0, wait=False)  # ← Non-blocking!
```

---

## 📖 추가 정보

### 의존성

- `rclpy`: ROS2 Python 클라이언트 라이브러리
- `ur_motion`: UR 로봇 모션 제어 패키지 (Action 정의)
- `ur_msgs`: UR ROS2 Driver 메시지 및 서비스
- `sensor_msgs`: 센서 메시지 (JointState 등)
- `std_msgs`: 표준 메시지
- `geometry_msgs`: 기하학적 메시지 (Transform 등)
- `tf2_ros`: TF2 라이브러리 (TCP 포즈 추적)
- `numpy`: 수치 계산 (4x4 변환 행렬)
- `scipy`: 과학 계산 (Rotation 변환)

### 관련 패키지

- **`ur_control_client`**: C++ 클라이언트 라이브러리 (동일 기능)
- **`ur_motion`**: Motion Action Server 및 MoveIt 백엔드
- **`ur_robot_driver_wrapper`**: UR ROS2 Driver 래퍼
- **`ur_moveit_config_wrapper`**: MoveIt 설정 래퍼
- **`opcua_server`**: OPC UA 서버 (산업용 통신)

### 참고 자료

- [ROS2 Actions 문서](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Python.html)
- [MoveIt2 문서](https://moveit.picknik.ai/humble/index.html)
- [UR ROS2 Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)

---

## 📄 라이선스

BSD-3-Clause

## 👥 Maintainer

hhanoo (woo980711@gmail.com)
