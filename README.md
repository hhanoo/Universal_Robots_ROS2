# Universal_Robots_ROS2

Universal Robots ROS2 패키지 모음

## 📦 패키지 구성

- `ur_robot_driver_wrapper` - UR 로봇 드라이버 래퍼
- `ur_moveit_config_wrapper` - MoveIt 설정 래퍼
- `ur_motion` - Motion Action Server (MoveJ/MoveL)
- `ur_robot_client` - C++ 클라이언트 라이브러리
- `ur_robot_client_py` - **Python 로봇 제어 라이브러리** (`URRobotClient` 클래스)

## 💡 주요 특징

### URRobotClient (Python)

`ur_robot_client_py` 패키지는 UR 로봇 제어를 위한 Python 라이브러리를 제공합니다.

**제공 기능:**

- ✅ **MoveJ/MoveL**: 관절 공간 및 직교 공간 모션 제어
- ✅ **Speed Control**: 속도 슬라이더 제어
- ✅ **Digital I/O**: 디지털 입출력 제어
- ✅ **State Monitoring**: 로봇 상태 모니터링
- ✅ **TCP Pose Tracking**: TF를 통한 실시간 TCP 포즈 추적

**구조:**

`URRobotClient`는 non-Node 클래스로, ROS2 Node 인스턴스를 받아 사용합니다.

```python
from rclpy.node import Node
from ur_robot_client_py import URRobotClient

node = Node('my_controller')
robot = URRobotClient(node)  # Node를 주입
robot.move_j([0, -1.57, 1.57, -1.57, -1.57, 0])
```

자세한 내용은 [`ur_robot_client_py/README.md`](ur_robot_client_py/README.md)를 참조하세요.

## 🚀 Launch 파일

### `ur_control.launch.py`

UR 로봇 제어 시스템의 모든 노드를 실행하는 통합 launch 파일입니다.

**실행하는 노드:**

1. UR Robot Driver - UR 로봇 하드웨어/시뮬레이터와 통신
2. MoveIt - 모션 플래닝 및 충돌 감지
3. Motion Action Server - MoveJ/MoveL 액션 서버

**사용 방법:**

```bash
# 워크스페이스 루트에서 실행
cd /ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# 기본 실행 (시뮬레이션 모드)
ros2 launch src/ur_control.launch.py

# 실제 로봇 사용
ros2 launch src/ur_control.launch.py \
    robot_ip:=192.168.1.25 \
    ur_type:=ur10e \
    use_fake_hardware:=false

# 시뮬레이션 모드
ros2 launch src/ur_control.launch.py \
    robot_ip:=127.0.0.1 \
    ur_type:=ur10e \
    use_fake_hardware:=true \
    launch_rviz:=true
```

**Launch Arguments:**

- `robot_ip` (default: `127.0.0.1`) - UR 로봇의 IP 주소 (시뮬레이션: `127.0.0.1`)
- `ur_type` (default: `ur10e`) - 로봇 타입 (`ur3`, `ur3e`, `ur5`, `ur5e`, `ur10`, `ur10e`, `ur16e`, `ur20`, `ur30`)
- `use_fake_hardware` (default: `false`) - 시뮬레이션 모드 사용 여부
- `launch_rviz` (default: `true`) - RViz 실행 여부

**모든 인자 확인:**

```bash
ros2 launch src/ur_control.launch.py --show-args
```
