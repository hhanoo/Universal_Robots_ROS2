# UR Robot Client (Python)

**`URRobotClient` — UR 로봇 제어 Python 클라이언트 라이브러리 + 예제 4종 (async/await 기반)**

> 설치·빌드·시스템 실행(Driver/MoveIt/Motion Server)·Docker·문제 해결은 [프로젝트 루트 README](../README.md)를 참고하세요. 이 문서는 이 패키지의 API와 사용 패턴만 다룹니다.

---

## 개요

- `URRobotClient`는 **non-Node 클래스**입니다 — ROS2 Node를 상속하지 않고 생성자에서 Node 인스턴스를 주입받아 사용하므로, 기존 노드에 쉽게 얹을 수 있습니다
- 스핀은 호출자 책임입니다 — `rclpy.spin_once()` 루프(또는 executor)를 직접 돌려야 콜백과 모션 결과가 처리됩니다
- **모션·서비스 API는 코루틴**입니다 — `await robot.move_j(...)`처럼 호출하며 `(success, message)` 튜플을 반환합니다
- 제어권(Program)·Robot/Safety mode·Pendant Remote/Local 상태 조회 제공 (자동 제어권 회복 watchdog은 C++ 클라이언트 전용)

---

## 제공 예제

| 예제               | 실행                                           | 내용                                         |
| ------------------ | ---------------------------------------------- | -------------------------------------------- |
| `example_movej`    | `ros2 run ur_robot_client_py example_movej`    | HOME → TARGET → HOME 관절 공간 모션          |
| `example_io_speed` | `ros2 run ur_robot_client_py example_io_speed` | Speed slider 50/100% + DI 읽기 + DO 토글     |
| `example_complete` | `ros2 run ur_robot_client_py example_complete` | Pick & Place 통합 시퀀스 (모든 기능 사용)    |
| `example_state`    | `ros2 run ur_robot_client_py example_state`    | 읽기 전용 상태 모니터링 (실로봇 연결 점검용) |

실행 전 UR 제어 스택이 떠 있어야 합니다 — Docker 컨테이너에서 `run-all`, 또는 [all.launch.py](../all.launch.py) (루트 README 참고).

---

## 기본 사용 패턴

```python
#!/usr/bin/env python3
import asyncio

import rclpy
from rclpy.node import Node

from ur_robot_client_py import URRobotClient


async def spin_node(node):
    """호출자가 스핀을 책임짐 — asyncio 태스크로 돌리는 패턴"""
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        await asyncio.sleep(0.01)


async def main_async():
    rclpy.init()
    node = Node("my_controller")
    robot = URRobotClient(node)  # Node 주입

    spin_task = asyncio.create_task(spin_node(node))

    # 상태 수신(관절/속도/TF)이 준비될 때까지 대기
    if not await robot.wait_robot_ready(timeout=10.0):
        node.get_logger().error("Robot not ready")
        return

    # MoveJ — 코루틴이므로 await, (success, message) 반환
    home = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
    success, msg = await robot.move_j(home, velocity=0.5)
    if not success:
        node.get_logger().error(f"MoveJ failed: {msg}")

    spin_task.cancel()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def main():
    asyncio.run(main_async())


if __name__ == "__main__":
    main()
```

새 스크립트를 추가하려면 [setup.py](setup.py)의 `entry_points`에 한 줄 등록 후 재빌드하세요.

---

## API 레퍼런스

모든 시그니처는 [ur_robot_client.py](ur_robot_client_py/ur_robot_client.py) 기준입니다.

### 연결 / 준비

```python
is_connected() -> bool                              # joint_states 1회 이상 수신
is_robot_ready(require_io=False) -> bool            # 관절 + 속도 + TF (옵션: I/O 포함)
await wait_robot_ready(timeout=5.0, require_io=False) -> bool
```

### 모션 제어 (코루틴)

```python
await move_j(joints, velocity=0.5, timeout=30.0) -> (bool, str)   # 6관절 [rad]
await move_l(tmatrix, velocity=0.5, timeout=30.0) -> (bool, str)  # 4x4 T-matrix (16개 row-major)
move_cancel() -> bool                                              # 진행 중인 모션 취소
await wait(duration_sec)                                           # 모션 사이 지연
```

```python
# 순차 실행 — await가 완료 대기 역할
success, msg = await robot.move_j(home, velocity=0.5)
await robot.wait(1.0)
success, msg = await robot.move_l(tmatrix, velocity=0.3)
```

### 속도 / I/O 제어 (코루틴)

```python
await set_speed_slider(slider_value, timeout=1.0) -> (bool, str)  # [0.01 ~ 1.0]
await set_digital_out(pin, value, timeout=1.0) -> (bool, str)     # pin: 0-17

get_speed_slider() -> float   # 마지막으로 설정한 값 (로컬 캐시)
get_speed_scaling() -> float  # 로봇이 실제 적용 중인 속도 [0.0 ~ 1.0]
get_digital_in(pin) -> bool
get_digital_out(pin) -> bool
```

> **Speed Slider vs Speed Scaling**: `slider`는 설정값(입력), `scaling`은 실제 적용값(출력)으로 `scaling = slider × target_speed_fraction` 관계입니다. 정상 운전에서는 같지만, Pendant 수동 조작이나 안전 감속(Reduced mode 등) 시 `scaling`만 달라집니다 — 두 값이 다르면 Pendant/안전 설정을 확인하세요.

### 상태 조회

```python
get_joint_positions() -> list | None   # shoulder_pan → wrist_3 순서 [rad]
get_tcp_pose() -> np.ndarray           # 4x4 T-matrix (base → tool0_controller)
is_tcp_pose_available() -> bool

is_program_running() -> bool   # external control 실행 여부 (False = 제어권 상실)
is_remote_control() -> int     # Pendant: 1=remote, 0=local, -1=unknown (5초 폴링 캐시)

get_robot_mode() -> int        # ur_dashboard_msgs RobotMode 상수 (RUNNING=7, IDLE=5, ...)
get_safety_mode() -> int       # ur_dashboard_msgs SafetyMode 상수 (NORMAL=1, PROTECTIVE_STOP=3, ...)
```

> fake hardware(`use_fake_hardware:=true`)에서는 program/robot mode/safety mode 토픽과 dashboard 서비스가 없어 각각 초기값(False / DISCONNECTED / -1)에 머뭅니다.

---

## Digital I/O 핀 맵

| Pin Range | Type         | 설명                   |
| --------- | ------------ | ---------------------- |
| 0-7       | Standard     | 표준 디지털 I/O        |
| 8-15      | Configurable | 설정 가능한 디지털 I/O |
| 16-17     | Tool         | 툴 디지털 I/O          |

---

## 관련 패키지

- [ur_robot_client](../ur_robot_client/): 동일 기능의 C++ 클라이언트 (자체 executor + watchdog 자동 회복 포함)
- [ur_motion](../ur_motion/): MoveJ/MoveL Action Server (Action 정의 포함)

## 라이선스

Apache-2.0 — [프로젝트 루트 LICENSE](../LICENSE) 참고.
