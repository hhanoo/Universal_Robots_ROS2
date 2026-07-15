# UR Robot Client (C++)

**`URRobotClient` — UR 로봇 제어 C++ 클라이언트 라이브러리 + 예제 4종**

> 설치·빌드·시스템 실행(Driver/MoveIt/Motion Server)·Docker·문제 해결은 [프로젝트 루트 README](../README.md)를 참고하세요. 이 문서는 이 패키지의 API와 사용 패턴만 다룹니다.

---

## 개요

- `URRobotClient`는 `rclcpp::Node` 상속 클래스로, **생성자에서 자체 background executor 스레드를 시작**합니다. 외부에서 spin하거나 다른 executor에 추가하면 `already been added to an executor` 예외로 크래시합니다 — 생성만 하면 됩니다
- 모션·서비스 API는 `std::future<MotionResult>`를 반환합니다 — `.get()`이면 동기(완료 대기), future를 보관하면 비동기
- **Program watchdog 내장**: e-stop/Local 모드로 제어권을 잃으면 로봇 복구(robot mode `RUNNING` + safety mode `NORMAL`) 시점에 `resend_robot_program`을 자동 호출해 제어권을 회복합니다 (3초 간격 재시도, 비상정지 해제 등 물리 복구는 자동화하지 않음)

---

## 제공 예제

단계 순서대로: 연결 점검 → 기본 모션 → I/O → 통합 시퀀스.

| 예제                 | 실행                                          | 내용                                            |
| -------------------- | --------------------------------------------- | ----------------------------------------------- |
| `example_state`      | `ros2 run ur_robot_client example_state`      | 1. 읽기 전용 상태 모니터링 (실로봇 연결 점검용) |
| `example_movej`      | `ros2 run ur_robot_client example_movej`      | 2. HOME → TARGET → HOME 관절 공간 모션          |
| `example_io_speed`   | `ros2 run ur_robot_client example_io_speed`   | 3. Speed slider 50/100% + DI 읽기 + DO 토글     |
| `example_pick_place` | `ros2 run ur_robot_client example_pick_place` | 4. Pick & Place 통합 시퀀스 (모든 기능 사용)    |

실행 전 UR 제어 스택이 떠 있어야 합니다 — Docker 컨테이너에서 `run-all`, 또는 [all.launch.py](../all.launch.py) (루트 README 참고).

---

## 기본 사용 패턴

```cpp
#include <rclcpp/rclcpp.hpp>

#include "ur_robot_client/ur_robot_client.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // 생성만 하면 자체 executor 스레드가 스핀 — 외부 spin 금지
    auto client = std::make_shared<URRobotClient>();

    // 상태 수신(관절/속도/TF)이 준비될 때까지 대기
    if (!client->waitRobotReady(5.0)) {
        RCLCPP_ERROR(client->get_logger(), "Robot not ready");
        rclcpp::shutdown();
        return 1;
    }

    // MoveJ — future.get()으로 완료까지 대기
    std::vector<double> home = {0.0, -1.57, 1.57, -1.57, -1.57, 0.0};
    auto result = client->moveJ(home, 0.5).get();  // velocity=0.5
    if (!result.success) {
        RCLCPP_ERROR(client->get_logger(), "MoveJ failed: %s", result.message.c_str());
    }

    rclcpp::shutdown();
    return 0;
}
```

새 실행 파일을 추가하려면 [CMakeLists.txt](CMakeLists.txt)의 예제 등록 블록(add_executable + install)을 복사해 사용하세요.

---

## API 레퍼런스

모든 시그니처는 [ur_robot_client.hpp](include/ur_robot_client/ur_robot_client.hpp) 기준입니다.

### 연결 / 준비

```cpp
bool isConnected() const;                          // joint_states 1회 이상 수신
bool isRobotReady(bool require_io = false) const;  // 관절 + 속도 + TF (옵션: I/O 포함)
bool waitRobotReady(double timeout_sec = 5.0, bool require_io = false);
```

### 모션 제어

```cpp
struct MotionResult {
    bool        success;
    std::string message;
};

std::future<MotionResult> moveJ(
    const std::vector<double>& joints,      // 6관절 [rad]
    double                     velocity = 0.5,   // [0.05 ~ 1.0]
    double                     timeout  = 60.0); // [s]

std::future<MotionResult> moveL(
    const std::array<double, 16>& tmatrix,  // 4x4 T-matrix (row-major)
    double                        velocity = 0.5,
    double                        timeout  = 60.0);

bool moveCancel();  // 진행 중인 MoveJ/MoveL 취소
```

```cpp
// 동기: 완료까지 대기
auto result = client->moveJ(home, 0.5).get();

// 비동기: future를 보관했다가 나중에 결과 확인
auto future = client->moveL(tmatrix, 0.3);
// ... 다른 작업 ...
auto result = future.get();
```

### 속도 / I/O 제어

```cpp
std::future<MotionResult> setSpeedSlider(double fraction, double timeout = 1.0);      // [0.01 ~ 1.0]
std::future<MotionResult> setDigitalOut(int pin, bool value, double timeout = 1.0);   // pin: 0-17

double getSpeedSlider() const;   // 마지막으로 설정한 값 (로컬 캐시)
double getSpeedScaling() const;  // 로봇이 실제 적용 중인 속도 [0.0 ~ 1.0]
bool   getDigitalIn(int pin) const;
bool   getDigitalOut(int pin) const;
```

> `speed_slider`는 설정값(입력), `speed_scaling`은 실제 적용값(출력)입니다. 정상 운전에서는 같지만, Pendant 수동 조작이나 안전 감속 시 `scaling`만 달라집니다.

### 상태 조회

```cpp
std::array<double, 6>  getJointPositions() const;  // shoulder_pan → wrist_3 순서 [rad]
std::array<double, 16> getTcpPose() const;         // 4x4 T-matrix (base → tool0_controller)
bool isTcpPoseAvailable() const;

bool isProgramRunning() const;  // external control 실행 여부 (false = 제어권 상실)
int  isRemoteControl() const;   // Pendant: 1=remote, 0=local, -1=unknown (5초 폴링 캐시)

int8_t  getRobotMode() const;   // ur_dashboard_msgs RobotMode 상수 (RUNNING=7, IDLE=5, ...)
uint8_t getSafetyMode() const;  // ur_dashboard_msgs SafetyMode 상수 (NORMAL=1, PROTECTIVE_STOP=3, ...)
```

---

## Digital I/O 핀 맵

| Pin Range | Type         | 설명                   |
| --------- | ------------ | ---------------------- |
| 0-7       | Standard     | 표준 디지털 I/O        |
| 8-15      | Configurable | 설정 가능한 디지털 I/O |
| 16-17     | Tool         | 툴 디지털 I/O          |

---

## 관련 패키지

- [ur_robot_client_py](../ur_robot_client_py/): 동일 기능의 Python 클라이언트 (async/await 기반)
- [ur_motion](../ur_motion/): MoveJ/MoveL Action Server (Action 정의 포함)

## 라이선스

Apache-2.0 — [프로젝트 루트 LICENSE](../LICENSE) 참고.
