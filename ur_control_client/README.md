# UR Control Client (C++)

C++ 기반 UR 로봇 제어 클라이언트 라이브러리 및 예제

## 📋 개요

이 패키지는 UR 로봇을 제어하기 위한 C++ 클라이언트 라이브러리와 예제 프로그램을 제공합니다.

**제공 기능:**

- ✅ **MoveJ**: 관절 공간 모션 제어
- ✅ **MoveL**: 직교 공간 직선 모션 제어
- ✅ **Speed Slider**: 속도 제어
- ✅ **Digital I/O**: 디지털 입출력 제어
- ✅ **State Monitoring**: 로봇 상태 모니터링

## 📦 패키지 정보

- **이름**: `ur_control_client`
- **버전**: `1.0.0`
- **빌드 타입**: `ament_cmake`
- **라이선스**: BSD-3-Clause
- **Maintainer**: hhanoo (woo980711@gmail.com)

---

## 🎯 제공 예제

### 1. `example_movej` - MoveJ 모션 제어

관절 공간에서 로봇을 제어하는 예제

**실행:**

```bash
ros2 run ur_control_client example_movej
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
ros2 run ur_control_client example_io_speed
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
ros2 run ur_control_client example_complete
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
colcon build --packages-select ur_control_client
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
ros2 run ur_control_client example_movej

# I/O & Speed 예제
ros2 run ur_control_client example_io_speed

# 통합 예제 (Pick & Place)
ros2 run ur_control_client example_complete
```

---

## 📝 API 사용 예제

### 기본 사용법

```cpp
#include "ur_control_client/ur_control_client.hpp"
#include <rclcpp/rclcpp.hpp>
#include <thread>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto client = std::make_shared<URControlClient>();

    // Spin in background thread
    std::thread spin_thread([client]() {
        rclcpp::spin(client);
    });

    // Wait for robot connection
    while (rclcpp::ok() && !client->isConnected()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (!client->isConnected()) {
        RCLCPP_ERROR(client->get_logger(), "Failed to connect to robot");
        rclcpp::shutdown();
        spin_thread.join();
        return 1;
    }

    // Your code here...

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}
```

### MoveJ 사용

```cpp
// Move to home position
std::vector<double> home = {0.0, -1.57, 1.57, -1.57, -1.57, 0.0};
bool success = client->moveJ(home, 0.5, true);  // velocity=0.5, wait=true

if (success) {
    RCLCPP_INFO(client->get_logger(), "Motion succeeded!");
}
```

### MoveL 사용

```cpp
// Create T-matrix (4x4 transformation matrix, row-major)
std::array<double, 16> tmatrix = {
    1.0, 0.0, 0.0, 0.4,   // R11, R12, R13, Tx
    0.0, 1.0, 0.0, 0.2,   // R21, R22, R23, Ty
    0.0, 0.0, 1.0, 0.3,   // R31, R32, R33, Tz
    0.0, 0.0, 0.0, 1.0    // 0,   0,   0,   1
};

bool success = client->moveL(tmatrix, 0.3, true);
```

### Speed Slider 제어

```cpp
// Set speed to 50%
client->setSpeedSlider(0.5);

// Get current speed
double current_speed = client->getSpeedScaling();
RCLCPP_INFO(client->get_logger(), "Current speed: %.1f%%", current_speed * 100.0);
```

### Digital I/O 제어

```cpp
// Set digital output
client->setDigitalOut(0, true);   // DO[0] = HIGH
client->setDigitalOut(0, false);  // DO[0] = LOW

// Read digital input
bool di_state = client->getDigitalIn(0);  // Read DI[0]

// Read digital output
bool do_state = client->getDigitalOut(0);  // Read DO[0]
```

### 상태 모니터링

```cpp
// Check connection
if (client->isConnected()) {
    RCLCPP_INFO(client->get_logger(), "Robot connected!");
}

// Get joint positions
std::vector<double> joints;
if (client->getJointPositions(joints)) {
    for (size_t i = 0; i < joints.size(); ++i) {
        RCLCPP_INFO(client->get_logger(), "Joint[%zu]: %.4f rad", i, joints[i]);
    }
}
```

---

## 🔧 자신만의 프로그램 만들기

### 1. 새 C++ 파일 생성

```bash
cd /ros2_ws/src/Universal_Robots_ROS2/ur_control_client/src
touch my_program.cpp
```

### 2. 기본 구조 작성

```cpp
#include "ur_control_client/ur_control_client.hpp"
#include <rclcpp/rclcpp.hpp>
#include <thread>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto client = std::make_shared<URControlClient>();

    // Spin in background thread
    std::thread spin_thread([client]() {
        rclcpp::spin(client);
    });

    // Wait for robot connection
    while (rclcpp::ok() && !client->isConnected()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (!client->isConnected()) {
        RCLCPP_ERROR(client->get_logger(), "Failed to connect to robot");
        rclcpp::shutdown();
        spin_thread.join();
        return 1;
    }

    // ========== Your custom logic here ==========

    std::vector<double> home = {0.0, -1.57, 1.57, -1.57, -1.57, 0.0};
    client->moveJ(home, 0.5, true);

    // More motions...

    // ============================================

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}
```

### 3. CMakeLists.txt에 추가

```cmake
# Add your executable
add_executable(my_program src/my_program.cpp)
target_link_libraries(my_program ${PROJECT_NAME}_lib)
ament_target_dependencies(my_program rclcpp)

# Install your executable
install(
  TARGETS my_program
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
```

### 4. 빌드 및 실행

```bash
cd /ros2_ws
colcon build --packages-select ur_control_client
source install/setup.bash
ros2 run ur_control_client my_program
```

---

## 📚 API 참고

### URControlClient 클래스

#### Motion Control

```cpp
bool moveJ(const std::vector<double>& joints, double velocity = 0.5, bool wait = true);
bool moveL(const std::array<double, 16>& tmatrix, double velocity = 0.5, bool wait = true);
```

#### Speed Control

```cpp
bool setSpeedSlider(double fraction);  // [0.01 ~ 1.0]
double getSpeedScaling() const;
```

#### I/O Control

```cpp
bool setDigitalOut(int pin, bool value);  // pin: 0-17
bool getDigitalIn(int pin) const;
bool getDigitalOut(int pin) const;
```

#### State Monitoring

```cpp
bool isConnected() const;
bool getJointPositions(std::vector<double>& joints) const;
```

---

## 🔌 Digital I/O Pin Mapping

| Pin Range | Type         | Description            |
| --------- | ------------ | ---------------------- |
| 0-7       | Standard     | 표준 디지털 I/O        |
| 8-15      | Configurable | 설정 가능한 디지털 I/O |
| 16-17     | Tool         | 툴 디지털 I/O          |

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
```

### 2. 로봇 연결 실패

```
Failed to connect to robot
```

**해결:**

```bash
# UR Driver가 실행 중인지 확인
ros2 topic list | grep joint_states
# /joint_states 토픽이 있어야 함
```

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

### 4. MoveL 실패

```
MoveL failed
```

**해결:**

- 목표 위치가 로봇의 작업 공간 내에 있는지 확인
- RViz에서 충돌이 발생하지 않는지 확인
- 속도를 낮춰서 다시 시도

---

## 📖 추가 정보

### 의존성

- `rclcpp`: ROS2 C++ 클라이언트 라이브러리
- `rclcpp_action`: ROS2 Action 클라이언트
- `ur_motion`: UR 로봇 모션 제어 패키지 (Action 정의)
- `ur_msgs`: UR ROS2 Driver 메시지 및 서비스
- `sensor_msgs`: 센서 메시지 (JointState 등)
- `std_msgs`: 표준 메시지

### 관련 패키지

- **`ur_control_client_py`**: Python 클라이언트 라이브러리 (동일 기능)
- **`ur_motion`**: Motion Action Server 및 MoveIt 백엔드
- **`ur_robot_driver_wrapper`**: UR ROS2 Driver 래퍼
- **`ur_moveit_config_wrapper`**: MoveIt 설정 래퍼
- **`opcua_server`**: OPC UA 서버 (산업용 통신)

### 참고 자료

- [ROS2 Actions 문서](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html)
- [MoveIt2 문서](https://moveit.picknik.ai/humble/index.html)
- [UR ROS2 Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)

---

## 📄 라이선스

BSD-3-Clause

## 👥 Maintainer

hhanoo (woo980711@gmail.com)
