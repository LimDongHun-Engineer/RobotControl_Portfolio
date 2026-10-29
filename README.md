# 🚀 C++ 기반 로봇 모션 제어 시스템 코어 (Motion Control Core)

이 프로젝트는 상용 모션 제어기(PMAC 등)의 블랙박스 종속성에서 벗어나, 로봇 제어의 필수적인 수학적/물리적 이론을 C++ 객체지향 프로그래밍(OOP)으로 밑바닥부터 직접 구현한 포트폴리오입니다.

## 📌 주요 파이프라인 및 구현 내용 (Part 1. C++ Core)

현업의 로봇 제어 파이프라인(목표 좌표 입력 → 역기구학 연산 → 궤적 생성 → PID 추종)을 5단계로 모듈화하여 구현했습니다.

*   **Phase 1. 이산시간(Discrete-time) PID 제어기**
    *   1ms 제어 주기를 갖는 차분 방정식 기반 PID 알고리즘 구현
    *   모터의 물리적 한계를 고려한 Anti-windup (적분 누적 방지) 방어 로직 적용
*   **Phase 2. 사다리꼴 궤적 생성기 (Trapezoidal Trajectory)**
    *   최대 속도($V_{max}$)와 최대 가속도($A_{max}$) 제한을 고려한 위치 지령(Command) 보간
    *   목표 거리가 짧을 경우 발생하는 삼각 궤적 예외 처리 구현 및 CSV 데이터 추출
*   **Phase 3. 2-DOF 정기구학 (Forward Kinematics)**
    *   삼각함수 덧셈 정리와 호도법(Radian)을 이용한 끝단 좌표 연산
    *   `Eigen` 선형대수 라이브러리의 `Vector2d`를 활용한 좌표계 최적화
*   **Phase 4. 2-DOF 역기구학 (Inverse Kinematics)**
    *   제2 코사인 법칙 및 `atan2`를 활용하여 목표 좌표를 모터의 관절 각도로 역산
    *   작업 반경(Workspace) 이탈 시의 안전 예외 처리(Defensive Programming) 구현
*   **Phase 5. 통합 다관절 제어 시뮬레이터 (Integrated System)**
    *   역기구학 모듈과 2개의 PID 제어기(1축, 2축) 객체를 결합하여, 공간 좌표 입력 시 스스로 오차를 줄이며 목표로 기동하는 통합 시스템 완성

## 🛠 기술 스택 (Tech Stack)
*   **Language:** Modern C++
*   **Math/Algebra:** Eigen3 Library
*   **Tools:** Visual Studio 2022, Git, SourceTree
*   **Domain:** Motion Control, Kinematics, Dynamics, Control Theory

---
*(💡 진행 예정: Part 2에서는 위에서 설계한 C++ 제어 알고리즘 코어를 ROS2 아키텍처(Node, Topic)에 이식하고 분산 제어 시스템으로 고도화하는 과정을 진행합니다.)*
