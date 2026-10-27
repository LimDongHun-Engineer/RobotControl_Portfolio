#define _USE_MATH_DEFINES

#include <iostream>
#include <cmath>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

// [1] PID 제어기 클래스
class PIDController {
private:
	double Kp, Ki, Kd, dt;
	double integral_error, previous_error, output_limit;

public:
	double calculate(double target, double current) {
		double error = target - current;
		integral_error += error * dt;
		double derivative = (error - previous_error) / dt;
		double output = (Kp * error) + (Ki * integral_error) + (Kd * derivative);

		// Anti-windup
		if (output > output_limit) output = output_limit;
		if (output < -output_limit) output = -output_limit;

		previous_error = error;
		return output;
	}
};

// [2] 역기구학(IK) 해석기 클래스
class InverseKinematics {
private:
	double L1, L2;

public:
	InverseKinematics(double l1, double l2) : L1(l1), L2(l2) {}

	Vector2d getTargetAngles(double target_x, double target_y) {
		double distance = sqrt((target_x * target_x) + (target_y * target_y));
		if (distance > (L1 + L2)) {
			cout << "[에러] 목표 위치가 작업 반경을 벗어났습니다!" << endl;
			return Vector2d(0, 0);
		}
		double cos_theta2 = ((target_x * target_x) + (target_y * target_y) - (L1 * L1) - (L2 * L2)) / (2 * L1 * L2);
		double theta2_rad = acos(cos_theta2);

		double alpha = atan2(target_y, target_x);
		double beta = atan2(L2 * sin(theta2_rad), L1 + (L2 * cos(theta2_rad)));
		double theta1_rad = alpha - beta;

		return Vector2d(theta1_rad * (180.0 / M_PI), theta2_rad * (180.0 / M_PI));
	}
};
int main() {
    cout << "====================================================\n";
    cout << "[최종 Portfolio] 다관절 로봇 통합 제어 시뮬레이션\n";
    cout << "====================================================\n\n";

    // 1. 하드웨어 스펙 세팅 (Link 길이 10, 5)
    InverseKinematics robot_ik(10.0, 5.0);

    // 2. 1번 축과 2번 축의 PID 제어기 독립 생성 (주기 1ms, 최대 출력 10.0)
    PIDController axis1_pid(1.5, 0.05, 0.01, 0.001, 10.0);
    PIDController axis2_pid(1.8, 0.02, 0.01, 0.001, 10.0);

    // 3. 현재 모터 상태 (0도에서 시작)
    double current_theta1 = 0.0;
    double current_theta2 = 0.0;

    // 4. [제어 파이프라인 가동] 목표 좌표(X: 0, Y: 15) 입력
    cout << "[시스템] 목표 좌표 (X: 0, Y: 15) 지령 수신.\n";
    Vector2d target_angles = robot_ik.getTargetAngles(0.0, 15.0);
    cout << "[시스템] 역기구학 연산 완료 -> 1번 축 목표: " << target_angles.x() << "도, 2번 축 목표: " << target_angles.y() << "도\n\n";

    // 5. PID 제어 루프 (실제로는 무한 루프지만, 시뮬레이션을 위해 15번만 실행)
    cout << "--- PID 제어 추종 시작 ---\n";
    for (int step = 1; step <= 15; ++step) {
        // 각 축별로 필요한 제어량(출력) 계산
        double out1 = axis1_pid.calculate(target_angles.x(), current_theta1);
        double out2 = axis2_pid.calculate(target_angles.y(), current_theta2);

        // 간이 물리 모델: 모터가 출력만큼 즉시 회전한다고 가정
        current_theta1 += out1;
        current_theta2 += out2;

        cout << "Time " << step * 1 << "ms | "
            << "1번 축: " << current_theta1 << "도 (오차: " << target_angles.x() - current_theta1 << ") | "
            << "2번 축: " << current_theta2 << "도 (오차: " << target_angles.y() - current_theta2 << ")\n";
    }

    cout << "\n[시스템] 시뮬레이션 종료." << endl;
    return 0;
}






