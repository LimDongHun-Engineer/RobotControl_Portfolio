#define _USE_MATH_DEFINES // 원주율(M_PI) 사용을 위한 매크로 (반드시 맨 위)

#include <iostream>
#include <cmath>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

class InverseKinematics2DOF {
private:
	double L1;
	double L2;

public:
	InverseKinematics2DOF(double link1_len, double link2_len) {
		L1 = link1_len;
		L2 = link2_len;
	}
	
	// 목표 좌표 X, Y를 주면 필요한 모터 각도(theta1, theta2)를 Vector2d로 반환
	Vector2d calculateIK(double target_x, double target_y) {
		// 방어적 프로그래밍: 목표 지점이 로봇 팔이 닿을 수 없는 거리인지 검사
		double distance = sqrt((target_x * target_x) + (target_y * target_y));
		if (distance > (L1 + L2)) {
			cout << "[에러] 목표 위치가 로봇 팔의 최대 작업 반경(" << L1 + L2 << ")을 벗어났습니다!" << endl;
			return Vector2d(0, 0);	// 에러 발생 시 0도로 정지
		}

		// [Step 2] 제2 코사인 법칙을 이용해 theta2 계산
		double cos_theta2 = ((target_x * target_x) + (target_y * target_y) - (L1 * L1) - (L2 * L2)) / (2 * L1 * L2);
		double theta2_rad = acos(cos_theta2);

		// [Step 3] theta1 계산을 위한 알파, 베타 계산 (atan2 사용)
		double alpha = atan2(target_y, target_x);
		double beta = atan2(L2 * sin(theta2_rad), L1 + (L2 * cos(theta2_rad)));
		double theta1_rad = alpha - beta;

		// [최종 처리] Radian 값을 모터가 이해하기 쉬운 Degree 값으로 변환
		double theta1_deg = theta1_rad * (180.0 / M_PI);
		double theta2_deg = theta2_rad * (180.0 / M_PI);

		return Vector2d(theta1_deg, theta2_deg);
	}
};

int main() {
	cout << "[Portfolio Phase 4] 2자유도 로봇 팔 역기구학(IK) 제어기\n" << endl;

	// L1 = 10.0, L2 = 5.0 인 로봇 팔 (이전 정기구학과 동일 스펙)
	InverseKinematics2DOF robot(10.0, 5.0);

	// 테스트 1: (X: 15.0, Y: 0.0) -> 쭉 뻗은 상태
	// 예상 결과: 모터1 0도, 모터2 0도
	Vector2d joint_angles1 = robot.calculateIK(15.0, 0.0);
	cout << "목표(15, 0) 이동 명령 -> 모터1: " << joint_angles1.x() << "도, 모터2: " << joint_angles1.y() << "도" << endl;

	// 테스트 2: (X: 0.0, Y: 15.0) -> 위로 뻗은 상태
	// 예상 결과: 모터1 90도, 모터2 0도
	Vector2d joint_angles2 = robot.calculateIK(0.0, 15.0);
	cout << "목표(0, 15) 이동 명령 -> 모터1: " << joint_angles2.x() << "도, 모터2: " << joint_angles2.y() << "도" << endl;

	// 테스트 3: 팔이 절대 닿을 수 없는 곳 (방어 로직 테스트)
	cout << "\n[안전 테스트] 작업 반경 이탈 시도:" << endl;
	robot.calculateIK(20.0, 20.0);

	return 0;
}