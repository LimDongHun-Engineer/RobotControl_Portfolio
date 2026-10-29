#define _USE_MATH_DEFINES  // 반드시 <cmath> 보다 위에 있어야 합니다.

#include <iostream>
#include <cmath>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

// 1. 2자유도 평면 로봇 기구학 클래스 설계
class ForwardKinematics2DOF {
private:
	double L1;	// 첫 번째 뼈대(Link 1)의 길이
	double L2;	// 두 번째 뼈대(Link 2)의 길이

public:
	// 생성자: 로봇 팔의 물리적 길이 세팅
	ForwardKinematics2DOF(double link1_len, double link2_len) {
		L1 = link1_len;
		L2 = link2_len;
	}

	// 2. 정기구학(FK) 계산 함수: 모터 각도(Degree)를 입력받아 끝단 좌표(X, Y)반환
	Vector2d calculateFK(double theta1_deg, double theta2_deg) {
		// [핵심 1] Degree → Radian 변환
		// M_PI는 <cmath>에서 제공하는 원주율(3.141592...) 상수입니다.
		double theta1_rad = theta1_deg * (M_PI / 180.0);
		double theta2_rad = theta2_deg * (M_PI / 180.0);

		// [핵심 2] 정기구학 수학 공식 적용 (L1*cos(t1) + L2*cos(t1+t2))
		double x = L1 * cos(theta1_rad) + L2 * cos(theta1_rad + theta2_rad);
		double y = L1 * sin(theta1_rad) + L2 * sin(theta1_rad + theta2_rad);

		// [핵심 3] 계산된 x, y 좌표를 Eigen의 2차원 벡터 객체로 묶어서 반환
		Vector2d end_effecgtor_pos(x, y);

		return end_effecgtor_pos;
	}
};

int main() {
	cout << "[Portfolio Phase 3] 2자유도 로봇 팔 정기구학(FK) 해석기\n" << endl;

	// Link 1 길이가 10.0, Link 2 길이가 5.0 인 가상의 로봇 팔 생성
	ForwardKinematics2DOF robot_arm(10.0, 5.0);

	// 테스트 케이스 1: 두 관절이 모두 0도일 때 (일자로 쭉 뻗은 상태)
		// 예상 X: 15.0, 예상 Y: 0.0
	Vector2d pos1 = robot_arm.calculateFK(0.0, 0.0);
	cout << "테스트 1 (0도, 0도) -> X: " << pos1.x() << ", Y: " << pos1.y() << endl;

	// 테스트 케이스 2: 1번 축이 90도 서있고, 2번 축은 그대로인 상태 (위로 뻗은 상태)
	// 예상 X: 0.0, 예상 Y: 15.0
	Vector2d pos2 = robot_arm.calculateFK(90.0, 0.0);
	// 컴퓨터의 부동소수점 오차로 인해 X가 완전한 0이 아니라 0.000000000000000X 처럼 나올 수 있습니다.
	cout << "테스트 2 (90도, 0도) -> X: " << pos2.x() << ", Y: " << pos2.y() << endl;

	// 테스트 케이스 3: 1번 축 45도, 2번 축 -45도 회전
	Vector2d pos3 = robot_arm.calculateFK(45.0, -45.0);
	cout << "테스트 3 (45도, -45도) -> X: " << pos3.x() << ", Y: " << pos3.y() << endl;

	return 0;
}