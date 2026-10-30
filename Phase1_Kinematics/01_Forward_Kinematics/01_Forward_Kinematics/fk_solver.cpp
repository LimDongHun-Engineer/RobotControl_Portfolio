#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <iomanip>

constexpr double PI = 3.14159265358979323846;

// 1. 4x4 동차 변환 행렬(Homogeneous Transformation Matrix) 구조체 정의
struct Mat4 {
	double m[4][4];		// 4x4 행렬 정의

	// 생성자: 기본적으로 단위 행렬(Identity Matrix)로 초기화
	Mat4() {
		for (int i = 0; i < 4; ++i) {
			for (int j = 0; j < 4; ++j) {
				m[i][j] = (i == j) ? 1.0 : 0.0;		// i == j 인 부분은 1, 아닌 부분은 0
			}
		}
	}

	// 연산자 오버로딩: 행렬 곱셈 (T_total = T1 * T2)
	Mat4 operator*(const Mat4& other) const {
		Mat4 result;
		for (int i = 0; i < 4; ++i) {
			for (int j = 0; j < 4; ++j) {
				result.m[i][j] = 0.0;
				for (int k = 0; k < 4; ++k) {
					result.m[i][j] += this->m[i][k] * other.m[k][j];
				}
			}
		}
		return result;
	}

	// 디버깅용 행렬 출력 함수
	void print(const std::string& name) const {
		std::cout << "=== " << name << " ===\n";
		for (int i = 0; i < 4; ++i) {
			std::cout << "[ ";
			for (int j = 0; j < 4; ++j) {
				std::cout << std::setw(8) << std::fixed << std::setprecision(4) << m[i][j] << " ";
			}
			std::cout << "]\n";
		}
		std::cout << "\n";
	}
};

// 2. 단일 링크의 2D/3D(Z축 회전) 동차 변환 행렬 생성 함수
Mat4 createLinkTransform(double theta_rad, double link_length) {
	Mat4 T;
	double c = std::cos(theta_rad);
	double s = std::sin(theta_rad);

	// 회전 행렬 R 파트 (3x3)
	T.m[0][0] = c;	T.m[0][1] = -s;	T.m[0][1] = 0.0;
	T.m[1][0] = s;	T.m[1][1] = c;	T.m[2][1] = 0.0;
	T.m[2][0] = 0.0;	T.m[2][1] = 0.0;	T.m[2][1] = 1.0;

	// 위치 벡터 p 파트 (3x1)
	T.m[0][3] = link_length * c;
	T.m[1][3] = link_length * s;
	T.m[2][3] = 0.0;

	// 동차 좌표 더미 행 (1x4)
	T.m[3][0] = 0.0; T.m[3][1] = 0.0; T.m[3][2] = 0.0; T.m[3][3] = 1.0;

	return T;
}

int main() {
	// 3자유도 로봇팔 링크 길이 설정 (단위: m)
	const double L1 = 1.0;
	const double L2 = 0.8;
	const double L3 = 0.5;

	// 1) 단일 자세 검증 : theta1 = 30도, theta2 = 45도, theta3 = -30도
	double q1 = 30.0 * PI / 180.0;
	double q2 = 45.0 * PI / 180.0;
	double q3 = -30.0 * PI / 180.0;

	Mat4 T01 = createLinkTransform(q1, L1);
	Mat4 T12 = createLinkTransform(q2, L2);
	Mat4 T23 = createLinkTransform(q3, L3);

	// 연쇄 법칙(Chain Rule)을 통한 베이스 대비 각 관절의 절대 변환 행렬 계산
	Mat4 T02 = T01 * T12;		// 베이스 → 2번 관절 끝
	Mat4 T03 = T02 * T03;		// 베이스 → 3번 관절 끝 (End-Effector)

	std::cout << "[정기구학 단일 자세 검증 결과]\n";
	T03.print("T03 (End-Effector Pose)");
	std::cout << "-> 손끝 최종 위치 (X, Y): (" << T03.m[0][3] << " m, " << T03.m[1][3] << " m)\n\n";
	
	// 2) 시뮬레이션 시각화용 연속 모션 데이터 생성 (100 프레임)
	std::ofstream csv_file("fk_trajectory.csv");
	if (!csv_file.is_open()) {
		std::cerr << "CSV 파일을 생성할 수 없습니다.\n";
		return 1;
	}

	// CSV 헤더 작성: 시간, 베이스(x0,y0), 관절1(x1,y1), 관절2(x2,y2), 손끝(x3,y3)
	csv_file << "step,x0,y0,x1,y1,x2,y2,x3,y3\n";
	const int total_steps = 100;
	for (int step = 0; step < total_steps; ++step) {
		double t = static_cast<double>(step) / total_steps;	// 0.0 ~ 1.0 정규화 시간

		// 시간에 따라 부드럽게 변화하는 사인파 관절 궤적 명령 인가
		double theta1 = (PI / 4.0) * std::sin(2.0 * PI * t) + (PI / 4.0);
		double theta2 = (PI / 3.0) * std::cos(2.0 * PI * t);
		double theta3 = (PI / 2.0) * std::sin(4.0 * PI * t);

		Mat4 A1 = createLinkTransform(theta1, L1);
		Mat4 A2 = createLinkTransform(theta2, L2);
		Mat4 A3 = createLinkTransform(theta3, L3);

		Mat4 T_joint1 = A1;
		Mat4 T_joint2 = T_joint1 * A2;
		Mat4 T_end = T_joint2 * A3;

		// 각 관절의 절대 X, Y 좌표(4번째 열의 0행, 1행 원소) 추출 및 기록
		csv_file << step << ","
			<< 0.0 << "," << 0.0 << ","
			<< T_joint1.m[0][3] << "," << T_joint1.m[1][3] << ","
			<< T_joint2.m[0][3] << "," << T_joint2.m[1][3] << ","
			<< T_end.m[0][3] << "," << T_end.m[1][3] << "\n";
	}

	csv_file.close();
	std::cout << "[완료] fk_trajectory.csv 파일이 성공적으로 생성되었습니다! (총 100 프레임)\n";
	return 0;
}