#include <iostream>
#include <fstream>
#include <cmath>
#include <iomanip>
#include <Eigen/Dense>

constexpr double PI = 3.14159265358979323846;

/**
* @brief 단일 관절/링크의 4x4 동차 변환 행렬(Homogeneous Transformation Matrix) 생성 함수
* @param theta_rad 관절 회전 각도 [rad]
* @param link_length 링크 길이 [m]
* @return Eigen::Matrix4d 4x4 동차 변환 행렬 T
*/
Eigen::Matrix4d createLinkTransform(double theta_rad, double link_length) {
	// 1. 4x4 단위 행렬(Identity Matrix)로 초기화
	// 맨 아랫줄 [0, 0, 0, 1]이 자동으로 세팅됨
	Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
	
	double c = std::cos(theta_rad);
	double s = std::sin(theta_rad);

	// 2. 3x3 회전 행렬(Rotation Matrix, Z축 회전) 구성
	Eigen::Matrix3d R;
	R << c, -s, 0.0,
		 s, c, 0.0,
	   0.0, 0.0, 1.0;

	// 3. 3x1 위치 벡터(Position Vector) 구성
	Eigen::Vector3d p(link_length * c, link_length * s, 0.0);

	// 4. 블록 연산(Block Operation)을 통해 4x4 동차 변환 행렬 T에 조립
	T.block<3, 3>(0, 0) = R;	// 좌측 상단 3x3 영역에 회전 행렬 대입
	T.block<3, 1>(0, 3) = p;	// 우측 상단 3x1 영역(4번째 열)에 위치 벡터 대입

	return T;
}

int main() {
	// 3자유도 로봇팔 링크 길이 설정 (단위: m)
	const double L1 = 1.0;
	const double L2 = 0.8;
	const double L3 = 0.5;

	// =========================================================================
	// Part 1. 단일 자세 정기구학(FK) 검증 (theta1 = 30도, theta2 = 45도, theta3 = -30도)
	// =========================================================================
	double q1 = 30.0 * PI / 180.0;
	double q2 = 45.0 * PI / 180.0;
	double q3 = -30.0 * PI / 180.0;

	// 각 링크의 상대 변환 행렬 생성
	Eigen::Matrix4d T01 = createLinkTransform(q1, L1);
	Eigen::Matrix4d T12 = createLinkTransform(q2, L2);
	Eigen::Matrix4d T23 = createLinkTransform(q3, L3);

	// 연쇄 법칙 (Chain Rule): Eigen의 행렬 곱셈 연산자(*) 활용
	Eigen::Matrix4d T02 = T01 * T12;	// 베이스 → 2번 관절 끝
	Eigen::Matrix4d T03 = T02 * T23;	// 베이스 → 3번 관절 끝 (End-Effector)

	// 최종 변환 행렬 T03에서 회전 행렬(R)과 위치 벡터(p)를 블록 연산으로 추출
	Eigen::Matrix3d R_end = T03.block<3, 3>(0, 0);
	Eigen::Vector3d p_end = T03.block<3, 1>(0, 3);

	std::cout << std::fixed << std::setprecision(4);
	std::cout << "=== [Eigen 기반 정기구학 단일 자세 검증] ===\n";
	std::cout << "1) 최종 동차 변환 행렬 T03 (4x4):\n" << T03 << "\n\n";
	std::cout << "2) 추출된 손끝 회전 행렬 R_end (3x3):\n" << R_end << "\n\n";
	std::cout << "3) 추출된 손끝 위치 벡터 p_end (X, Y, Z):\n"
		<< "   X = " << p_end.x() << " m, Y = " << p_end.y() << " m, Z = " << p_end.z() << " m\n\n";
	
	// =========================================================================
	// Part 2. 시뮬레이션 시각화용 연속 모션 궤적 생성 (100 프레임 -> CSV 저장)
	// =========================================================================
	std::ofstream csv_file("fk_trajectory.csv");
	if (!csv_file.is_open()) {
		std::cerr << "[오류] fk_trajectory.csv 파일을 생성할 수 없습니다. \n";
		return 1;
	}

	// CSV 헤더 작성
	csv_file << "step, x0, y0, x1, y1, x2, y2, x3, y3\n";

	const int total_steps = 100;
	for (int step = 0; step < total_steps; ++step) {
		double t = static_cast<double>(step) / total_steps;	// 0.0 ~ 1.0 정규화 시간

		// 시간에 따른 사인파 관절 각도 명령 생성
		double theta1 = (PI / 4.0) * std::sin(2.0 * PI * t) + (PI / 4.0);
		double theta2 = (PI / 3.0) * std::cos(2.0 * PI * t);
		double theta3 = (PI / 2.0) * std::sin(4.0 * PI * t);

		// 각 관절의 상대 동차 변환 행렬 계산
		Eigen::Matrix4d A1 = createLinkTransform(theta1, L1);
		Eigen::Matrix4d A2 = createLinkTransform(theta2, L2);
		Eigen::Matrix4d A3 = createLinkTransform(theta3, L3);

		// 베이스 좌표계{0} 기준 각 관절의 누적 변환 행렬 계산
		Eigen::Matrix4d T_joint1 = A1;
		Eigen::Matrix4d T_joint2 = T_joint1 * A2;
		Eigen::Matrix4d T_end = T_joint2 * A3;
		
		// 각 변환 행렬의 (0, 3) 원소(X좌표)와 (1, 3) 원소(Y좌표)를 추출하여 CSV 기록
				// Eigen 행렬 원소 접근법: 행렬이름(행인덱스, 열인덱스) -> 소괄호() 사용!
		csv_file << step << ","
			<< 0.0 << "," << 0.0 << ","
			<< T_joint1(0, 3) << "," << T_joint1(1, 3) << ","
			<< T_joint2(0, 3) << "," << T_joint2(1, 3) << ","
			<< T_end(0, 3) << "," << T_end(1, 3) << "\n";
	}

	csv_file.close();
	std::cout << "[완료] Eigen 엔진 기반 fk_trajectory.csv 파일 생성이 완료되었습니다!\n";
	return 0;
}
