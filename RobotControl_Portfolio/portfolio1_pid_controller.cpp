#include <iostream>

using namespace std;

// 1. PID 제어기 클래스 설계
class PIDController {
private:
	double Kp, Ki, Kd;
	double dt;				// 제어 주기 (초 단위, ex: 0.001s = 1ms)
	double integral_error;	// 오차의 누적값 (I 제어용)
	double previous_error;	// 이전 주기의 오차 (D 제어용)
	double output_limit;	// Anti-windup을 위한 모터 출력 상한선

public:
	// 2. 제어기 초기화 (생성자)
	PIDController(double p, double i, double d, double time_step, double limit) {
		Kp = p;
		Ki = i;
		Kd = d;
		dt = time_step;
		output_limit = limit;
		integral_error = 0.0;
		previous_error = 0.0;
	}

	// 3. 매 제어 주기(예: 1ms)마다 실행되는 연산 함수
	double calculate(double target_position, double current_position) {
		// P 제어: 현재 오차
		double error = target_position - current_position;

		// I 제어: 오차의 누적 (오차 * 시간)
		integral_error += error * dt;

		// D 제어: 오차의 변화율 ((현재 오차 - 이전 오차) / 시간)
		double derivative = (error - previous_error) / dt;

		// PID 총 출력량 계산
		double output = (Kp * error) + (Ki * integral_error) + (Kd * derivative);

		// 4. 협업 필수 로직: Anti-windup 및 하드웨어 한계 처리
		// 모터가 낼 수 있는 최대 출력 (PWM/전압)을 제한하여 시스템을 보호합니다.
		if (output > output_limit) {
			output = output_limit;
		}
		else if (output < -output_limit) {
			output = -output_limit;
		}

		// 다음 주기의 D 제어를 위해 현재 오차를 저장
		previous_error = error;

		return output;
	}
};

int main() {
	cout << "[Portfolio Phase 1] 이산시간 PID 제어기 및 Anti-windup 구현\n" << endl;

	// PMAC 서보 튜닝하듯 게인과 스펙 설정
	// (Kp=1.5, Ki=0.1, Kd=0.05), 제어주기 1ms(0.001s), 모터 최대 출력 10.0 제한
	PIDController axis1_pid(1.5, 0.1, 0.05, 0.001, 10.0);

	double current_pos = 0.0;	// 현재 엔코더 위치
	double target_pos = 100.0;	// 목표 위치 지령

	cout << "--- 1ms 주기 PID 제어 10-Step 시뮬레이션 ---" << endl;

	// 10번의 제어 루프(총 10ms)를 돌며 모터가 목표를 향해 가는 과정 시뮬레이션
	for (int step = 1; step <= 10; ++step) {
		// 제어기에서 모터 드라이버에 인가할 제어 출력(PWM) 계산
		double control_signal = axis1_pid.calculate(target_pos, current_pos);

		// 물리적인 모터가 구동되었다고 가정 (간이 모델: 출력만큼 위치가 변함)
		current_pos += control_signal;

		cout << "Step " << step << " ( " << step * 1 << "ms )"
			<< " | 목표: " << target_pos
			<< " | 현재: " << current_pos
			<< " | 제어 출력: " << control_signal << endl;
	}

	return 0;
}

