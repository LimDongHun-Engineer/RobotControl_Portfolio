#include <iostream>

using namespace std;

// 1. 모터 제어를 위한 '설계도(Class)' 정의
class Motor {
private:
	// 정보 은닉(Encapsulation) : 외부에서 함부로 직접 조작하면 안 되는 모터의 내부 상태 파라미터
	int motor_id;
	double current_position;
	double target_position;
	bool is_servo_on;

public:
	// 2. 생산자 (Constructor) : 모터 객체가 처음 만들어질 때 초기값을 셋팅합니다.
	Motor(int id) {
		motor_id = id;
		current_position = 0.0;
		target_position = 0.0;
		is_servo_on = false;	// 기본적으로 서보는 꺼진 상태로 언전하게 시작
	}

	// 3. 서보 온/오프 제어 함수
	void setServo(bool state) {
		is_servo_on = state;
		cout << "[축 " << motor_id << "] 서보 상태 변경: " << (state ? "ON" : "OFF") << endl;
	}

	// 4. 절대 위치 이동 지령 함수 (안전 예외 처리 포함)
	void moveAbsolute(double target) {
		if (!is_servo_on) {
			cout << "[축 " << motor_id << "] 에러: 서보가 꺼져 있어 지령을 내릴 수 없습니다!" << endl;
			return;
		}
		target_position = target;
		cout << "[축 " << motor_id << "] 위치 이동 명령 하달 -> 목표: " << target_position << endl;

		// 기초 단계이므로 궤적 생성 없이 즉시 도달했다고 가상 처리
		current_position = target_position;
	}

	// 5. 현재 상태 모니터링 함수
	void printStatus() {
		cout << "  -> 축 " << motor_id << " 현재 위치: " << current_position
			<< " (Servo: " << (is_servo_on ? "ON" : "OFF") << ")" << endl;
	}
};

int main() {
	cout << "[기초 3] 객체지향 설계(Class)를 이용한 다축 모터 제어기 추상화\n" << endl;

	// PMAC의 #1, #2, #3 축을 클래스 기반의 독립적인 C++ 객체로 생성
	Motor axisX(1);
	Motor axisY(2);
	Motor axisZ(3);

	cout << "--- 장비 초기화 및 서보 온 ---" << endl;
	axisX.setServo(true);
	axisY.setServo(true);
	// z축은 작업자의 실수로 서보를 켜지 않았다고 가정해 봅시다.

	cout << "\n--- 다축 이동 명령 테스트 ---" << endl;
	axisX.moveAbsolute(150.5);
	axisY.moveAbsolute(300.0);

	// z축은 서보 오프 상태이므로 클래스 내부의 방어 로직에 의해 에러 메시지가 출력되어야 함
	axisZ.moveAbsolute(50.0);

	cout << "\n--- 현재 다축 시스템 전체 상태 모니터링 ---" << endl;
	axisX.printStatus();
	axisY.printStatus();
	axisZ.printStatus();

	return 0;
}