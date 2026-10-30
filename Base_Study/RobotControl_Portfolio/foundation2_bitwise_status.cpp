#include <iostream>
#include <cstdint>

using namespace std;

int main() {
	cout << "[기초 2] 비트 연산을 활용한 하드웨어 상태(Status) 파싱\n" << endl;

	// 1. 가상의 모터 상태 레지스터 값 (PMAC에서 통신으로 읽어온 32비트 원시 데이터)
	// 십진수 41을 이진수로 바꾸면 '0000...0010 1001' 입니다.
	uint32_t motor_status_register = 41;

	// 2. 비트 마스크 정의 (하드웨어 매뉴얼에 따라 상수로 정의)
	const uint32_t SERVO_ON_BIT		= (1 << 0);	// 0번 비트 : 서보 온 상태
	const uint32_t IN_POS_BIT		= (1 << 3);	// 3번 비트 : 목표 위치 도달(In-Position)
	const uint32_t AMP_FAULT_BIT	= (1 << 5);	// 5번 비트 : 앰프 에러 발생

	cout << "--- 모터 상태 점검 시작 ---\n" << endl;

    // 3. 비트 AND(&) 연산으로 개별 상태 분리 및 확인
    if (motor_status_register & SERVO_ON_BIT) {
        cout << "[정상] 서보 모터가 켜져 있습니다. (Servo ON)" << endl;
    }
    else {
        cout << "[대기] 서보 모터가 꺼져 있습니다. (Servo OFF)" << endl;
    }

    if (motor_status_register & IN_POS_BIT) {
        cout << "[알림] 모터가 목표 위치에 도달했습니다. (In-Position)" << endl;
    }

    if (motor_status_register & AMP_FAULT_BIT) {
        cout << "[경고] 앰프 알람이 발생했습니다! 즉시 정지합니다. (Amp Fault)" << endl;
    }
    else {
        cout << "[정상] 앰프 에러 없음." << endl;
    }

    // 4. 비트 OR(|) 연산으로 제어 명령 강제 덮어쓰기
    cout << "\n--- 알람 리셋(Fault Reset) 명령 하달 ---" << endl;
    const uint32_t FAULT_RESET_CMD = (1 << 7);  // 7번 비트가 리셋 명령이라고 가정

    // 기존 상태를 유지한 채로 7번 비트만 1로 켜서 덮어씁니다.
    motor_status_register = motor_status_register | FAULT_RESET_CMD;
    cout << "리셋 명령 전송 완료. 업데이트된 레지스터 값: " << motor_status_register << endl;

    return 0;
}
