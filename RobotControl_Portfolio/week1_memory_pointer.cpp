#include <iostream>
#include <cstdint>

using namespace std;

int main() {
	cout << "[Phase 1] 하드웨어 레지스터 메모리 제어 시뮬레이션\n" << endl;

	// 1. 가상의 하드웨어 레지스터 (PMAC의 P-변수 역할)
	// 로봇 엔지니어링에서는 하드웨어 스펙에 맞춰 32비트(int32_t) 자료형을 주로 사용합니다.
	int32_t raw_encoder_count = 10000;

	// 2. 하드웨어 주소를 매핑하는 포인터 선언 (PMAC의 M-변수 역할)
	// int32_t* 는 '32비트 정수가 들어있는 메모리 주소를 담는 변수'라는 뜻입니다.
	int32_t* p_encoder_register = &raw_encoder_count;

	cout << "--- 초기 상태(하드웨어 부팅 직후) --- " << endl;
	cout << "엔코더 메모리의 물리적 주소 : " << p_encoder_register << endl;
	cout << "현재 엔코더 값 (원본) : " << raw_encoder_count << " pulse\n" << endl;

	// 3. 포인터를 통한 하드웨어 메모리 직접 제어
	// *p_encoder_register는 "포인터가 가르키는 주소로 찾아가서 값을 바꿔라"는 명령어입니다.
	// 이는 모터 드라이버에 새로운 위치 명령을 하달하는 것과 같습니다.
	*p_encoder_register = 25000;

	cout << "--- 제어 명령 하달 후 (포인터로 메모리 값 변경) ---" << endl;
	cout << "포인터로 읽어온 엔코더 값   : " << *p_encoder_register << " pulse" << endl;
	cout << "실제 레지스터 값 (원본)     : " << raw_encoder_count << " pulse" << endl;

	// 결과적으로 원본 변수인 raw_encoder_count를 직접 건드리지 않고도, 
	// 주소(포인터)를 통해 값을 10000에서 25000으로 성공적으로 제어했습니다.

	return 0;
}