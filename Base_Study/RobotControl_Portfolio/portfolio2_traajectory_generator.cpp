#include <iostream>
#include <fstream>
#include <cmath>

using namespace std;

// 1. 사다리꼴 궤적 생성기 클래스
class TrapezoidalTrajectory {
private:
    double max_vel; // 최대 속도 (Pulse/sec)
    double max_acc; // 최대 가속도 (Pulse/sec^2)

public:
    TrapezoidalTrajectory(double velocity, double acceleration) {
        max_vel = velocity;
        max_acc = acceleration;
    }

    // 2. 이동 지령에 따른 궤적 계산 및 CSV 출력 함수
    void generateTrajectory(double target_distance, double dt, const string& filename) {
        // CSV 파일 저장 준비
        ofstream csv_file(filename);
        csv_file << "Time(s),Command_Position,Command_Velocity\n"; // 엑셀 헤더

        // 궤적 생성용 변수 계산
        double t_acc = max_vel / max_acc; // 가속에 걸리는 시간
        double d_acc = 0.5 * max_acc * t_acc * t_acc; // 가속 구간 이동 거리

        double t_flat, d_flat;

        // 거리가 짧아 최대 속도에 도달하지 못하는 경우 (삼각형 궤적)
        if (target_distance < 2 * d_acc) {
            t_acc = sqrt(target_distance / max_acc);
            d_acc = 0.5 * max_acc * t_acc * t_acc;
            t_flat = 0;
            d_flat = 0;
        }
        else {
            // 거리가 충분하여 등속 구간이 있는 경우 (사다리꼴 궤적)
            d_flat = target_distance - (2 * d_acc);
            t_flat = d_flat / max_vel;
        }

        double total_time = (2 * t_acc) + t_flat;
        cout << "[궤적 정보] 총 이동 시간: " << total_time << "초 (가속: " << t_acc << "초, 등속: " << t_flat << "초)" << endl;

        // 3. dt(예: 1ms) 간격으로 궤적 데이터 쪼개기
        for (double t = 0; t <= total_time; t += dt) {
            double current_pos = 0.0;
            double current_vel = 0.0;

            if (t <= t_acc) {
                // [가속 구간]
                current_pos = 0.5 * max_acc * t * t;
                current_vel = max_acc * t;
            }
            else if (t <= t_acc + t_flat) {
                // [등속 구간]
                current_pos = d_acc + max_vel * (t - t_acc);
                current_vel = max_vel;
            }
            else {
                // [감속 구간]
                double t_dec = t - (t_acc + t_flat);
                current_pos = d_acc + d_flat + (max_vel * t_dec) - (0.5 * max_acc * t_dec * t_dec);
                current_vel = max_vel - (max_acc * t_dec);
            }

            // CSV 파일에 기록
            csv_file << t << "," << current_pos << "," << current_vel << "\n";
        }

        csv_file.close();
        cout << "[저장 완료] " << filename << " 파일이 생성되었습니다. 엑셀에서 확인해 보세요." << endl;
    }
};

int main() {
    cout << "[Portfolio Phase 2] 사다리꼴 속도 프로파일 궤적 생성기\n" << endl;

    // 최대 속도 500, 최대 가속도 1000으로 설정
    TrapezoidalTrajectory traj_gen(500.0, 1000.0);

    // 1000만큼 이동하는 궤적을 0.001초(1ms) 주기로 쪼개어 CSV에 저장
    traj_gen.generateTrajectory(1000.0, 0.001, "trajectory_output.csv");

    return 0;
}