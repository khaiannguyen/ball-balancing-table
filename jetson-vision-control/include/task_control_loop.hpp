#ifndef TASK_CONTROL_LOOP_HPP
#define TASK_CONTROL_LOOP_HPP
#include <thread>
#include <atomic>
#include <memory>
#include <cstdint>
#include "pid_controller.hpp"
#include "trajectory.hpp"

/* TaskControlLoop (J7) — đọc system_state().ball (ghi bởi TaskBallDetect,
 * J6), chạy 2 PID độc lập (X->Roll_d, Y->Pitch_d), ghi kết quả vào
 * system_state().attitude_desired, nơi TaskCanTx đọc để gửi 0x204.
 *
 * Chạy 100Hz, cùng nhịp với TaskCanTx để dữ liệu luôn "tươi" khi TX đọc.
 *
 * An toàn: nếu ball.ball_detected == 0 HOẶC stm32_state_is_ok() == false,
 * PID được reset() và output ép về 0 (bàn phẳng) — không suy luận "mất
 * bóng" từ giá trị Ballx/Bally == 0, luôn kiểm tra cờ detected tường minh.
 *
 * THEM: nguon setpoint (x_d,y_d) gio phu thuoc mode doc tu
 * system_state().robot_state (field 'mode', ghi boi TaskCanRx khi parse
 * 0x103 ROBOT_STATE tu STM32):
 *   - opmode::BALANCE: setpoint do BalanceTrajectoryController TU SINH
 *     (giu tam O 2s -> chu ky Luc giac->Tron, xem trajectory.hpp).
 *   - opmode::POSITION: setpoint doc tu system_state().ball_desired (STM32
 *     gui qua 0x104 BALL_DESIRED, dieu khien boi UI/nut nhan STM32).
 *   - Cac mode khac (HOME/CALIB/MANUAL): setpoint co dinh (0,0) - an toan
 *     mac dinh, khong chay trajectory. */
class TaskControlLoop {
public:
    TaskControlLoop() = default;
    ~TaskControlLoop();

    TaskControlLoop(const TaskControlLoop&)            = delete;
    TaskControlLoop& operator=(const TaskControlLoop&) = delete;

    // Gain mặc định CỐ TÌNH nhỏ + out_limit_deg nhỏ — dùng cho lần test
    // đầu tiên trên phần cứng thật (bước 1-2 quy trình test an toàn J7).
    // Tune tăng dần theo quy trình đã ghi trong tài liệu tiến độ.
    bool start(float kp = 0.1f, float ki = 0.0f, float kd = 0.015f,
               float out_limit_deg = 3.0f);
    void stop();

    bool is_running() const { return running_.load(std::memory_order_relaxed); }

private:
    void run();

    std::unique_ptr<PIDController> pid_x_; // Ballx -> Roll_d (xác nhận ánh xạ trục thật khi test)
    std::unique_ptr<PIDController> pid_y_; // Bally -> Pitch_d
    std::unique_ptr<std::thread>   thread_;
    std::atomic<bool> running_{false};

    // THEM: quan ly quy dao tu dong cho Mode Balance mo rong.
    BalanceTrajectoryController balance_traj_;
    // THEM: theo doi mode CHU KY TRUOC, de phat hien thoi diem VUA CHUYEN
    // VAO Mode Balance -> goi balance_traj_.reset() dung 1 lan luc do (bat
    // dau lai tu HOLD_CENTER moi lan vao mode, khong reset lien tuc moi
    // chu ky - se lam trajectory khong bao gio chay qua duoc HOLD_CENTER).
    // Gia tri khoi tao 0xFF (khong khop bat ky opmode:: nao hop le 0-4) de
    // dam bao lan dau tien vao Mode Balance CHAC CHAN duoc phat hien la
    // "vua chuyen vao", du gia tri that su cua mode ban dau la gi.
    uint8_t last_mode_ = 0xFF;

    // THEM (fix bug: circle bi bo qua do mat bong 1 frame thoang qua):
    // dem thoi gian MAT BONG/MAT STM32 LIEN TUC (giay). Chi reset
    // balance_traj_ khi thoi gian nay VUOT NGUONG kBallLossResetThresholdS,
    // KHONG reset ngay tu frame dau tien mat bong nua. PID van reset() +
    // output ep ve 0 NGAY LAP TUC moi frame mat bong (khong doi - AN TOAN
    // khong doi), CHI CO hanh vi reset TRAJECTORY la duoc "khoan dung"
    // hon voi nhieu thoang qua cua camera (vd 1-2 frame loi detect do anh
    // sang/goc nhin, khong phai that su mat bong/mat ket noi).
    float ball_loss_elapsed_s_ = 0.f;
    static constexpr float kBallLossResetThresholdS = 0.3f; // TODO: chinh
        // neu can nhay/cham hon. 0.3s = du dai de loc nhieu 1-2 frame
        // camera (thuong ~10-33ms/frame), du ngan de van an toan (bong
        // that su mat -> trajectory reset trong <1 chu ky dwell ngan nhat).
};

#endif