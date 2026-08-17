#include "task_control_loop.hpp"
#include "system_state.hpp"
#include "operating_mode.hpp"
#include <cstdio>
#include <chrono>
#include "task_watchdog.hpp"

bool TaskControlLoop::start(float kp, float ki, float kd, float out_limit_deg) {
    if (running_.load(std::memory_order_relaxed)) {
        std::fprintf(stderr, "TaskControlLoop: da chay roi\n");
        return false;
    }

    pid_x_ = std::make_unique<PIDController>(kp, ki, kd, -out_limit_deg, out_limit_deg);
    pid_y_ = std::make_unique<PIDController>(kp, ki, kd, -out_limit_deg, out_limit_deg);

    // THEM (giu nguyen tu ban ban gui): wire deadband + loc velocity vao
    // PIDController. deadband=1.5mm: Ballx/Bally trong log bi luong tu hoa
    // ve so nguyen mm, +-1mm la nhieu do binh thuong khi bong da dung yen
    // gan tam. alpha=0.3: loc EMA them cho vx/vy truoc khi nhan kd_, phong
    // khi loc EMA o task_ball_detect (neu co) chua du manh/chua ton tai.
    // TODO (J8): tune lai 2 gia tri nay tren phan cung that, dac biet neu
    // van con dao dong sau khi bong bi nhieu that (khong phai nhieu do).
    pid_x_->set_error_deadband(1.5f);
    pid_x_->set_velocity_filter_alpha(0.3f);
    pid_y_->set_error_deadband(1.5f);
    pid_y_->set_velocity_filter_alpha(0.3f);

    running_.store(true, std::memory_order_relaxed);
    thread_ = std::make_unique<std::thread>(&TaskControlLoop::run, this);
    std::printf("TaskControlLoop: da start (kp=%.4f ki=%.4f kd=%.4f out_limit=+-%.2fdeg)\n",
                kp, ki, kd, out_limit_deg);
    return true;
}

void TaskControlLoop::stop() {
    if (!running_.load(std::memory_order_relaxed)) return;
    running_.store(false, std::memory_order_relaxed);
    if (thread_ && thread_->joinable()) thread_->join();
    std::printf("TaskControlLoop: da dung.\n");
}

TaskControlLoop::~TaskControlLoop() {
    stop();
}

void TaskControlLoop::run() {
    using clock = std::chrono::steady_clock;
    auto next_wake = clock::now();
    const auto period = std::chrono::milliseconds(10); // 100Hz, khop nhip TaskCanTx

    auto last_time = clock::now();

    struct sched_param sp;
    sp.sched_priority = 80;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
        std::fprintf(stderr, "TaskControlLoop: khong set duoc SCHED_FIFO "
                     "(can CAP_SYS_NICE/sudo), chay priority mac dinh\n");
    }

    while (running_.load(std::memory_order_relaxed)) {
        next_wake += period;

        auto now = clock::now();
        float dt = std::chrono::duration<float>(now - last_time).count();
        last_time = now;

        int16_t x, y, vx, vy;
        uint8_t detected;
        ball_state_read(system_state().ball, x, y, vx, vy, detected);

        bool safe_to_run = (detected != 0) && stm32_state_is_ok();

        float roll_d = 0.f, pitch_d = 0.f;

        // Doc mode hien tai tu STM32 (0x103 ROBOT_STATE, da duoc TaskCanRx
        // parse san vao system_state().robot_state).
        uint8_t mode, robot_bits;
        telemetry_robot_state_read(system_state().robot_state, mode, robot_bits);

        // Phat hien thoi diem VUA CHUYEN VAO Mode Balance (tu mode khac,
        // hoac lan dau khoi dong) -> reset trajectory ve HOLD_CENTER, dam
        // bao MOI LAN vao Mode Balance deu bat dau lai dung tu dau (giu tam
        // 2s roi moi chay quy dao), khong tiep tuc dang do o giua quy dao
        // cu neu da tung chay truoc do roi chuyen sang mode khac.
        if (mode == opmode::BALANCE && last_mode_ != opmode::BALANCE) {
            balance_traj_.reset();
            ball_loss_elapsed_s_ = 0.f; // THEM: dong bo lai bo dem khi vua vao mode moi
        }
        last_mode_ = mode;

        // Chon nguon setpoint (x_d, y_d mm) theo mode hien tai.
        float setpoint_x_mm = 0.f, setpoint_y_mm = 0.f;
        float setpoint_height_mm = 0.f; // THEM (Ke hoach 2): setpoint chieu
                                         // cao, mac dinh 0 - CHI mode BALANCE
                                         // moi co the khac 0 (pha BOUNCE).
        if (mode == opmode::BALANCE) {
            // Mode Balance: Jetson TU SINH setpoint theo quy dao
            // (HOLD_CENTER -> Luc giac -> Tron -> Tang bong tai tam -> lap
            // lai, xem trajectory.hpp).
            Waypoint wp = balance_traj_.update(dt);
            setpoint_x_mm = wp.x_mm;
            setpoint_y_mm = wp.y_mm;
            setpoint_height_mm = wp.height_mm; // SUA: doc height tu trajectory
                                                // (truoc day luon = 0)
        } else if (mode == opmode::POSITION) {
            // Mode Position: setpoint do STM32/nguoi dung nut nhan quyet
            // dinh, nhan qua 0x104 BALL_DESIRED (TaskCanRx da ghi san vao
            // system_state().ball_desired). STM32 la nguon su that duy
            // nhat cho setpoint nay o mode nay - Jetson chi doc lai.
            int16_t x_d, y_d, height_d_unused;
            ball_desired_read(system_state().ball_desired, x_d, y_d, height_d_unused);
            setpoint_x_mm = (float)x_d;
            setpoint_y_mm = (float)y_d;
        } else {
            // Cac mode khac (HOME/CALIB/MANUAL): setpoint co dinh (0,0) -
            // gia tri an toan mac dinh, KHONG chay trajectory/khong doc
            // ball_desired trong cac mode nay.
            setpoint_x_mm = 0.f;
            setpoint_y_mm = 0.f;
        }

        if (safe_to_run) {
            // XAC NHAN DAU khi test tren phan cung: neu ban nghieng SAI
            // chieu (day bong ra xa hon), dao dau o day (vd doi thanh
            // -pid_x_->update(...)), KHONG sua trong PIDController.
            //
            // update_with_velocity(): D-term dung THANG vx/vy do duoc tu
            // task_ball_detect (da loc EMA, tinh theo khoang thoi gian
            // THAT giua 2 lan detect thay), roi duoc loc EMA THEM 1 lan
            // nua ben trong PIDController (set_velocity_filter_alpha o
            // start()) + deadband cho error (set_error_deadband) - xem
            // giai thich chi tiet trong pid_controller.hpp.
            // Anh xa truc: vx di cung x (-> pitch_d qua pid_x_), vy di
            // cung y (-> roll_d qua pid_y_) — GIU NGUYEN anh xa cheo truc
            // nhu code goc, khong doi them.
            //
            // Setpoint gio la setpoint_x_mm/setpoint_y_mm (dong, theo
            // mode) thay vi hang so 0.f hay ball_desired co dinh.
            pitch_d = pid_x_->update_with_velocity(setpoint_x_mm, (float)x, (float)vx, dt);
            roll_d  = pid_y_->update_with_velocity(setpoint_y_mm, (float)y, (float)vy, dt);
        } else {
            pid_x_->reset();
            pid_y_->reset();
            roll_d = 0.f;
            pitch_d = 0.f;

            // SUA (fix bug: Circle bi bo qua do mat bong 1 frame thoang
            // qua - xem log data.csv thuc te, dung luc bong vua toi diem
            // CUOI cua luc giac (F) truoc khi vao Circle thi mat bong
            // dung 1 frame (~10-20ms), kich hoat reset() ngay lap tuc,
            // keo trajectory ve lai O, phai chay lai TOAN BO ~34s chuoi
            // luc giac tu dau).
            //
            // GIU NGUYEN AN TOAN: PID van reset + roll_d/pitch_d ep ve 0
            // NGAY frame nay (khong doi gi ca - ban van nam phang ngay
            // khi mat bong/mat STM32, dung tinh than an toan cu).
            //
            // CHI DOI cach xu ly balance_traj_: cong don thoi gian mat
            // bong LIEN TUC (ball_loss_elapsed_s_), chi goi reset() khi
            // vuot nguong kBallLossResetThresholdS (0.3s) - tuc mat bong
            // THAT SU keo dai, khong phai nhieu 1-2 frame camera thoang
            // qua. Neu duoi nguong, GIU NGUYEN trang thai/vi tri hien tai
            // trong chuoi (seq_index_/sub_phase_/phase_elapsed_s_ khong
            // doi) - khi bong xuat hien tro lai ngay sau do, trajectory
            // tiep tuc DUNG CHO ngay tai diem dang do, khong phai chay
            // lai tu dau.
            ball_loss_elapsed_s_ += dt;
            if (ball_loss_elapsed_s_ >= kBallLossResetThresholdS) {
                balance_traj_.reset();
            }
        }

        // Bong da duoc phat hien lai (safe_to_run true o chu ky nay) ->
        // xoa bo dem mat bong lien tuc, san sang cho lan mat bong tiep
        // theo (neu co) duoc tinh lai tu 0.
        if (safe_to_run) {
            ball_loss_elapsed_s_ = 0.f;
        }

        // SUA (Ke hoach 2 - buoc 2, tang bong tai tam): height_d gio DOC
        // TU setpoint_height_mm (trajectory sinh ra trong pha BOUNCE, cac
        // pha/mode khac van la 0 nhu cu) - KHONG con hardcode 0.0f nua.
        // roll_d/pitch_d KHONG doi (PID x/y van dieu khien binh thuong,
        // giu bong tai tam trong luc height dao dong).
        attitude_desired_write(system_state().attitude_desired, roll_d, pitch_d, setpoint_height_mm);

        std::this_thread::sleep_until(next_wake);
        task_alive_mark(ALIVE_BIT_CONTROL_LOOP);
    }
}