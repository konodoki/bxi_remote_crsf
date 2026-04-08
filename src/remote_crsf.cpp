#include <cstdio>
#include <crsf_parser.hpp>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <communication/msg/motion_commands.hpp>
#include <mutex>
using namespace std::chrono_literals;
#define CRSR_MAX 1811
#define CRSR_MIN 174
static constexpr float CRSR_MID = (CRSR_MAX + CRSR_MIN) * 0.5f;
static constexpr float CRSR_SCALE = (CRSR_MAX - CRSR_MIN) * 0.5f;
float channels[16] = { 0 };
std::mutex channel_lock;
void crsf_callback(uint16_t channels_[])
{
    const std::lock_guard<std::mutex> guard(channel_lock);
    for (int i = 0; i < 16; i++) {
        channels[i] = (channels_[i] - CRSR_MID) / CRSR_SCALE;
        // printf("%.2f ", channels[i]);
    }
    // printf("\n");
}
void print_channel()
{
    for (int i = 0; i < 16; i++) {
        printf("%.2f ", channels[i]);
    }
    printf("\n");
}
#define VELX_CHANNEL (1)
#define VELY_CHANNEL (9)
#define VELR_CHANNEL (3)
#define SYSCTRL_CHANNEL (7)
#define MODE_CHANNEL (5)
#define PD_CHANNEL (8)

#define MIN_SPEED_X -1.0
#define MAX_SPEED_X 1.0
#define MIN_SPEED_Y -1.0
#define MAX_SPEED_Y 1.0
#define MIN_SPEED_R -1.0
#define MAX_SPEED_R 1.0
#define STAND_HEIGHT 1.0
class CRSFRemote : public rclcpp::Node {
public:
    CRSFRemote()
        : rclcpp::Node("CRSFRemote")
    {
        com_pub = this->create_publisher<communication::msg::MotionCommands>(
            "motion_commands", 20);
        timer_ = this->create_wall_timer(
            10ms, std::bind(&CRSFRemote::timer_callback, this));
    }

private:
    void timer_callback()
    {
        // 处理遥控器逻辑
        // 启动程序
        static bool launch_lock = false;
        if (channels[SYSCTRL_CHANNEL] > 0.5f) {
            if (launch_lock == false) {
                system("mkdir -p /var/log/bxi_log");
                system(
                    "ros2 launch bxi_example_py_elf3 example_launch_demo_hw.py > /var/log/bxi_log/$(date +%Y-%m-%d_%H-%M-%S)_elf.log  2>&1 &");
                system(
                    "ros2 launch bxi_example_bms bms.launch.py > /var/log/bxi_log/bms_$(date +%Y-%m-%d_%H-%M-%S)_bms.log 2>&1 &");
                reset_value();
                launch_lock = true;
                RCLCPP_INFO(this->get_logger(), "启动程序");
            }
        }
        // 停止程序
        if (channels[SYSCTRL_CHANNEL] < -0.5f) {
            if (launch_lock) {
                system("killall -SIGINT hardware_elf3");
                system("killall -SIGINT bxi_example_py_elf3");
                system("killall -SIGINT bxi_example_py_elf3_demo");
                system("killall -SIGINT bxi_bms");
                system("killall -SIGINT bxi_example_bms");
                launch_lock = false;
                reset_value();
                RCLCPP_INFO(this->get_logger(), "杀死程序");
            }
        }
        // pd模式
        static float pd_channel_last = channels[PD_CHANNEL];
        if (pd_channel_last < 0 && channels[PD_CHANNEL] > 0) {
            pd_brake_mode = !pd_brake_mode;
            RCLCPP_INFO(this->get_logger(), "切换为PD");
        }
        pd_channel_last = channels[PD_CHANNEL];
        // 走路模式切换
        static int mode_last = -1; //-1为固定零位 0为越障 1为跑步
        int mode_now = channels[MODE_CHANNEL] < -0.5 ?
                           -1 :
                           (channels[MODE_CHANNEL] > 0.5 ? 1 : 0);
        if (mode_now != mode_last) {
            if (mode_now < 0) {
                initial_pos_mode = !initial_pos_mode;
                RCLCPP_INFO(this->get_logger(), "切换为零位");
            } else if (mode_now == 0) {
                normal_mode = !normal_mode;
                RCLCPP_INFO(this->get_logger(), "切换为普通");
            } else if (mode_now > 0) {
                amp_run_mode = !amp_run_mode;
                RCLCPP_INFO(this->get_logger(), "切换为跑步");
            }
        }
        mode_last = mode_now;
        auto message = communication::msg::MotionCommands();
        { // initialize a ROS2 message
            const std::lock_guard<std::mutex> guard(channel_lock);
            velxy[0] = channels[VELX_CHANNEL];
            velxy[1] = channels[VELY_CHANNEL];
            velr = channels[VELR_CHANNEL];

            // 按定义最大速度缩放
            if (velxy[0] > 0) {
                velxy[0] *= MAX_SPEED_X;
            } else if (velxy[0] < 0) {
                velxy[0] *= -MIN_SPEED_X;
            }

            if (velxy[1] > 0) {
                velxy[1] *= MAX_SPEED_Y;
            } else if (velxy[1] < 0) {
                velxy[1] *= -MIN_SPEED_Y;
            }

            if (velr > 0) {
                velr *= MAX_SPEED_R;
            } else if (velr < 0) {
                velr *= -MIN_SPEED_R;
            }

            velxy_filt[0] = velxy[0] * 0.03 + velxy_filt[0] * 0.97;
            velxy_filt[1] = velxy[1] * 0.03 + velxy_filt[1] * 0.97;

            velr_filt = velr * 0.05 + velr_filt * 0.95;

            message.vel_des.x = velxy_filt[0];
            message.vel_des.y = velxy_filt[1];
            message.yawdot_des = velr_filt;
            // message.mode = mode;

            // RB组合键
            message.btn_1 = normal_mode ? 1 : 0;
            message.btn_2 = zero_torque_mode ? 1 : 0;
            message.btn_3 = pd_brake_mode ? 1 : 0;
            message.btn_4 = initial_pos_mode ? 1 : 0;

            // LB组合键
            message.btn_5 = dance_mode ? 1 : 0;
            message.btn_6 = host_mode ? 1 : 0;
            message.btn_7 = normal_run ? 1 : 0;
            message.btn_8 = amp_run_mode ? 1 : 0;

            // 纯按键
            message.btn_9 = dance_flag ? 1 : 0;

            message.height_des = STAND_HEIGHT;
        }

        com_pub->publish(message);
    }
    void reset_value()
    {
        const std::lock_guard<std::mutex> guard(channel_lock);
        memset(velxy, 0, sizeof(velxy));
        memset(velxy_filt, 0, sizeof(velxy_filt));
        velr_filt = 0;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<communication::msg::MotionCommands>::SharedPtr com_pub;
    double velxy[2] = { 0 }; // x y速度       (x,y speed)
    double velxy_filt[2] = { 0 }; // x y速度滤波值  (x,y speed filter)
    double velr = 0; // 旋转速度       (rotation speed)
    double velr_filt = 0;
    // 按下RB的变量
    bool normal_mode = false; // 按下改变状态，切换为普通模式，站立走路跑步
                              // (change to normal state,for stand run and walk)
    bool zero_torque_mode = false; // 按下改变状态，切换为零力模式 (change to
                                   // zero torque mode)
    bool pd_brake_mode = false; // 按下改变状态，切换为pd抱死模式 (change to
                                // zero torque mode)
    bool initial_pos_mode = false; // 按下改变状态，切换为初始位置模式 (set
                                   // motors to zero position)
    // 按下LB的变量
    bool host_mode = false; // 按下改变状态，切换为host起身模式 (change to host
                            // mode, for stand up)
    bool dance_mode = false; // 按下改变状态，切换为跳舞模式 (change to dance
                             // mode)
    bool normal_run = false;
    bool amp_run_mode = false;

    bool dance_flag = false; // 按下改变状态，暂停或继续跳舞               (stop
                             // or continue dancing)
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    CRSFParser crsf("/dev/ttyCRSF", 420000, crsf_callback);
    rclcpp::spin(std::make_shared<CRSFRemote>());
    rclcpp::shutdown();
    return 0;
}
