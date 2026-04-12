#include <chrono>
#include <communication/msg/detail/battery_states__struct.hpp>
#include <communication/msg/motion_commands.hpp>
#include <communication/msg/battery_states.hpp>
#include <crsf_parser.hpp>
#include <cstdio>
#include <functional>
#include <geometry_msgs/msg/detail/twist_stamped__struct.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/executors.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "com_publisher.hpp"
std::shared_ptr<COMPublisher> com_publisher;
std::shared_ptr<CRSFParser> crsf_paser;
using namespace std::chrono_literals;
#define CRSR_MAX 1811
#define CRSR_MIN 174
static constexpr float CRSR_MID = (CRSR_MAX + CRSR_MIN) * 0.5f;
static constexpr float CRSR_SCALE = (CRSR_MAX - CRSR_MIN) * 0.5f;
float channels[16] = { 0 };
std::mutex channel_lock;
std::chrono::high_resolution_clock::time_point crsf_last_rec_time;
void crsf_callback(uint16_t channels_[])
{
    crsf_last_rec_time = std::chrono::high_resolution_clock::now();
    const std::lock_guard<std::mutex> guard(channel_lock);
    for (int i = 0; i < 16; i++) {
        channels[i] = (channels_[i] - CRSR_MID) / CRSR_SCALE;
        // printf("%.2f ", channels[i]);
    }
    // printf("\n");
}
#define CRSF_VELX_CHANNEL (1)
#define CRSF_VELY_CHANNEL (0)
#define CRSF_VELR_CHANNEL (3)
#define CRSF_NAVCTL_CHANNEL (4)
#define CRSF_SYSCTRL_CHANNEL (7)
#define CRSF_MODE_CHANNEL (5)
#define CRSF_PD_CHANNEL (8)

#define CRSF_MIN_SPEED_X -1.0
#define CRSF_MAX_SPEED_X 1.0
#define CRSF_MIN_SPEED_Y -1.0
#define CRSF_MAX_SPEED_Y 1.0
#define CRSF_MIN_SPEED_R -1.0
#define CRSF_MAX_SPEED_R 1.0
#define CRSF_STAND_HEIGHT 1.0
#define CRSF_AXIS_DEAD_ZONE 0.05
class CRSFRemote : public rclcpp::Node {
public:
    CRSFRemote()
        : rclcpp::Node("CRSFRemote")
    {
        com_pub = this->create_publisher<communication::msg::MotionCommands>(
            "motion_commands", 20);
        timer_ = this->create_wall_timer(
            10ms, std::bind(&CRSFRemote::timer_callback, this));

        heartbeat_timer_ = this->create_wall_timer(
            1s, std::bind(&CRSFRemote::heartbeat_timer_callback, this));
        vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "cmd_vel", 10,
            std::bind(&CRSFRemote::velcmd_callback, this,
                      std::placeholders::_1));
        bat_sub_ = this->create_subscription<communication::msg::BatteryStates>(
            "battery_states", 10,
            std::bind(&CRSFRemote::bat_callback, this, std::placeholders::_1));
    }

private:
    communication::msg::BatteryStates lastest_bat_msg_;
    void heartbeat_timer_callback()
    {
        if (crsf_paser == nullptr)
            return;
        crsf_paser->send_battery(lastest_bat_msg_.voltage * 10,
                                 lastest_bat_msg_.current * 10,
                                 (100 - lastest_bat_msg_.soc) * (10000),
                                 lastest_bat_msg_.soc);
    }
    void bat_callback(communication::msg::BatteryStates::SharedPtr msg)
    {
        lastest_bat_msg_ = *msg;
    }
    void timer_callback()
    {
        // 处理遥控器逻辑
        double connect_time =
            std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::high_resolution_clock::now() - crsf_last_rec_time)
                .count();
        if (connect_time > 3) {
            if (is_crsf_connected) {
                is_crsf_connected = false;
                RCLCPP_INFO(this->get_logger(), "遥控器断连");
            }
        } else {
            if (!is_crsf_connected) {
                is_crsf_connected = true;
                RCLCPP_INFO(this->get_logger(), "遥控器重连");
            }
        }
        communication::msg::MotionCommands &message = motion_cmd_messages;
        if (is_crsf_connected) {
            // 启动程序
            static bool launch_lock = false;
            if (channels[CRSF_SYSCTRL_CHANNEL] > 0.5f) {
                if (launch_lock == false) {
                    system("mkdir -p /var/log/bxi_log");
                    system(
                        "ros2 launch bxi_example_py_elf3 example_launch_demo_hw.py > "
                        "/var/log/bxi_log/$(date +%Y-%m-%d_%H-%M-%S)_elf.log  2>&1 &");
                    system(
                        "ros2 launch bxi_example_bms bms.launch.py > "
                        "/var/log/bxi_log/bms_$(date +%Y-%m-%d_%H-%M-%S)_bms.log 2>&1 &");
                    reset_value();
                    launch_lock = true;
                    RCLCPP_INFO(this->get_logger(), "启动程序");
                }
            }
            // 停止程序
            if (channels[CRSF_SYSCTRL_CHANNEL] < -0.5f) {
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
            static float pd_channel_last = channels[CRSF_PD_CHANNEL];
            if (pd_channel_last < -0.5 && channels[CRSF_PD_CHANNEL] > 0.5) {
                pd_brake_mode = !pd_brake_mode;
                RCLCPP_INFO(this->get_logger(), "切换为PD");
            }
            pd_channel_last = channels[CRSF_PD_CHANNEL];
            // 走路模式切换
            static int mode_last = -1; //-1为固定零位 0为越障 1为跑步
            int mode_now = channels[CRSF_MODE_CHANNEL] < -0.5 ?
                               -1 :
                               (channels[CRSF_MODE_CHANNEL] > 0.5 ? 1 : 0);
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
            { // initialize a ROS2 message
                const std::lock_guard<std::mutex> guard(channel_lock);
                if (channels[CRSF_NAVCTL_CHANNEL] < -0.5) {
                    velxy[0] = channels[CRSF_VELX_CHANNEL];
                    velxy[1] = -channels[CRSF_VELY_CHANNEL];
                    velr = -channels[CRSF_VELR_CHANNEL];
                    velxy[0] = fabs(velxy[0]) > CRSF_AXIS_DEAD_ZONE ? velxy[0] :
                                                                      0;
                    velxy[1] = fabs(velxy[1]) > CRSF_AXIS_DEAD_ZONE ? velxy[1] :
                                                                      0;
                    velr = fabs(velr) > CRSF_AXIS_DEAD_ZONE ? velr : 0;

                    // 按定义最大速度缩放
                    if (velxy[0] > 0) {
                        velxy[0] *= CRSF_MAX_SPEED_X;
                    } else if (velxy[0] < 0) {
                        velxy[0] *= -CRSF_MIN_SPEED_X;
                    }

                    if (velxy[1] > 0) {
                        velxy[1] *= CRSF_MAX_SPEED_Y;
                    } else if (velxy[1] < 0) {
                        velxy[1] *= -CRSF_MIN_SPEED_Y;
                    }

                    if (velr > 0) {
                        velr *= CRSF_MAX_SPEED_R;
                    } else if (velr < 0) {
                        velr *= -CRSF_MIN_SPEED_R;
                    }
                    velxy_filt[0] = velxy[0] * 0.03 + velxy_filt[0] * 0.97;
                    velxy_filt[1] = velxy[1] * 0.03 + velxy_filt[1] * 0.97;
                    velr_filt = velr * 0.05 + velr_filt * 0.95;
                    message.vel_des.x = velxy_filt[0];
                    message.vel_des.y = velxy_filt[1];
                    message.yawdot_des = velr_filt;
                } else if (channels[CRSF_NAVCTL_CHANNEL] > 0.5) {
                    // 导航模式没有滤波
                    message.vel_des.x = velxy[0];
                    message.vel_des.y = velxy[1];
                    message.yawdot_des = velr_filt;
                }
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

                message.height_des = CRSF_STAND_HEIGHT;
                // 为了兼容手柄
                com_publisher->motion_cmd_messages = message;
                com_publisher->normal_mode = message.btn_1 == 1;
                com_publisher->zero_torque_mode = message.btn_2 == 1;
                com_publisher->pd_brake_mode = message.btn_3 == 1;
                com_publisher->initial_pos_mode = message.btn_4 == 1;
                com_publisher->dance_mode = message.btn_5 == 1;
                com_publisher->host_mode = message.btn_6 == 1;
                com_publisher->normal_run = message.btn_7 == 1;
                com_publisher->amp_run_mode = message.btn_8 == 1;
                com_publisher->dance_flag = message.btn_9 == 1;
            }
        } else {
            // 遥控器断连则使用摇杆控制
            if (com_pub == nullptr)
                return;
            // 为了兼容手柄
            message = com_publisher->motion_cmd_messages;
            normal_mode = message.btn_1 == 1;
            zero_torque_mode = message.btn_2 == 1;
            pd_brake_mode = message.btn_3 == 1;
            initial_pos_mode = message.btn_4 == 1;
            dance_mode = message.btn_5 == 1;
            host_mode = message.btn_6 == 1;
            normal_run = message.btn_7 == 1;
            amp_run_mode = message.btn_8 == 1;
            dance_flag = message.btn_9 == 1;
        }
        com_pub->publish(message);
    }
    void velcmd_callback(geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        if (channels[CRSF_NAVCTL_CHANNEL] > 0.5 && is_crsf_connected) {
            velxy[0] = msg->twist.linear.x;
            velxy[1] = msg->twist.linear.y;
            velr = msg->twist.angular.z;
        }
    }
    void reset_value()
    {
        const std::lock_guard<std::mutex> guard(channel_lock);
        memset(velxy, 0, sizeof(velxy));
        memset(velxy_filt, 0, sizeof(velxy_filt));
        velr_filt = 0;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    rclcpp::Publisher<communication::msg::MotionCommands>::SharedPtr com_pub;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
    rclcpp::Subscription<communication::msg::BatteryStates>::SharedPtr bat_sub_;
    communication::msg::MotionCommands motion_cmd_messages;
    bool is_crsf_connected = false;
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
    crsf_paser =
        std::make_shared<CRSFParser>("/dev/ttyCRSF", 420000, crsf_callback);
    com_publisher = std::make_shared<COMPublisher>("/dev/input/js0");
    std::shared_ptr<CRSFRemote> crsf_remote = std::make_shared<CRSFRemote>();
    rclcpp::executors::MultiThreadedExecutor multi_threaded_executor;
    multi_threaded_executor.add_node(com_publisher);
    multi_threaded_executor.add_node(crsf_remote);
    multi_threaded_executor.spin();
    rclcpp::shutdown();
    return 0;
}
