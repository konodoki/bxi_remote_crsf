#include <fcntl.h>
#include <linux/joystick.h>
#include <unistd.h>

#include <chrono>
#include <communication/msg/actuator_states.hpp>
#include <communication/msg/battery_states.hpp>
#include <communication/msg/detail/actuator_states__struct.hpp>
#include <communication/msg/motion_commands.hpp>
#include <crsf_parser.hpp>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mutex>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <thread>

using namespace std::chrono_literals;
using namespace std;

// ===================== 通用宏定义 =====================
// 速度限制
#define MIN_SPEED_X -1.0
#define MAX_SPEED_X 1.0
#define MIN_SPEED_Y -1.0
#define MAX_SPEED_Y 1.0
#define MIN_SPEED_R -1.0
#define MAX_SPEED_R 1.0

// 死区定义
#define AXIS_DEAD_ZONE 1000
#define CRSF_AXIS_DEAD_ZONE 0.05
#define AXIS_VALUE_MAX 32767

// 高度参数
#define STAND_HEIGHT 1.0
#define STAND_HEIGHT_MIN 1.0
#define STAND_HEIGHT_MAX 3.0

// ===================== 手柄(JS)宏定义 =====================
#if 0 // PS4 JS
#define JS_VELX_AXIS 4
#define JS_VELX_AXIS_DIR -1
#define JS_VELY_AXIS 0
#define JS_VELY_AXIS_DIR -1
#define JS_VELR_AXIS 6
#define JS_VELR_AXIS_DIR -1

#define JS_STOP_BT 10
#define JS_GAIT_STAND_BT 0
#define JS_GAIT_WALK_BT 2
#define JS_HEIGHT_UPPER_BT 1
#define JS_HEIGHT_LOWER_BT 3
#define JS_MODE_BT 5
#define JS_START_BT 9

#else // XBOX JS
#define JS_VELX_AXIS 3
#define JS_VELX_AXIS_DIR -1
#define JS_VELY_AXIS 0
#define JS_VELY_AXIS_DIR -1
#define JS_VELR_AXIS 6
#define JS_VELR_AXIS_DIR -1

#define JS_STOP_BT 11 // 终止程序
#define JS_START_BT 14 // 启动程序
#define JS_LB_BT 6 // LB功能
#define JS_RB_BT 7 // RB功能
#define JS_SWITCH_X 3 // 暂停/继续跳舞 | 组合LB:跳舞 | 组合RB:普通模式
#define JS_SWITCH_A 0 // 组合LB:HOST起身 | 组合RB:零力
#define JS_SWITCH_B 1 // 组合LB:amp台阶 | 组合RB:pd模式
#define JS_SWITCH_Y 4 // 组合LB:amp行走 | 组合RB:初始位置模式
#define JS_START2_BT 14
#endif

// ===================== CRSF宏定义 =====================
#define CRSR_MAX 1811
#define CRSR_MIN 174
static constexpr float CRSR_MID = (CRSR_MAX + CRSR_MIN) * 0.5f;
static constexpr float CRSR_SCALE = (CRSR_MAX - CRSR_MIN) * 0.5f;

#define CRSF_VELX_CHANNEL (1)
#define CRSF_VELY_CHANNEL (0)
#define CRSF_VELR_CHANNEL (3)
#define CRSF_SPEED_RANGE_CHANNEL (9)
#define CRSF_NAVCTL_CHANNEL (4)
#define CRSF_SYSCTRL_CHANNEL (7)
#define CRSF_MODE_CHANNEL (5)
#define CRSF_PD_CHANNEL (8)

// ===================== 统一节点类 =====================
class RemoteControlNode : public rclcpp::Node {
public:
    RemoteControlNode(std::string crsf_path, std::string js_path)
        : Node("remote_control_node")
        , js_path_(js_path)
    {
        // 初始化发布者
        motion_cmd_pub_ =
            this->create_publisher<communication::msg::MotionCommands>(
                "motion_commands", 20);

        // 初始化定时器
        control_timer_ = this->create_wall_timer(
            10ms, std::bind(&RemoteControlNode::control_timer_callback, this));
        heartbeat_timer_ = this->create_wall_timer(
            1s, std::bind(&RemoteControlNode::heartbeat_timer_callback, this));

        motor_tmp_timer_ = this->create_wall_timer(
            1s, std::bind(&RemoteControlNode::motor_timer_callback, this));

        // 初始化订阅者
        vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "cmd_vel", 10,
            std::bind(&RemoteControlNode::velcmd_callback, this,
                      std::placeholders::_1));
        bat_sub_ = this->create_subscription<communication::msg::BatteryStates>(
            "battery_states", 10,
            std::bind(&RemoteControlNode::bat_callback, this,
                      std::placeholders::_1));
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        qos.best_effort();
        motor_sub_ =
            this->create_subscription<communication::msg::ActuatorStates>(
                "hardware/actuator_states", qos,
                std::bind(&RemoteControlNode::motor_callback, this,
                          std::placeholders::_1));
        // 初始化CRSF
        crsf_parser_ = std::make_shared<CRSFParser>(
            crsf_path, 420000,
            std::bind(&RemoteControlNode::crsf_callback, this,
                      std::placeholders::_1));

        // 初始化手柄
        js_fd_ = open(js_path_.c_str(), O_RDONLY);
        if (js_fd_ < 0) {
            RCLCPP_WARN(this->get_logger(),
                        "Failed to open joystick, will retry...");
        }
        js_loop_thread_ = std::thread(&RemoteControlNode::js_loop, this);

        // 初始化状态
        reset_all_values();
    }

    ~RemoteControlNode()
    {
        // 资源清理
        if (js_fd_ > 0)
            close(js_fd_);
        if (js_loop_thread_.joinable())
            js_loop_thread_.join();
    }

private:
    // 发布/订阅/定时器
    rclcpp::Publisher<communication::msg::MotionCommands>::SharedPtr
        motion_cmd_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    rclcpp::TimerBase::SharedPtr motor_tmp_timer_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
    rclcpp::Subscription<communication::msg::BatteryStates>::SharedPtr bat_sub_;
    rclcpp::Subscription<communication::msg::ActuatorStates>::SharedPtr
        motor_sub_;

    // CRSF相关
    std::shared_ptr<CRSFParser> crsf_parser_;
    float crsf_channels_[16] = { 0 };
    std::chrono::high_resolution_clock::time_point crsf_last_rec_time_;
    bool is_crsf_connected_ = false;
    communication::msg::BatteryStates lastest_bat_msg_;
    communication::msg::ActuatorStates lastest_motor_msg_;
    std::chrono::high_resolution_clock::time_point lastest_motor_msg_time;
    std::chrono::high_resolution_clock::time_point lastest_bat_msg_time;
    // 手柄相关
    std::string js_path_;
    int js_fd_ = -1;
    std::thread js_loop_thread_;
    double js_axis_[20] = { 0 };
    bool LB_press_ = false;
    bool RB_press_ = false;

    // 控制状态
    std::mutex state_mutex_;
    double velxy_[2] = { 0 }; // 原始速度
    double velxy_filt_[2] = { 0 }; // 滤波后速度
    double velr_ = 0; // 原始旋转速度
    double velr_filt_ = 0; // 滤波后旋转速度
    double stand_height_ = STAND_HEIGHT;
    double height_filt_ = STAND_HEIGHT;
    bool launch_lock_ = false; // 程序启动锁

    // 模式
    bool normal_mode_ = false; // 普通模式
    bool zero_torque_mode_ = false; // 零力模式
    bool pd_brake_mode_ = false; // PD抱死模式
    bool initial_pos_mode_ = false; // 初始位置模式
    bool host_mode_ = false; // HOST起身模式
    bool dance_mode_ = false; // 跳舞模式
    bool normal_run_ = false; // 普通行走
    bool amp_run_mode_ = false; // AMP行走
    bool dance_flag_ = false; // 跳舞暂停/继续

    // ========== CRSF回调 ==========
    void crsf_callback(uint16_t channels[])
    {
        const std::lock_guard<std::mutex> guard(state_mutex_);
        crsf_last_rec_time_ = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < 16; i++) {
            crsf_channels_[i] = (channels[i] - CRSR_MID) / CRSR_SCALE;
        }
    }

    // ========== 手柄循环 ==========
    void js_loop()
    {
        struct js_event event;
        while (rclcpp::ok()) {
            ssize_t len = read(js_fd_, &event, sizeof(event));

            if (len == sizeof(event)) {
                const std::lock_guard<std::mutex> guard(state_mutex_);
                if (event.type & JS_EVENT_AXIS) {
                    js_axis_[event.number] = event.value;
                } else if (event.type & JS_EVENT_BUTTON) {
                    handle_js_button(event.number, event.value);
                }
            } else {
                // 手柄断开重连
                RCLCPP_WARN(this->get_logger(),
                            "Joystick disconnected, retrying...");
                close(js_fd_);
                while (rclcpp::ok() && js_fd_ < 0) {
                    js_fd_ = open(js_path_.c_str(), O_RDONLY);
                    if (js_fd_ < 0)
                        sleep(1);
                }
                RCLCPP_INFO(this->get_logger(), "Joystick reconnected");
            }
        }
    }

    // ========== 手柄按键处理 ==========
    void handle_js_button(int btn_num, int value)
    {
        if (value) { // 按键按下
            switch (btn_num) {
            case JS_STOP_BT:
                stop_robot_program();
                launch_lock_ = false;
                reset_all_values();
                break;
            case JS_START_BT:
                if (!launch_lock_) {
                    start_robot_program();
                    launch_lock_ = true;
                    reset_all_values();
                } else {
                    RCLCPP_WARN(this->get_logger(), "Program already running!");
                }
                break;
            case JS_LB_BT:
                LB_press_ = true;
                break;
            case JS_RB_BT:
                RB_press_ = true;
                break;
            case JS_SWITCH_X:
                if (LB_press_) {
                    dance_mode_ = !dance_mode_;
                    RCLCPP_INFO(this->get_logger(), "Dance mode: %d",
                                dance_mode_);
                } else if (RB_press_) {
                    normal_mode_ = !normal_mode_;
                    RCLCPP_INFO(this->get_logger(), "Normal mode: %d",
                                normal_mode_);
                } else {
                    dance_flag_ = !dance_flag_;
                    RCLCPP_INFO(this->get_logger(), "Dance flag: %d",
                                dance_flag_);
                }
                break;
            case JS_SWITCH_Y:
                if (LB_press_) {
                    amp_run_mode_ = !amp_run_mode_;
                    RCLCPP_INFO(this->get_logger(), "AMP run mode: %d",
                                amp_run_mode_);
                } else if (RB_press_) {
                    initial_pos_mode_ = !initial_pos_mode_;
                    RCLCPP_INFO(this->get_logger(), "Initial pos mode: %d",
                                initial_pos_mode_);
                }
                break;
            case JS_SWITCH_A:
                if (LB_press_) {
                    host_mode_ = !host_mode_;
                    RCLCPP_INFO(this->get_logger(), "Host mode: %d",
                                host_mode_);
                } else if (RB_press_) {
                    zero_torque_mode_ = !zero_torque_mode_;
                    RCLCPP_INFO(this->get_logger(), "Zero torque mode: %d",
                                zero_torque_mode_);
                }
                break;
            case JS_SWITCH_B:
                if (LB_press_) {
                    normal_run_ = !normal_run_;
                    RCLCPP_INFO(this->get_logger(), "Normal run mode: %d",
                                normal_run_);
                } else if (RB_press_) {
                    pd_brake_mode_ = !pd_brake_mode_;
                    RCLCPP_INFO(this->get_logger(), "PD brake mode: %d",
                                pd_brake_mode_);
                }
                break;
            default:
                break;
            }
        } else { // 按键释放
            switch (btn_num) {
            case JS_LB_BT:
                LB_press_ = false;
                break;
            case JS_RB_BT:
                RB_press_ = false;
                break;
            default:
                break;
            }
        }
    }

    // ========== 控制定时器回调（核心逻辑） ==========
    void control_timer_callback()
    {
        communication::msg::MotionCommands msg;
        const std::lock_guard<std::mutex> guard(state_mutex_);

        // 检查CRSF连接状态
        check_crsf_connection();

        // 优先使用CRSF，断开时使用手柄
        if (is_crsf_connected_) {
            handle_crsf_control(msg);
        } else {
            handle_js_control(msg);
        }

        // 填充模式状态
        fill_mode_state(msg);

        // 发布消息
        motion_cmd_pub_->publish(msg);
    }

    // ========== 检查CRSF连接 ==========
    void check_crsf_connection()
    {
        double connect_time =
            std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::high_resolution_clock::now() - crsf_last_rec_time_)
                .count();

        if (connect_time > 3) {
            if (is_crsf_connected_) {
                is_crsf_connected_ = false;
                RCLCPP_INFO(this->get_logger(),
                            "CRSF disconnected, switch to joystick");
            }
        } else {
            if (!is_crsf_connected_) {
                is_crsf_connected_ = true;
                RCLCPP_INFO(this->get_logger(), "CRSF connected");
            }
        }
    }

    // ========== 处理CRSF控制 ==========
    void handle_crsf_control(communication::msg::MotionCommands &msg)
    {
        // 启动/停止程序
        handle_crsf_sysctrl();

        // PD模式切换
        static float pd_last = crsf_channels_[CRSF_PD_CHANNEL];
        if (pd_last < -0.5 && crsf_channels_[CRSF_PD_CHANNEL] > 0.5) {
            pd_brake_mode_ = !pd_brake_mode_;
            RCLCPP_INFO(this->get_logger(), "CRSF: PD brake mode %d",
                        pd_brake_mode_);
        }
        pd_last = crsf_channels_[CRSF_PD_CHANNEL];

        // 模式切换（零位/普通/跑步）
        static int mode_last = -1;
        int mode_now = crsf_channels_[CRSF_MODE_CHANNEL] < -0.5 ?
                           -1 :
                           (crsf_channels_[CRSF_MODE_CHANNEL] > 0.5 ? 1 : 0);
        if (mode_now != mode_last) {
            if (mode_now < 0) {
                initial_pos_mode_ = !initial_pos_mode_;
                RCLCPP_INFO(this->get_logger(), "CRSF: Initial pos mode %d",
                            initial_pos_mode_);
            } else if (mode_now == 0) {
                normal_mode_ = !normal_mode_;
                RCLCPP_INFO(this->get_logger(), "CRSF: Normal mode %d",
                            normal_mode_);
            } else if (mode_now > 0) {
                amp_run_mode_ = !amp_run_mode_;
                RCLCPP_INFO(this->get_logger(), "CRSF: AMP run mode %d",
                            amp_run_mode_);
            }
            mode_last = mode_now;
        }

        // 速度控制
        if (crsf_channels_[CRSF_NAVCTL_CHANNEL] < -0.5) {
            // 手动模式（带滤波）
            velxy_[0] = crsf_channels_[CRSF_VELX_CHANNEL];
            velxy_[1] = -crsf_channels_[CRSF_VELY_CHANNEL];
            velr_ = -crsf_channels_[CRSF_VELR_CHANNEL];

            // 死区处理
            velxy_[0] = fabs(velxy_[0]) > CRSF_AXIS_DEAD_ZONE ? velxy_[0] : 0;
            velxy_[1] = fabs(velxy_[1]) > CRSF_AXIS_DEAD_ZONE ? velxy_[1] : 0;
            velr_ = fabs(velr_) > CRSF_AXIS_DEAD_ZONE ? velr_ : 0;

            // 速度缩放
            scale_speed();
            // 滤波
            velxy_filt_[0] = velxy_[0] * 0.03 + velxy_filt_[0] * 0.97;
            velxy_filt_[1] = velxy_[1] * 0.03 + velxy_filt_[1] * 0.97;
            velr_filt_ = velr_ * 0.05 + velr_filt_ * 0.95;

            msg.vel_des.x = velxy_filt_[0];
            msg.vel_des.y = velxy_filt_[1];
            msg.yawdot_des = velr_filt_;
        } else if (crsf_channels_[CRSF_NAVCTL_CHANNEL] > 0.5) {
            // 导航模式（无滤波）
            msg.vel_des.x = velxy_[0];
            msg.vel_des.y = velxy_[1];
            msg.yawdot_des = velr_filt_;
        }

        msg.height_des = STAND_HEIGHT;
    }

    // ========== 处理手柄 ==========
    void handle_js_control(communication::msg::MotionCommands &msg)
    {
        // 读取手柄轴数据
        velxy_[0] =
            (js_axis_[JS_VELX_AXIS] * JS_VELX_AXIS_DIR) / AXIS_VALUE_MAX;
        velxy_[1] =
            (js_axis_[JS_VELY_AXIS] * JS_VELY_AXIS_DIR) / AXIS_VALUE_MAX;
        velr_ = (js_axis_[JS_VELR_AXIS] * JS_VELR_AXIS_DIR) / AXIS_VALUE_MAX;

        // 死区处理
        velxy_[0] = fabs(velxy_[0]) >
                            (AXIS_DEAD_ZONE / (double)AXIS_VALUE_MAX) ?
                        velxy_[0] :
                        0;
        velxy_[1] = fabs(velxy_[1]) >
                            (AXIS_DEAD_ZONE / (double)AXIS_VALUE_MAX) ?
                        velxy_[1] :
                        0;
        velr_ =
            fabs(velr_) > (AXIS_DEAD_ZONE / (double)AXIS_VALUE_MAX) ? velr_ : 0;

        // 速度缩放
        scale_speed();

        // 滤波
        velxy_filt_[0] = velxy_[0] * 0.03 + velxy_filt_[0] * 0.97;
        velxy_filt_[1] = velxy_[1] * 0.03 + velxy_filt_[1] * 0.97;
        velr_filt_ = velr_ * 0.05 + velr_filt_ * 0.95;

        // 填充速度
        msg.vel_des.x = velxy_filt_[0];
        msg.vel_des.y = velxy_filt_[1];
        msg.yawdot_des = velr_filt_;

        // 高度滤波
        height_filt_ = height_filt_ * 0.9 + stand_height_ * 0.1;
        msg.height_des = height_filt_;
    }

    // ========== 填充模式状态到消息 ==========
    void fill_mode_state(communication::msg::MotionCommands &msg)
    {
        // RB组合键
        msg.btn_1 = normal_mode_ ? 1 : 0;
        msg.btn_2 = zero_torque_mode_ ? 1 : 0;
        msg.btn_3 = pd_brake_mode_ ? 1 : 0;
        msg.btn_4 = initial_pos_mode_ ? 1 : 0;

        // LB组合键
        msg.btn_5 = dance_mode_ ? 1 : 0;
        msg.btn_6 = host_mode_ ? 1 : 0;
        msg.btn_7 = normal_run_ ? 1 : 0;
        msg.btn_8 = amp_run_mode_ ? 1 : 0;

        // 纯按键
        msg.btn_9 = dance_flag_ ? 1 : 0;
    }

    // ========== CRSF启动/停止 ==========
    void handle_crsf_sysctrl()
    {
        // 启动程序
        if (crsf_channels_[CRSF_SYSCTRL_CHANNEL] > 0.5f && !launch_lock_) {
            start_robot_program();
            launch_lock_ = true;
            RCLCPP_INFO(this->get_logger(), "CRSF: Start robot program");
        }

        // 停止程序
        if (crsf_channels_[CRSF_SYSCTRL_CHANNEL] < -0.5f && launch_lock_) {
            stop_robot_program();
            launch_lock_ = false;
            RCLCPP_INFO(this->get_logger(), "CRSF: Stop robot program");
        }
    }

    // ========== 速度缩放 ==========
    void scale_speed()
    {
        double speed_range =
            crsf_channels_[CRSF_SPEED_RANGE_CHANNEL] + 1; // 0~2
        if (velxy_[0] > 0)
            velxy_[0] *= MAX_SPEED_X * speed_range;
        else if (velxy_[0] < 0)
            velxy_[0] *= -MIN_SPEED_X;

        if (velxy_[1] > 0)
            velxy_[1] *= MAX_SPEED_Y * speed_range;
        else if (velxy_[1] < 0)
            velxy_[1] *= -MIN_SPEED_Y;

        if (velr_ > 0)
            velr_ *= MAX_SPEED_R;
        else if (velr_ < 0)
            velr_ *= -MIN_SPEED_R;
    }

    // ========== 启动机器人程序 ==========
    void start_robot_program()
    {
        system("mkdir -p /var/log/bxi_log");
        system("ros2 launch bxi_example_py_elf3 example_launch_demo_hw.py > "
               "/var/log/bxi_log/$(date +%Y-%m-%d_%H-%M-%S)_elf.log  2>&1 &");
        system(
            "ros2 launch bxi_example_bms bms.launch.py > "
            "/var/log/bxi_log/bms_$(date +%Y-%m-%d_%H-%M-%S)_bms.log 2>&1 &");
    }

    // ========== 停止机器人程序 ==========
    void stop_robot_program()
    {
        system("killall -SIGINT hardware_elf3");
        system("killall -SIGINT bxi_example_py_elf3");
        system("killall -SIGINT bxi_example_py_elf3_demo");
        system("killall -SIGINT bxi_bms");
        system("killall -SIGINT bxi_example_bms");
    }

    // ========== 重置所有值 ==========
    void reset_all_values()
    {
        memset(velxy_, 0, sizeof(velxy_));
        memset(velxy_filt_, 0, sizeof(velxy_filt_));
        velr_ = 0;
        velr_filt_ = 0;
        stand_height_ = STAND_HEIGHT;
        height_filt_ = STAND_HEIGHT;
        memset(js_axis_, 0, sizeof(js_axis_));
    }
    double get_dur_time(std::chrono::high_resolution_clock::time_point time)
    {
        return std::chrono::duration_cast<std::chrono::seconds>(
                   std::chrono::high_resolution_clock::now() - time)
            .count();
    }
    // ========== 发送电池信息 ==========
    void heartbeat_timer_callback()
    {
        if (crsf_parser_ && is_crsf_connected_ &&
            get_dur_time(lastest_bat_msg_time) < 5) {
            crsf_parser_->send_battery(lastest_bat_msg_.voltage * 10,
                                       lastest_bat_msg_.current * 10,
                                       lastest_bat_msg_.soc / 100 * 10, // 10Ah
                                       lastest_bat_msg_.soc);

            // crsf_parser_->send_battery(48.5 * 10, 3.2 * 10, (100 - 44) *
            // 10000,
            //                            44);
        }
    }
    void motor_timer_callback()
    {
        static int motor_indx = 0;
        if (crsf_parser_ && is_crsf_connected_ &&
            get_dur_time(lastest_motor_msg_time) < 60) {
            char textBuf[16];
            snprintf(
                textBuf, sizeof(textBuf), "%c%c", ((char)motor_indx) + 1,
                (char)round(lastest_motor_msg_.motor_temperature[motor_indx]));

            // std::random_device rd;
            // std::mt19937 gen(rd());
            // std::uniform_real_distribution<double> dist(28.0, 200.0);
            // memset(textBuf, 0, 16);
            // snprintf(textBuf, sizeof(textBuf), "%c%c", ((char)motor_indx) +
            // 1,
            //          (char)(dist(gen)));
            crsf_parser_->send_text(textBuf);
            motor_indx++;
            if (motor_indx >= lastest_motor_msg_.motor_temperature.size())
                motor_indx = 0;
        }
    }

    // ========== 速度指令订阅回调 ==========
    void velcmd_callback(geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        const std::lock_guard<std::mutex> guard(state_mutex_);
        if (is_crsf_connected_ && crsf_channels_[CRSF_NAVCTL_CHANNEL] > 0.5) {
            velxy_[0] = msg->twist.linear.x;
            velxy_[1] = msg->twist.linear.y;
            velr_ = msg->twist.angular.z;
        }
    }

    // ========== 电池状态订阅回调 ==========
    void bat_callback(communication::msg::BatteryStates::SharedPtr msg)
    {
        lastest_bat_msg_ = *msg;
        lastest_bat_msg_time = std::chrono::high_resolution_clock::now();
    }
    // =========== 电机温度订阅回调 ===========
    void motor_callback(communication::msg::ActuatorStates::SharedPtr msg)
    {
        lastest_motor_msg_ = *msg;
        lastest_motor_msg_time = std::chrono::high_resolution_clock::now();
    }
};

// ===================== 主函数 =====================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node =
        std::make_shared<RemoteControlNode>("/dev/ttyCRSF", "/dev/input/js0");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
