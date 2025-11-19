#include "motor_ros2/motor_cfg.h"
#include <rclcpp/node.hpp>
#include <thread>
#include <unistd.h>
#include <vector> 
#include <memory> 
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include "stdint.h"
#include <atomic>
#include <iostream>

int servo_num = 2;

class MultiMotorControl : public rclcpp::Node
{
public:
    MultiMotorControl() : rclcpp::Node("multi_motor_control_node")
    {

        current_positions.resize(servo_num, 0.0f);
        directions.resize(servo_num, true);

        // 创建8个电机实例，使用不同的CAN ID
        for (int i = 0; i < servo_num; i++) {
            // 假设电机ID从1到8
            uint8_t motor_id = 126 + i;
            auto motor = std::make_shared<RobStrideMotor>("can0", 0xFF, motor_id, 0);
            motors_.push_back(motor);
            
            // 启用每个电机
            try {
                auto [position_feedback, velocity, torque, temperature] = motor->enable_motor();
                current_positions[i] = position_feedback;
                RCLCPP_INFO(this->get_logger(), "Motor %d enabled, initial position: %.3f", motor_id, position_feedback);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to enable motor %d: %s", motor_id, e.what());
            }
        }
        
        usleep(1000);
        worker_thread_ = std::thread(&MultiMotorControl::execute_loop, this);
        RCLCPP_INFO(this->get_logger(), "MultiMotorControl node started with %d motors", (int)motors_.size());


    }

    ~MultiMotorControl()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down MultiMotorControl node");
        running_ = false;
        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }
        
        // 禁用所有电机
        for (auto& motor : motors_) {
            try {
                motor->Disenable_Motor(0);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error disabling motor: %s", e.what());
            }
        }
    }

    void execute_loop()
    {

        
        const float MIN_POSITION = -3.1415926f*2; // -360 弧度
        const float MAX_POSITION = 3.1415926f*2;  // 360 弧度
        const float POSITION_INCREMENT = 0.001f; // 每个周期的位置增量
        
        int counter = 0;
        
        
        
        while (rclcpp::ok() && running_)
        {
            for (size_t i = 0; i < motors_.size(); i++) {
                try {
                    // 根据方向更新位置
                    if (directions[i]) {
                        // 正向运动
                        current_positions[i] += POSITION_INCREMENT;
                        if (current_positions[i] >= MAX_POSITION) {
                            current_positions[i] = MAX_POSITION; // 确保不超限
                            directions[i] = false; // 改变方向
                            RCLCPP_WARN(this->get_logger(), "Motor %zu reached MAX position (360°), reversing direction", i);
                        }
                    } else {
                        // 反向运动
                        current_positions[i] -= POSITION_INCREMENT;
                        if (current_positions[i] <= MIN_POSITION) {
                            current_positions[i] = MIN_POSITION; // 确保不超限
                            directions[i] = true; // 改变方向
                            RCLCPP_WARN(this->get_logger(), "Motor %zu reached MIN position (-360°), reversing direction", i);
                        }
                    }
                    
                    // 发送CSP位置控制命令
                    auto [position_feedback, velocity_feedback, torque, temperature] = 
                        motors_[i]->RobStrite_Motor_PosCSP_control( current_positions[i],0.1f);
                    
                    // 定期输出状态信息
                    if (counter % 50 == 0) {
                        RCLCPP_INFO_THROTTLE(
                            this->get_logger(),
                            *this->get_clock(),
                            500,
                            "Motor %zu - Target: %7.2f°, Actual: %7.2f°, Vel: %6.3f, Dir: %s", 
                            i, 
                            current_positions[i] * (180.0f / M_PI), // 弧度转角度显示
                            position_feedback * (180.0f / M_PI),    // 弧度转角度显示
                            velocity_feedback,
                            directions[i] ? "→" : "←"
                        );
                    }
                    
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Motor %zu control error: %s", i, e.what());
                }
            }
            
            counter++;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

private:
    std::thread worker_thread_;
    std::atomic<bool> running_ = true;
    std::vector<std::shared_ptr<RobStrideMotor>> motors_;
    std::vector<float> current_positions;  // 正确写法
    std::vector<bool> directions;          // 正确写法
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto controller = std::make_shared<MultiMotorControl>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(controller);
    
    RCLCPP_INFO(controller->get_logger(), "Starting multi-motor executor");
    executor.spin();
    
    RCLCPP_INFO(controller->get_logger(), "Shutting down");
    rclcpp::shutdown();

    return 0;
}