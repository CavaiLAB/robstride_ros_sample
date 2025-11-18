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

class MultiMotorControl : public rclcpp::Node
{
public:
    MultiMotorControl() : rclcpp::Node("multi_motor_control_node")
    {
        // 创建8个电机实例，使用不同的CAN ID
        for (int i = 120; i < 120+8; i++) {
            // 假设电机ID从1到8
            uint8_t motor_id = i;
            auto motor = std::make_shared<RobStrideMotor>("can0", 0xFF, motor_id, 0);
            motors_.push_back(motor);
            
            // 启用每个电机
            auto [position, velocity, torque, temperature] = motor->enable_motor();
            RCLCPP_INFO(this->get_logger(), "Motor %d enabled", motor_id);
        }
        
        usleep(1000);
        worker_thread_ = std::thread(&MultiMotorControl::execute_loop, this);
        RCLCPP_INFO(this->get_logger(), "MultiMotorControl node started with %d motors", motors_.size());
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
            motor->Disenable_Motor(0);
        }
    }

    void execute_loop()
    {
        std::vector<float> target_positions = {1.57f, -1.57f, 0.78f, -0.78f, 2.35f, -2.35f, 3.14f, -3.14f};
        std::vector<float> target_velocities = {0.1f, 0.15f, 0.2f, 0.25f, 0.3f, 0.35f, 0.4f, 0.45f};
        int counter = 0;
        
        while (rclcpp::ok() && running_)
        {
            // 控制所有电机
            for (size_t i = 0; i < motors_.size(); i++) {
                try {
                    auto [position_feedback, velocity_feedback, torque, temperature] = 
                        motors_[i]->send_velocity_mode_command(target_velocities[i]);
                    
                    // 发布每个电机的状态（可选）
                    if (counter % 100 == 0) {
                        RCLCPP_INFO_THROTTLE(
                            this->get_logger(), *this->get_clock(), 1000,
                            "Motor %zu - Pos: %.3f, Vel: %.3f, Torque: %.3f", 
                            i, position_feedback, velocity_feedback, torque
                        );
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Motor %zu error: %s", i, e.what());
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