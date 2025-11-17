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

class MotorControlSample : public rclcpp::Node
{
public:
    MotorControlSample() : 
    rclcpp::Node("motor_control_set_node"),
    motor(RobStrideMotor("can0", 0xFF, 127, 0))//ID:127
    {
        auto [position_, velocity_, torque_, temperature_] =  motor.enable_motor();

        usleep(1000);
        worker_thread_ = std::thread(&MotorControlSample::excute_loop, this);
        
        RCLCPP_INFO(this->get_logger(), "MotorControlSample node started");
    }

    ~MotorControlSample()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down MotorControlSample node");
        motor.Disenable_Motor(0);
        running_ = false;
        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }
    }

    void excute_loop()
    {
        float position = 1.57f;
        float velocity = 0.1f;
        int counter = 0;
        
        while (rclcpp::ok() && running_)
        {
            auto [position_feedback, velocity_feedback, torque, temperature] =
                // motor.send_motion_command(0.0, position, velocity, 0.1f, 0.1f);
            // motor.RobStrite_Motor_PosCSP_control(float Speed, float Acceleration, float Angle);
            // motor.RobStrite_Motor_Current_control(float IqCommand, float IdCommand);
            motor.send_velocity_mode_command(0.1f);
            // motor.RobStrite_Motor_PosCSP_control(position, velocity);

            
            // 使用 ROS 2 日志（推荐）
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,  // 每1000ms输出一次
                "Position: %.3f, Velocity: %.3f, Counter: %d", 
                position_feedback, velocity_feedback, counter++
            );
            

            
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

private:
    std::thread worker_thread_;
    std::atomic<bool> running_ = true;
    RobStrideMotor motor;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto controller = std::make_shared<MotorControlSample>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(controller);
    
    RCLCPP_INFO(controller->get_logger(), "Starting executor");
    executor.spin();
    
    RCLCPP_INFO(controller->get_logger(), "Shutting down");
    rclcpp::shutdown();

    return 0;
}