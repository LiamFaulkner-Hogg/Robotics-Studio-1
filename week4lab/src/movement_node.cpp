#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>
#include <iostream>
#include <geometry_msgs/msg/vector3.hpp>

class movement_node : public rclcpp::Node
{
public:
    movement_node() : Node("movement_node")
    {
        // Create a subscriber to the laser scan topic
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "scan", 10, std::bind(&movement_node::scanCallback, this, std::placeholders::_1));

        // Create a publisher for the filtered laser scan data
        speed_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&movement_node::timer_callback, this));
    }

private:
    {
        void setSpeedandDirection(float speed, char direction, float angle, char axis)
        {
            geometry_msgs::msg::Vector3 movement{};
            geometry_msgs::msg::Vector3 rotation{};

            // Set movement based on direction
            switch (direction)
            {
            case 'x':
                movement.x = speed;
                break;
            case 'y':
                movement.y = speed;
                break;
            case 'z':
                movement.z = speed;
                break;
            default:
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Direction incorrectly set");
                break;
            }

            // Set rotation based on axis
            switch (axis)
            {
            case 'x':
                rotation.x = angle;
                break;
            case 'y':
                rotation.y = angle;
                break;
            case 'z':
                rotation.z = angle;
                break;
            default:
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Axis incorrectly set");
                break;
            }

            geometry_msgs::msg::Twist move;
            move.linear = movement;
            move.angular = rotation;
        }

        void sendCmd(geometry_msgs::msg::Twist move, int distance)
        {
            int time = distance / move.linear;
            if (timer_ > time):
                break;
            else:
                speed_publisher_->publish(move);
        }

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr speed_publisher_;
    }
}