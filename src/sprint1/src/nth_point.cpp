#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>
#include <iostream>

class nth_point : public rclcpp::Node
{
public:
    nth_point() : Node("Lidar_Points")
    {
        // Create a subscriber to the laser scan topic
        lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&nth_point::scanCallback, this, std::placeholders::_1));

        // Create a publisher for the filtered laser scan data
        data_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("filtered_scan", 10);
    }

private:
    void scanCallback(const std::shared_ptr<sensor_msgs::msg::LaserScan> scan)
    {
        // Desired point increment for range reading nth_points
        float desired_points = 5;
        float angle_step = scan->angle_increment*desired_points;


        // Create a new LaserScan message for the filtered data
        auto new_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*scan);
        std::vector<float> points;
        for (int i = 1; i*desired_points < size(scan->ranges); i++)
        {
            int index = i*desired_points;
            points.push_back(scan->ranges.at(index));
            max_index = index;
        }
        new_scan->ranges = points;
        new_scan->angle_increment = angle_step;
        new_scan->angle_min = scan->angle_increment * desired_points;
        new_scan->angle_max = scan->angle_increment * max_index;

        // Publish the filtered laser scan data
        data_publisher_->publish(*new_scan);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr data_publisher_;
    int max_index;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<nth_point>());
    rclcpp::shutdown();
    return 0;
}
