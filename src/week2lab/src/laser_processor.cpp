#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LaserProcessor : public rclcpp::Node
{
public:
    LaserProcessor() : Node("laser_processor")
    {
        // Subscribe to the /scan topic
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LaserProcessor::scanCallback, this, std::placeholders::_1));

        // Create a publisher for the filtered scan data
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_filtered", 10);
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        // Extract range reading for a specific angle (e.g., 180 degrees)
        int desired_angle = (315 * (M_PI/180))/ scan_msg -> angle_increment;
        //int angle_index = desired_angle - (scan_msg -> angle_min);
        float specific_range = scan_msg->ranges[desired_angle];
        //float specific_range = scan_msg->ranges[angle_index];
        RCLCPP_INFO(this->get_logger(), "angle index: %f", desired_angle); 
        RCLCPP_INFO(this->get_logger(), "Range at specific range: %f", specific_range);

        // Create a new LaserScan message to hold the filtered data
        auto filtered_scan = *scan_msg;

        // Select a subset of range values (e.g., from 30 to 150 degrees)
        int start_index = scan_msg->ranges.size() * 30 / 360;
        int end_index = scan_msg->ranges.size() * 150 / 360;

        filtered_scan.ranges.assign(scan_msg->ranges.begin() + start_index, scan_msg->ranges.begin() + end_index);

        // Publish the filtered scan data
        scan_pub_->publish(filtered_scan);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserProcessor>());
    rclcpp::shutdown();
    return 0;
}
