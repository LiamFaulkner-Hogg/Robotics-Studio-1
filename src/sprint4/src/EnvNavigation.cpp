#include <chrono>
#include <iostream>
#include <memory>
#include <vector>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/duration.hpp"

using namespace std::chrono_literals;

class EnvNavigation : public rclcpp::Node
{
public:
    EnvNavigation() : Node("EnvNavigation")
    {
        // Initialize the ROS 2 Node
        this->declare_parameter("navigation_goal_timeout", 1000); // Timeout for goals in seconds
        this->declare_parameter("preempt_time", 500);             // Preemption time in seconds

        // Create the action client for NavigateToPose
        action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(this, "navigate_through_poses");

        path_client_ = rclcpp_action::create_client<nav2_msgs::action::ComputePathThroughPoses>(this, "compute_path_through_poses");

        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/injured_path", 10);

        initial_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10, std::bind(&EnvNavigation::onInitialPoseReceived, this, std::placeholders::_1));

        injured_persons_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/civilian", 10, std::bind(&EnvNavigation::personLocation, this, std::placeholders::_1));

        if (!path_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        }

        // Wait until the action server is available
        while (!action_client_->wait_for_action_server(1s))
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
        }
    }

private:
    rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr action_client_;
    rclcpp_action::Client<nav2_msgs::action::ComputePathThroughPoses>::SharedPtr path_client_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr injured_persons_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_subscription_;

    void onInitialPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Initial pose received, starting navigation...");

        rclcpp::sleep_for(std::chrono::seconds(5));

        std::vector<geometry_msgs::msg::PoseStamped> goal_poses = SetGoalPoses();
        SendGoal(goal_poses);
    }

    void personLocation(const geometry_msgs::msg::PoseStamped &injured)
    {
        auto send_goal_option = rclcpp_action::Client<nav2_msgs::action::ComputePathThroughPoses>::SendGoalOptions();
        send_goal_option.result_callback = std::bind(&EnvNavigation::HandleInjuredResult, this, std::placeholders::_1);

        // Create the goal message for ComputePathThroughPoses action
        geometry_msgs::msg::PoseStamped injured_person = injured;
        auto person_msg = nav2_msgs::action::ComputePathThroughPoses::Goal();
        geometry_msgs::msg::PoseStamped start_pose;

        person_msg.goals.clear();

        start_pose.header.frame_id = "map"; // Reference the global frame
        start_pose.header.stamp = this->get_clock()->now();

        // Define the fixed starting position (example coordinates, adjust as needed)
        start_pose.pose.position.x = 1.6;
        start_pose.pose.position.y = 9;
        start_pose.pose.orientation.z = 0.5;
        start_pose.pose.orientation.w = 1.0;

        // Set the starting pose in the goal message
        person_msg.goals.push_back(start_pose);

        // Since person_msg.poses expects a vector, we need to create one and add the single pose
        injured_person.header.frame_id = "map";
        injured_person.pose.position.x = injured_person.pose.position.x;
        injured_person.pose.position.y = injured_person.pose.position.y;
        injured_person.pose.orientation.z = 1.0;
        injured_person.pose.orientation.w = 1.0;

        person_msg.goals.push_back(injured_person);

        RCLCPP_INFO(this->get_logger(), "Sending location of injured person");

        // Send the location using the action client
        auto person = path_client_->async_send_goal(person_msg, send_goal_option);

        RCLCPP_INFO(this->get_logger(), "Goal sent to compute_path_through_poses");

        // nav_msgs::msg::Path path;
        // path.header.stamp = this->get_clock()->now();
        // path.header.frame_id = "map";
        // path.poses = result.result->path.poses;

        // path_publisher_->publish(path);
    }

    void HandleInjuredResult(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathThroughPoses>::WrappedResult &result)

    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(this->get_logger(), "Path to injured person successfully computed.");
            size_t path_size = result.result->path.poses.size();
            RCLCPP_INFO(this->get_logger(), "Computed path size: %lu", result.result->path.poses.size());

            const double target_x = 1.6;
            const double target_y = 9.0;

            // Vector to hold the poses from the found point to the end of the path
            std::vector<geometry_msgs::msg::PoseStamped> selected_poses;

            // Flag to indicate whether the target has been found
            bool found_target = false;

            // Loop through the poses in the computed path
            for (const auto &pose : result.result->path.poses)
            {
                // Check if the current pose matches the target coordinates
                if (!found_target && (pose.pose.position.x == target_x) && (pose.pose.position.y == target_y))
                {
                    found_target = true; // Set the flag when the target is found
                }

                // If the target has been found, add the current pose to the selected poses
                if (found_target)
                {
                    selected_poses.push_back(pose);
                }
            }

            PublishSelectedPath(selected_poses); // Publish the path if successful
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Failed to compute path to injured person.");
            if (result.code == rclcpp_action::ResultCode::ABORTED)
            {
                RCLCPP_ERROR(this->get_logger(), "Action aborted.");
            }
            else if (result.code == rclcpp_action::ResultCode::CANCELED)
            {
                RCLCPP_ERROR(this->get_logger(), "Action canceled.");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Unknown result code: %d", result.code);
            }
        }
    }

    void PublishSelectedPath(const std::vector<geometry_msgs::msg::PoseStamped> &selected_poses)
    {
        nav_msgs::msg::Path selected_path;
        selected_path.header.frame_id = "map";
        selected_path.header.stamp = this->get_clock()->now();

        // Add selected poses to the new path
        selected_path.poses = selected_poses;

        // Publish the selected path
        path_publisher_->publish(selected_path);

        RCLCPP_INFO(this->get_logger(), "Published selected path from (1.6, 9) to the end of the computed path.");
    }

    void SendGoal(const std::vector<geometry_msgs::msg::PoseStamped> &goal_pose)
    {
        auto goal_msg = nav2_msgs::action::NavigateThroughPoses::Goal();
        goal_msg.poses = goal_pose;

        // Send the goal to the action server
        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions();
        send_goal_options.result_callback = std::bind(&EnvNavigation::HandleResult, this, std::placeholders::_1);

        RCLCPP_INFO(this->get_logger(), "Sending goal with %lu waypoints", goal_msg.poses.size());

        // Send the goal
        auto goal_handle_future = action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void HandleResult(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Navigation Succeeded");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Navigation was Canceled");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Navigation Aborted");
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            break;
        }
    }

    std::vector<geometry_msgs::msg::PoseStamped> SetGoalPoses()
    {
        std::vector<geometry_msgs::msg::PoseStamped> goal_poses;

        // Define multiple goal poses as before
        geometry_msgs::msg::PoseStamped goal_pose;

        goal_pose.header.frame_id = "map";
        goal_pose.header.stamp = this->get_clock()->now();

        goal_pose.pose.position.x = 1.63;
        goal_pose.pose.position.y = -6.01;
        goal_pose.pose.orientation.z = 1;
        goal_pose.pose.orientation.w = 1.0;
        goal_poses.push_back(goal_pose);

        goal_pose.pose.position.x = 3.65;
        goal_pose.pose.position.y = -6.09;
        goal_pose.pose.orientation.z = 0.72;
        goal_pose.pose.orientation.w = 1.0;
        goal_poses.push_back(goal_pose);

        goal_pose.pose.position.x = 3.83;
        goal_pose.pose.position.y = 1.99;
        goal_pose.pose.orientation.z = 0.72;
        goal_pose.pose.orientation.w = 1.0;
        goal_poses.push_back(goal_pose);

        goal_pose.pose.position.x = 5.42;
        goal_pose.pose.position.y = 2.03;
        goal_pose.pose.orientation.z = 1;
        goal_pose.pose.orientation.w = 1;
        goal_poses.push_back(goal_pose);

        goal_pose.pose.position.x = 5.49;
        goal_pose.pose.position.y = -9.0;
        goal_pose.pose.orientation.z = 0.67;
        goal_pose.pose.orientation.w = 1;
        goal_poses.push_back(goal_pose);

        goal_pose.pose.position.x = -3.77;
        goal_pose.pose.position.y = -7.78;
        goal_pose.pose.orientation.z = -0.71;
        goal_pose.pose.orientation.w = 1;
        goal_poses.push_back(goal_pose);

        goal_pose.pose.position.x = -3.79;
        goal_pose.pose.position.y = 8.94;
        goal_pose.pose.orientation.z = -0.69;
        goal_pose.pose.orientation.w = 1;
        goal_poses.push_back(goal_pose);

        // goal_pose.pose.position.x = 1.6;
        // goal_pose.pose.position.y = 9.5;
        // goal_pose.pose.orientation.z = -0.01;
        // goal_pose.pose.orientation.w = 1.0;
        // goal_poses.push_back(goal_pose);

        // goal_pose.pose.position.x = 4.67;
        // goal_pose.pose.position.y = 9.7;
        // goal_pose.pose.orientation.z = -0.01;
        // goal_pose.pose.orientation.w = 1.0;
        // goal_poses.push_back(goal_pose);

        goal_pose.pose.position.x = -6.1;
        goal_pose.pose.position.y = 9.19;
        goal_pose.pose.orientation.z = 0;
        goal_pose.pose.orientation.w = 1;
        goal_poses.push_back(goal_pose);

        return goal_poses;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EnvNavigation>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

// NOTES

// Improve the "handleresults" function by checking the "result.result" rather than "result.code" this is because result.result returns more
// detailed information about the state of the robot such as pose accuracy (just one example)

// Also change the setup a little bit, line 32 could be made a world variable and directly assign the goal_poses in the function without needing
// to make it equal the result of the function. Hopefully that makes sense.
