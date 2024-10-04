#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

class ScanToImageNode : public rclcpp::Node
{
public:
    ScanToImageNode() : Node("scan_to_image_node"), angle_difference_(0.0), relative_orientation_(0.0)
    {
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ScanToImageNode::scanCallback, this, std::placeholders::_1));
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        RCLCPP_INFO(this->get_logger(), "Scan to Image Node started.");
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Purpose: This function is a callback that gets triggered whenever a new LaserScan message is received from the /scan topic.

        // Functionality:

        //     Convert LaserScan to Image: The laserScanToMat function is called to convert the LaserScan data into an image (a cv::Mat).
        //     Handle First Image: If it's the first time this function is being called, it captures and displays the first image.
        //     Handle Second Image: If it's the second time this function is called, it captures and displays the second image, then calculates the change in orientation (yaw).
        //     Update and Rotate: For subsequent calls, it updates the images, calculates the yaw change, and logs the relative orientation.

        cv::Mat image = laserScanToMat(msg);

        if (!first_image_captured_)
        {
            first_image_ = image;
            first_image_captured_ = true;
            cv::imshow("Image 1:", first_image_);
            cv::waitKey(1);
            RCLCPP_INFO(this->get_logger(), "First image captured.");
        }
        else if (!second_image_captured_)
        {
            second_image_ = image;
            second_image_captured_ = true;
            cv::imshow("Image 2:", second_image_);
            cv::waitKey(1);
            RCLCPP_INFO(this->get_logger(), "Second image captured.");
            calculateYawChange();
        }
        else
        {
            // Optionally update images and calculate yaw again if required
            first_image_ = second_image_;
            second_image_ = image;
            cv::destroyAllWindows();
            cv::imshow("Image 1:", first_image_);
            cv::imshow("Image 2:", second_image_);
            cv::waitKey(1);
            calculateYawChange();
        }

        RCLCPP_INFO(this->get_logger(), "Press enter to run again or pres 'q' to quit");
        std::string input;
        std::getline(std::cin, input);

        if (input == "q" || input == "Q")
        {
            rclcpp::shutdown();
        }
    }

    cv::Mat laserScanToMat(const sensor_msgs::msg::LaserScan::SharedPtr &scan)
    {

        // Purpose: Converts a LaserScan message into a binary image (cv::Mat), where each pixel represents the presence of an obstacle detected by the laser scanner.

        // Functionality:

        //      Create Image: Initializes a blank image of size 500x500 pixels.
        //      Map Polar to Cartesian: Iterates over the laser scan data, converting polar coordinates (distance and angle) to Cartesian coordinates (x, y) and sets the corresponding pixel in the image to white (255) if within range.
        cv::Mat image = cv::Mat::zeros(cv::Size(500, 500), CV_8UC1);

        for (size_t i = 0; i < scan->ranges.size(); i++)
        {
            if (std::isinf(scan->ranges[i]) || std::isnan(scan->ranges[i]))
            {
                continue;
            }

            float hypotenuse = scan->ranges[i];
            float angle = (scan->angle_increment * i) + scan->angle_min;

            int x_cord = static_cast<int>(250 + hypotenuse * cos(angle) * 100);
            int y_cord = static_cast<int>(250 + hypotenuse * sin(angle) * 100);

            if (x_cord >= 0 && x_cord < 500 && y_cord >= 0 && y_cord < 500)
            {
                image.at<uchar>(y_cord, x_cord) = 255;
            }
        }
        return image;
    }

    void calculateYawChange()
    {
        // Purpose: Estimates the change in orientation (yaw angle) of the robot by comparing two images.

        // Functionality:

        //     Feature Matching: Uses feature detection and matching to find corresponding points between the two images.
        // std::vector<cv::Point2f> srcPoints, dstPoints;
        // detectAndMatchFeatures(first_image_, second_image_, srcPoints, dstPoints);
        //     Estimate Transformation: Computes an affine transformation matrix to determine the rotation between the two images.
        //     Calculate Angle: Extracts the rotation angle from the transformation matrix and converts it to degrees.

        std::vector<cv::Point2f> srcPoints, dstPoints;
        detectAndMatchFeatures(first_image_, second_image_, srcPoints, dstPoints);

        if (srcPoints.size() >= 3 && dstPoints.size() >= 3)
        {
            cv::Mat transform = cv::estimateAffinePartial2D(srcPoints, dstPoints);

            if (!transform.empty())
            {
                double rotation_radians = atan2(transform.at<double>(1, 0), transform.at<double>(0, 0));
                angle_difference_ = rotation_radians * (180.0 / M_PI); // Convert to degrees

                relative_orientation_ = relative_orientation_ + angle_difference_;
                if (relative_orientation_ > 360)
                {
                    relative_orientation_ = relative_orientation_ - 360;
                }

                RCLCPP_INFO(this->get_logger(), "Yaw change: %f degrees", angle_difference_);
                RCLCPP_INFO(this->get_logger(), "Relative orientation: %f degrees", relative_orientation_);

                geometry_msgs::msg::Twist cmd_msg;
                // cmd_msg.angular.z = angle_difference_;
                if (angle_difference_ < 0)
                {
                    cmd_msg.angular.z = 0.1;
                }
                else
                {
                    cmd_msg.angular.z = -0.1;
                }

                float time = ((abs(rotation_radians)) * 10) * 1000;
                RCLCPP_INFO(this->get_logger(), "Time for Rotation: %f milliseconds", time);
                auto start_time = std::chrono::steady_clock::now();
                double elapsed_time = 0;
                while (time > elapsed_time)
                {
                    cmd_publisher_->publish(cmd_msg);

                    RCLCPP_INFO(this->get_logger(), "Running turning command");

                    elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                                       std::chrono::steady_clock::now() - start_time)
                                       .count();
                }
                cmd_msg.angular.z = 0.0;
                cmd_publisher_->publish(cmd_msg);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Transformation estimation failed.");
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Not enough feature points detected.");
        }
    }

    void detectAndMatchFeatures(const cv::Mat &img1, const cv::Mat &img2,
                                std::vector<cv::Point2f> &srcPoints, std::vector<cv::Point2f> &dstPoints)
    {
        cv::Ptr<cv::ORB> orb = cv::ORB::create();
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        cv::Mat descriptors1, descriptors2;

        orb->detectAndCompute(img1, cv::noArray(), keypoints1, descriptors1);
        orb->detectAndCompute(img2, cv::noArray(), keypoints2, descriptors2);

        cv::BFMatcher matcher(cv::NORM_HAMMING);
        std::vector<cv::DMatch> matches;
        matcher.match(descriptors1, descriptors2, matches);

        // Sort matches based on distance (lower distance means better match)
        std::sort(matches.begin(), matches.end(), [](const cv::DMatch &a, const cv::DMatch &b)
                  { return a.distance < b.distance; });

        // Determine the number of top matches to keep (30% of total matches)
        size_t numGoodMatches = static_cast<size_t>(matches.size() * 0.15);

        // Keep only the best matches (top 30%)
        std::vector<cv::DMatch> goodMatches(matches.begin(), matches.begin() + numGoodMatches);

        for (const auto &match : matches)
        {
            srcPoints.push_back(keypoints1[match.queryIdx].pt);
            dstPoints.push_back(keypoints2[match.trainIdx].pt);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;

    cv::Mat first_image_, second_image_;
    bool first_image_captured_ = false;
    bool second_image_captured_ = false;

    double angle_difference_;
    double relative_orientation_ = 0.0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ScanToImageNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
