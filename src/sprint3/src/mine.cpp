// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/laser_scan.hpp>
// #include <geometry_msgs/msg/twist.hpp>
// #include <nav_msgs/msg/odometry.hpp>
// #include <nav_msgs/msg/occupancy_grid.hpp>
// #include <opencv2/opencv.hpp>
// #include <vector>

// class cylinder_Detector : public rclcpp::Node
// {
// public:
//     cylinder_Detector() : Node("simple_localizer"), cylinderFound(false), imgShown(false)
//     {
//         scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
//             "/scan", 10, std::bind(&cylinder_Detector::scanCallback, this, std::placeholders::_1));
//     }

// private:
//     void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
//     {
//         if (!cylinderFound)
//         {
//             laser_map_ = laserScanToMat(msg);
//             cv::imshow("Laser Map", laser_map_);
//             cv::waitKey(10);

//             if (!imgShown)
//             {
//                 semiCircle_img = semiCircle();
//             }

//             detectAndMatchFeatures(laser_map_, semiCircle_img);
//         }
//     }

//     cv::Mat laserScanToMat(const sensor_msgs::msg::LaserScan::SharedPtr &scan)
//     {
//         int img_size = 500;
//         float max_range = scan->range_max;
//         cv::Mat img = cv::Mat::zeros(img_size, img_size, CV_8UC1);
//         for (size_t i{0}; i < scan->ranges.size(); i++)
//         {
//             float range = scan->ranges.at(i);
//             if (std::isfinite(range) && range >= scan->range_min && range <= scan->range_max)
//             {
//                 float angle = scan->angle_min + i * scan->angle_increment;
//                 int x = static_cast<int>((range * cos(angle)) * img_size / (2 * max_range)) + img_size / 2;
//                 int y = static_cast<int>((range * sin(angle)) * img_size / (2 * max_range)) + img_size / 2;
//                 if (x >= 0 && x <= img_size && y >= 0 && y <= img_size)
//                 {
//                     img.at<uchar>(y, x) = 255; // Set the pixel to white
//                 }
//             }
//         }
//         return img;
//     }

//     cv::Mat semiCircle(void)
//     {
//         int img_size = 500;
//         cv::Mat semiCircle_img = cv::Mat::zeros(img_size, img_size, CV_8UC1);
//         cv::Point center(250, 250);
//         int radius = 100;

//         // draw upright black halfcircle
//         double startAngleUpright = 160;
//         cv::ellipse(semiCircle_img, center, cv::Size(radius, radius), 0, startAngleUpright, startAngleUpright + 140, 255, 0);

//         cv::imshow("semiCircle", semiCircle_img);
//         imgShown = true;

//         return semiCircle_img;
//     }

//     void detectAndMatchFeatures(const cv::Mat &img1, const cv::Mat &img2)
//     {
//         cv::Ptr<cv::ORB> orb = cv::ORB::create();
//         std::vector<cv::KeyPoint> keypoints1, keypoints2;
//         cv::Mat descriptors1, descriptors2;

//         orb->detectAndCompute(img1, cv::noArray(), keypoints1, descriptors1);
//         orb->detectAndCompute(img2, cv::noArray(), keypoints2, descriptors2);

//         cv::BFMatcher matcher(cv::NORM_HAMMING);
//         std::vector<cv::DMatch> matches;
//         matcher.match(descriptors1, descriptors2, matches);

//         // Define a threshold for a "good" match.
//         const float maxDistanceThreshold = 4.0f; // Adjust this value to control match similarity

//         // Filter matches based on the distance threshold
//         std::vector<cv::DMatch> goodMatches;
//         for (const auto &match : matches)
//         {
//             if (match.distance < maxDistanceThreshold)
//             {
//                 goodMatches.push_back(match);
//             }
//         }

//         std::cout << "Matches:" << goodMatches.size() << std::endl;

//         // Check if there are enough good matches to verify if the semicircle is present
//         if (goodMatches.size() > 10) // Adjust this threshold based on experimentation
//         {
//             // Get points from good matches
//             std::vector<cv::Point2f> pointsimg, pointssemiCircle;
//             for (const auto &match : goodMatches)
//             {
//                 pointsimg.push_back(keypoints1[match.queryIdx].pt);
//                 pointssemiCircle.push_back(keypoints2[match.trainIdx].pt);
//             }

//             // Find homography to check if there is a valid transformation between the template and the updating image
//             cv::Mat homography = cv::findHomography(pointsimg, pointssemiCircle, cv::RANSAC);

//             // If homography is found, the semicircle is likely present in the updating image
//             if (!homography.empty())
//             {
//                 std::cout << "Semicircle detected in the updating image." << std::endl;
//             }
//             else
//             {
//                 std::cout << "No valid transformation found. Semicircle not detected." << std::endl;
//             }
//         }
//         else
//         {
//             std::cout << "Not enough good matches found to detect semicircle." << std::endl;
//         }

//         // RCLCPP_INFO(logger, "Number of good matches: %zu", goodMatches.size());
//         goodMatches.clear();
//         matches.clear();
//         // pointsimg.clear();
//         // pointssemiCircle.clear();
//     }

//     bool cylinderFound;
//     bool imgShown;

//     rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;

//     cv::Mat laser_map_, semiCircle_img;
// };

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<cylinder_Detector>());
//     rclcpp::shutdown();
//     return 0;
// }