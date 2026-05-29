#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class TargetLocatorNode : public rclcpp::Node
{
public:
    TargetLocatorNode() : Node("target_locator")
    {
        // Initialize TF2 Broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Subscriptions
        image_sub_.subscribe(this, "/camera/image", rmw_qos_profile_sensor_data);
        pc_sub_.subscribe(this, "/camera/points", rmw_qos_profile_sensor_data);

        // Time Synchronizer
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), image_sub_, pc_sub_);
        sync_->registerCallback(std::bind(&TargetLocatorNode::process_vision_callback, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "Multimodal Preprocessing Pipeline Active. Scanning for AprilTag 36h11...");
    }

private:
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::PointCloud2> SyncPolicy;
    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> pc_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    void process_vision_callback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
                                 const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pc_msg)
    {
        RCLCPP_INFO_ONCE(this->get_logger(), ">>> SYNCHRONIZER CONNECTED: Receiving matched frames! <<<");
        // 1. 2D OPTICAL PREPROCESSING (OpenCV)
        cv_bridge::CvImagePtr cv_ptr;
        RCLCPP_INFO_ONCE(this->get_logger(), "Image encoding: %s", image_msg->encoding.c_str());
        try {
            cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        RCLCPP_INFO_ONCE(this->get_logger(), "Image successfully converted to BGR8");
        cv::Mat gray_image;
        cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);
        // Configure standard Aruco/AprilTag detector
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
        cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
        parameters->minDistanceToBorder = 0;  // Allow detection closer to edges
        parameters->minMarkerDistanceRate = 0.01;  // Lower threshold

        cv::aruco::detectMarkers(gray_image, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

        if (markerIds.empty() && !rejectedCandidates.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
            "Rejected candidates found: %zu. Tag may be at image edge or low contrast.", 
            rejectedCandidates.size());
            return;
        }
        else if (markerIds.empty()) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Frames received, but no AprilTag detected in 2D image.");
          return; // No target found in this frame
        }

        // 2. EXTRACT 2D PIXEL COORDINATES
        // Calculate the center pixel of the first detected tag
        cv::Point2f center_pixel(0.f, 0.f);
        for (const auto& corner : markerCorners[0]) {
            center_pixel += corner;
        }
        center_pixel *= 0.25f;
        
        int u = static_cast<int>(center_pixel.x);
        int v = static_cast<int>(center_pixel.y);

        // 3. 3D SPATIAL EXTRACTION (PCL)
        // Convert ROS PointCloud2 to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*pc_msg, cloud);

        // Ensure the cloud is organized (a 2D array matching the image pixels)
        if (!cloud.isOrganized()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Point cloud is unorganized. Cannot map 2D pixel to 3D point.");
            return;
        }

        // Extract the exact 3D spatial coordinate at the tag's center pixel
        pcl::PointXYZ target_3d = cloud.at(u, v);

        // Filter out bad depth data (NaN or Infinity)
        if (!std::isfinite(target_3d.x) || !std::isfinite(target_3d.y) || !std::isfinite(target_3d.z)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Target detected, but depth data is invalid (too close/far).");
            return;
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Target Locked! 3D Coord: X:%.2f, Y:%.2f, Z:%.2f", target_3d.x, target_3d.y, target_3d.z);

        // 4. BROADCAST TF2 COORDINATE FRAME
        // Send this data to the ROS 2 network so RViz and robotic arms can see it
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = pc_msg->header.stamp;
        t.header.frame_id = pc_msg->header.frame_id; // camera_link_optical
        t.child_frame_id = "cargo_target";

        t.transform.translation.x = target_3d.x;
        t.transform.translation.y = target_3d.y;
        t.transform.translation.z = target_3d.z;
        
        // Default orientation (facing the camera)
        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = 0.0;
        t.transform.rotation.w = 1.0;

        tf_broadcaster_->sendTransform(t);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TargetLocatorNode>());
    rclcpp::shutdown();
    return 0;
}