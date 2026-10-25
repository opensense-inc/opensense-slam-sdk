#include <memory>
#include <string>
#include <mutex>
#include <vector>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <opencv2/core/core.hpp>

#include "System.h"
#include "ImuTypes.h"

class OrbSlam3Node : public rclcpp::Node
{
public:
    OrbSlam3Node() : Node("orbslam3_node")
    {
        this->declare_parameter<std::string>("vocab_file", "/opt/ORB_SLAM3/Vocabulary/ORBvoc.txt");
        this->declare_parameter<std::string>("settings_file", "");
        this->declare_parameter<std::string>("mode", "RGB-D");
        this->declare_parameter<bool>("enable_pangolin_viewer", false);
        this->declare_parameter<std::string>("mono_topic", "/camera/image_raw");
        this->declare_parameter<std::string>("rgb_topic", "/camera/color/image_raw");
        this->declare_parameter<std::string>("depth_topic", "/camera/aligned_depth_to_color/image_raw");
        this->declare_parameter<std::string>("stereo_left_topic", "/camera/left/image_raw");
        this->declare_parameter<std::string>("stereo_right_topic", "/camera/right/image_raw");
        this->declare_parameter<std::string>("imu_topic", "/imu/data");
        this->declare_parameter<std::string>("world_frame_id", "map");
        this->declare_parameter<std::string>("camera_frame_id", "camera_link");
        this->declare_parameter<int>("queue_size", 50);

        vocab_file_ = this->get_parameter("vocab_file").as_string();
        settings_file_ = this->get_parameter("settings_file").as_string();
        sensor_mode_str_ = this->get_parameter("mode").as_string();
        enable_viewer_ = this->get_parameter("enable_pangolin_viewer").as_bool();
        mono_topic_ = this->get_parameter("mono_topic").as_string();
        rgb_topic_ = this->get_parameter("rgb_topic").as_string();
        depth_topic_ = this->get_parameter("depth_topic").as_string();
        stereo_left_topic_ = this->get_parameter("stereo_left_topic").as_string();
        stereo_right_topic_ = this->get_parameter("stereo_right_topic").as_string();
        imu_topic_ = this->get_parameter("imu_topic").as_string();
        world_frame_id_ = this->get_parameter("world_frame_id").as_string();
        camera_frame_id_ = this->get_parameter("camera_frame_id").as_string();
        queue_size_ = this->get_parameter("queue_size").as_int();

        if (settings_file_.empty()) {
            RCLCPP_FATAL(this->get_logger(), "Parameter 'settings_file' must be specified.");
            rclcpp::shutdown();
            return;
        }

        // Dynamic sensor profile mapping
        if (sensor_mode_str_ == "Monocular") {
            sensor_type_ = ORB_SLAM3::System::MONOCULAR;
        } else if (sensor_mode_str_ == "Stereo") {
            sensor_type_ = ORB_SLAM3::System::STEREO;
        } else if (sensor_mode_str_ == "RGB-D" || sensor_mode_str_ == "RGBD") {
            sensor_type_ = ORB_SLAM3::System::RGBD;
        } else if (sensor_mode_str_ == "Monocular-Inertial") {
            sensor_type_ = ORB_SLAM3::System::IMU_MONOCULAR;
        } else if (sensor_mode_str_ == "Stereo-Inertial") {
            sensor_type_ = ORB_SLAM3::System::IMU_STEREO;
        } else if (sensor_mode_str_ == "RGB-D-Inertial" || sensor_mode_str_ == "RGBD-Inertial") {
            sensor_type_ = ORB_SLAM3::System::IMU_RGBD;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Unknown sensor mode '%s', falling back to RGB-D.", sensor_mode_str_.c_str());
            sensor_type_ = ORB_SLAM3::System::RGBD;
        }

        RCLCPP_INFO(this->get_logger(), "Initializing ORB-SLAM3 System...");
        RCLCPP_INFO(this->get_logger(), "Mode: %s", sensor_mode_str_.c_str());
        RCLCPP_INFO(this->get_logger(), "Vocab file: %s", vocab_file_.c_str());
        RCLCPP_INFO(this->get_logger(), "Settings file: %s", settings_file_.c_str());

        slam_system_ = std::make_unique<ORB_SLAM3::System>(
            vocab_file_,
            settings_file_,
            sensor_type_,
            enable_viewer_
        );

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/orbslam3/camera_pose", 10);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/orbslam3/trajectory", 10);

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto custom_qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile));

        // Setup IMU subscriber if mode requires inertial data
        if (sensor_type_ == ORB_SLAM3::System::IMU_MONOCULAR ||
            sensor_type_ == ORB_SLAM3::System::IMU_STEREO ||
            sensor_type_ == ORB_SLAM3::System::IMU_RGBD) {
            imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
                imu_topic_, custom_qos,
                std::bind(&OrbSlam3Node::imuCallback, this, std::placeholders::_1)
            );
            RCLCPP_INFO(this->get_logger(), "Subscribed to IMU topic: %s", imu_topic_.c_str());
        }

        // Setup visual subscribers based on active mode
        if (sensor_type_ == ORB_SLAM3::System::MONOCULAR || sensor_type_ == ORB_SLAM3::System::IMU_MONOCULAR) {
            mono_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                mono_topic_, custom_qos,
                std::bind(&OrbSlam3Node::monoCallback, this, std::placeholders::_1)
            );
            RCLCPP_INFO(this->get_logger(), "Subscribed to Monocular topic: %s", mono_topic_.c_str());
        } else if (sensor_type_ == ORB_SLAM3::System::STEREO || sensor_type_ == ORB_SLAM3::System::IMU_STEREO) {
            stereo_left_sub_.subscribe(this, stereo_left_topic_, custom_qos.get_rmw_qos_profile());
            stereo_right_sub_.subscribe(this, stereo_right_topic_, custom_qos.get_rmw_qos_profile());
            stereo_sync_ = std::make_shared<message_filters::Synchronizer<ApproxSyncPolicy>>(
                ApproxSyncPolicy(queue_size_), stereo_left_sub_, stereo_right_sub_
            );
            stereo_sync_->registerCallback(
                std::bind(&OrbSlam3Node::stereoCallback, this, std::placeholders::_1, std::placeholders::_2)
            );
            RCLCPP_INFO(this->get_logger(), "Subscribed to Stereo topics: %s & %s (ApproximateTime queue_size: %d)",
                        stereo_left_topic_.c_str(), stereo_right_topic_.c_str(), queue_size_);
        } else {
            // Default: RGB-D / RGB-D-Inertial
            rgb_sub_.subscribe(this, rgb_topic_, custom_qos.get_rmw_qos_profile());
            depth_sub_.subscribe(this, depth_topic_, custom_qos.get_rmw_qos_profile());
            rgbd_sync_ = std::make_shared<message_filters::Synchronizer<ApproxSyncPolicy>>(
                ApproxSyncPolicy(queue_size_), rgb_sub_, depth_sub_
            );
            rgbd_sync_->registerCallback(
                std::bind(&OrbSlam3Node::rgbdCallback, this, std::placeholders::_1, std::placeholders::_2)
            );
            RCLCPP_INFO(this->get_logger(), "Subscribed to RGB-D topics: %s & %s (ApproximateTime queue_size: %d)",
                        rgb_topic_.c_str(), depth_topic_.c_str(), queue_size_);
        }

        path_msg_.header.frame_id = world_frame_id_;
        RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 Node Initialized successfully.");
    }

    ~OrbSlam3Node() override
    {
        if (slam_system_) {
            RCLCPP_INFO(this->get_logger(), "Shutting down ORB-SLAM3 System...");
            slam_system_->Shutdown();
        }
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg_imu)
    {
        std::lock_guard<std::mutex> lock(mutex_imu_);
        const auto& acc = msg_imu->linear_acceleration;
        const auto& gyr = msg_imu->angular_velocity;
        double t = msg_imu->header.stamp.sec + msg_imu->header.stamp.nanosec * 1e-9;
        imu_buf_.emplace_back(
            ORB_SLAM3::IMU::Point(acc.x, acc.y, acc.z, gyr.x, gyr.y, gyr.z, t)
        );
    }

    void monoCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg_mono)
    {
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(msg_mono, sensor_msgs::image_encodings::MONO8);
        } catch (const cv_bridge::Exception&) {
            try {
                cv_ptr = cv_bridge::toCvShare(msg_mono, sensor_msgs::image_encodings::BGR8);
            } catch (const cv_bridge::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge Mono exception: %s", e.what());
                return;
            }
        }

        rclcpp::Time timestamp = msg_mono->header.stamp;
        double t_frame = timestamp.seconds();

        Sophus::SE3f Twc;
        {
            std::lock_guard<std::mutex> lock(mutex_track_);
            if (sensor_type_ == ORB_SLAM3::System::IMU_MONOCULAR) {
                std::vector<ORB_SLAM3::IMU::Point> vImu;
                {
                    std::lock_guard<std::mutex> lock_imu(mutex_imu_);
                    vImu = imu_buf_;
                    imu_buf_.clear();
                }
                Twc = slam_system_->TrackMonocular(cv_ptr->image, t_frame, vImu);
            } else {
                Twc = slam_system_->TrackMonocular(cv_ptr->image, t_frame);
            }
        }

        if (slam_system_->GetTrackingState() == ORB_SLAM3::Tracking::OK) {
            publishPose(Twc, timestamp);
        }
    }

    void stereoCallback(
        const sensor_msgs::msg::Image::ConstSharedPtr& msg_left,
        const sensor_msgs::msg::Image::ConstSharedPtr& msg_right)
    {
        cv_bridge::CvImageConstPtr cv_ptr_left, cv_ptr_right;
        try {
            cv_ptr_left = cv_bridge::toCvShare(msg_left, sensor_msgs::image_encodings::MONO8);
            cv_ptr_right = cv_bridge::toCvShare(msg_right, sensor_msgs::image_encodings::MONO8);
        } catch (const cv_bridge::Exception&) {
            try {
                cv_ptr_left = cv_bridge::toCvShare(msg_left, sensor_msgs::image_encodings::BGR8);
                cv_ptr_right = cv_bridge::toCvShare(msg_right, sensor_msgs::image_encodings::BGR8);
            } catch (const cv_bridge::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge Stereo exception: %s", e.what());
                return;
            }
        }

        rclcpp::Time timestamp = msg_left->header.stamp;
        double t_frame = timestamp.seconds();

        Sophus::SE3f Twc;
        {
            std::lock_guard<std::mutex> lock(mutex_track_);
            if (sensor_type_ == ORB_SLAM3::System::IMU_STEREO) {
                std::vector<ORB_SLAM3::IMU::Point> vImu;
                {
                    std::lock_guard<std::mutex> lock_imu(mutex_imu_);
                    vImu = imu_buf_;
                    imu_buf_.clear();
                }
                Twc = slam_system_->TrackStereo(cv_ptr_left->image, cv_ptr_right->image, t_frame, vImu);
            } else {
                Twc = slam_system_->TrackStereo(cv_ptr_left->image, cv_ptr_right->image, t_frame);
            }
        }

        if (slam_system_->GetTrackingState() == ORB_SLAM3::Tracking::OK) {
            publishPose(Twc, timestamp);
        }
    }

    void rgbdCallback(
        const sensor_msgs::msg::Image::ConstSharedPtr& msg_rgb,
        const sensor_msgs::msg::Image::ConstSharedPtr& msg_depth)
    {
        cv_bridge::CvImageConstPtr cv_ptr_rgb;
        try {
            cv_ptr_rgb = cv_bridge::toCvShare(msg_rgb, sensor_msgs::image_encodings::BGR8);
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge RGB exception: %s", e.what());
            return;
        }

        cv_bridge::CvImageConstPtr cv_ptr_depth;
        try {
            cv_ptr_depth = cv_bridge::toCvShare(msg_depth, sensor_msgs::image_encodings::TYPE_32FC1);
        } catch (const cv_bridge::Exception&) {
            try {
                cv_ptr_depth = cv_bridge::toCvShare(msg_depth, sensor_msgs::image_encodings::TYPE_16UC1);
            } catch (const cv_bridge::Exception& e2) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge Depth exception: %s", e2.what());
                return;
            }
        }

        rclcpp::Time timestamp = msg_rgb->header.stamp;
        double t_frame = timestamp.seconds();

        Sophus::SE3f Twc;
        {
            std::lock_guard<std::mutex> lock(mutex_track_);
            if (sensor_type_ == ORB_SLAM3::System::IMU_RGBD) {
                std::vector<ORB_SLAM3::IMU::Point> vImu;
                {
                    std::lock_guard<std::mutex> lock_imu(mutex_imu_);
                    vImu = imu_buf_;
                    imu_buf_.clear();
                }
                Twc = slam_system_->TrackRGBD(cv_ptr_rgb->image, cv_ptr_depth->image, t_frame, vImu);
            } else {
                Twc = slam_system_->TrackRGBD(cv_ptr_rgb->image, cv_ptr_depth->image, t_frame);
            }
        }

        if (slam_system_->GetTrackingState() == ORB_SLAM3::Tracking::OK) {
            publishPose(Twc, timestamp);
        }
    }

    void publishPose(const Sophus::SE3f& Twc, const rclcpp::Time& timestamp)
    {
        Eigen::Vector3f translation = Twc.translation();
        Eigen::Quaternionf quat = Twc.unit_quaternion();

        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = timestamp;
        pose.header.frame_id = world_frame_id_;

        pose.pose.position.x = static_cast<double>(translation.x());
        pose.pose.position.y = static_cast<double>(translation.y());
        pose.pose.position.z = static_cast<double>(translation.z());
        pose.pose.orientation.x = static_cast<double>(quat.x());
        pose.pose.orientation.y = static_cast<double>(quat.y());
        pose.pose.orientation.z = static_cast<double>(quat.z());
        pose.pose.orientation.w = static_cast<double>(quat.w());

        pose_pub_->publish(pose);

        path_msg_.header.stamp = timestamp;
        path_msg_.poses.push_back(pose);
        path_pub_->publish(path_msg_);
    }

    std::string vocab_file_;
    std::string settings_file_;
    std::string sensor_mode_str_;
    ORB_SLAM3::System::eSensor sensor_type_;
    bool enable_viewer_;
    std::string mono_topic_;
    std::string rgb_topic_;
    std::string depth_topic_;
    std::string stereo_left_topic_;
    std::string stereo_right_topic_;
    std::string imu_topic_;
    std::string world_frame_id_;
    std::string camera_frame_id_;
    int queue_size_{50};

    std::unique_ptr<ORB_SLAM3::System> slam_system_;
    std::mutex mutex_track_;
    std::mutex mutex_imu_;
    std::vector<ORB_SLAM3::IMU::Point> imu_buf_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mono_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> stereo_left_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> stereo_right_sub_;

    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image,
        sensor_msgs::msg::Image
    > ApproxSyncPolicy;

    std::shared_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> rgbd_sync_;
    std::shared_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> stereo_sync_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    nav_msgs::msg::Path path_msg_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OrbSlam3Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
