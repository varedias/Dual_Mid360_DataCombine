#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "livox_ros_driver2/msg/custom_point.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <Eigen/Dense>
#include <deque>
#include <cmath>

using std::placeholders::_1;

class MergeCloudNode : public rclcpp::Node
{
public:
  MergeCloudNode() : Node("merge_cloud_node")
  {
    // 加载标定参数
    loadCalibrationParams();
    
    cloud1_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
      "/livox/lidar_192_168_1_151", 10, 
      std::bind(&MergeCloudNode::cloud1Callback, this, _1));
      
    cloud2_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
      "/livox/lidar_192_168_1_3", 10, 
      std::bind(&MergeCloudNode::cloud2Callback, this, _1));
      
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/livox/imu_192_168_1_151", 100, 
      std::bind(&MergeCloudNode::imuCallback, this, _1));

    merged_cloud_pub_ = this->create_publisher<livox_ros_driver2::msg::CustomMsg>("/merged_cloud", 10);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/cloud_registered_body/imu", 10);
    
    RCLCPP_INFO(this->get_logger(), "MergeCloudNode with manual sync initialized.");
    RCLCPP_INFO(this->get_logger(), "Lidar1 transform: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", 
             roll1_, pitch1_, yaw1_, tx1_, ty1_, tz1_);
    RCLCPP_INFO(this->get_logger(), "Lidar2 transform: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", 
             roll2_, pitch2_, yaw2_, tx2_, ty2_, tz2_);
  }

private:
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr cloud1_sub_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr cloud2_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr merged_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  livox_ros_driver2::msg::CustomMsg::ConstSharedPtr last_cloud1_;
  livox_ros_driver2::msg::CustomMsg::ConstSharedPtr last_cloud2_;

  std::deque<sensor_msgs::msg::Imu> imu_buffer_;
  const size_t max_imu_buffer_size_ = 1000;

  livox_ros_driver2::msg::CustomMsg cached_custom_msg_;

  float roll1_ = M_PI / 2.0;
  float pitch1_ = -M_PI / 2.0;
  float yaw1_ = M_PI / 2.0;
  float tx1_ = 0.11;
  float ty1_ = 0.0;
  float tz1_ = 0.0;

  float roll2_ = 0.0;
  float pitch2_ = -M_PI / 2.0;
  float yaw2_ = 0.0;
  float tx2_ = 0.0;
  float ty2_ = 0.0;
  float tz2_ = 0.0;

  // 添加参数加载函数
  void loadCalibrationParams()
  {
    this->declare_parameter("lidar1.roll", roll1_);
    this->declare_parameter("lidar1.pitch", pitch1_);
    this->declare_parameter("lidar1.yaw", yaw1_);
    this->declare_parameter("lidar1.tx", tx1_);
    this->declare_parameter("lidar1.ty", ty1_);
    this->declare_parameter("lidar1.tz", tz1_);
    
    this->declare_parameter("lidar2.roll", roll2_);
    this->declare_parameter("lidar2.pitch", pitch2_);
    this->declare_parameter("lidar2.yaw", yaw2_);
    this->declare_parameter("lidar2.tx", tx2_);
    this->declare_parameter("lidar2.ty", ty2_);
    this->declare_parameter("lidar2.tz", tz2_);

    this->get_parameter("lidar1.roll", roll1_);
    this->get_parameter("lidar1.pitch", pitch1_);
    this->get_parameter("lidar1.yaw", yaw1_);
    this->get_parameter("lidar1.tx", tx1_);
    this->get_parameter("lidar1.ty", ty1_);
    this->get_parameter("lidar1.tz", tz1_);
    
    this->get_parameter("lidar2.roll", roll2_);
    this->get_parameter("lidar2.pitch", pitch2_);
    this->get_parameter("lidar2.yaw", yaw2_);
    this->get_parameter("lidar2.tx", tx2_);
    this->get_parameter("lidar2.ty", ty2_);
    this->get_parameter("lidar2.tz", tz2_);
  }

  void cloud1Callback(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg)
  {
    last_cloud1_ = msg;
    trySync();
  }

  void cloud2Callback(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg)
  {
    last_cloud2_ = msg;
    trySync();
  }

  void trySync()
  {
    if (!last_cloud1_ || !last_cloud2_)
      return;

    uint64_t t1 = last_cloud1_->timebase;
    uint64_t t2 = last_cloud2_->timebase;

//    RCLCPP_INFO(this->get_logger(), "cloud1 time: %lu, cloud2 time: %lu, diff: %lu", 
             //  t1, t2, (t1 > t2) ? (t1 - t2) : (t2 - t1));

    // 更严格的时间同步阈值：10ms
    if ((t1 > t2 ? (t1 - t2) : (t2 - t1)) < 10000000)
    {
      syncCallback(last_cloud1_, last_cloud2_);
      last_cloud1_.reset();
      last_cloud2_.reset();
    }
  }

  void syncCallback(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr cloud1_msg,
                    const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr cloud2_msg)
  {
    Eigen::Matrix4f transform1 = createTransformMatrix(roll1_, pitch1_, yaw1_, tx1_, ty1_, tz1_);
    Eigen::Matrix4f transform2 = createTransformMatrix(roll2_, pitch2_, yaw2_, tx2_, ty2_, tz2_);

    livox_ros_driver2::msg::CustomMsg custom_msg;
    rclcpp::Time cloud1_time(cloud1_msg->header.stamp);
    rclcpp::Time cloud2_time(cloud2_msg->header.stamp);
    
    // 使用较早的时间戳保持时序一致性
    rclcpp::Time cloud_time = (cloud1_time < cloud2_time) ? cloud1_time : cloud2_time;
    custom_msg.header.stamp = cloud_time;
    custom_msg.header.frame_id = "body";
    custom_msg.timebase = (cloud1_msg->timebase < cloud2_msg->timebase) ? 
                          cloud1_msg->timebase : cloud2_msg->timebase;
    custom_msg.lidar_id = 0;

    // 保持原始的offset_time相对关系
    for (const auto& pt : cloud1_msg->points)
    {
      Eigen::Vector4f p(pt.x, pt.y, pt.z, 1.0f);
      Eigen::Vector4f tp = transform1 * p;
      livox_ros_driver2::msg::CustomPoint cpt;
      cpt.x = tp[0]; cpt.y = tp[1]; cpt.z = tp[2];
      cpt.reflectivity = pt.reflectivity;
      cpt.tag = 0; cpt.line = pt.line;
      cpt.offset_time = pt.offset_time; // 保持原始偏移时间
      custom_msg.points.push_back(cpt);
    }

    for (const auto& pt : cloud2_msg->points)
    {
      Eigen::Vector4f p(pt.x, pt.y, pt.z, 1.0f);
      Eigen::Vector4f tp = transform2 * p;
      livox_ros_driver2::msg::CustomPoint cpt;
      cpt.x = tp[0]; cpt.y = tp[1]; cpt.z = tp[2];
      cpt.reflectivity = pt.reflectivity;
      cpt.tag = 1; cpt.line = pt.line + 16;
      cpt.offset_time = pt.offset_time; // 保持原始偏移时间
      custom_msg.points.push_back(cpt);
    }

    custom_msg.point_num = custom_msg.points.size();
    
    // 直接发布
    merged_cloud_pub_->publish(custom_msg);

    // 使用点云的原始时间戳查找IMU
    sensor_msgs::msg::Imu matched_imu;
    if (findNearestImu(cloud_time, matched_imu))
    {
      sensor_msgs::msg::Imu transformed_imu = transformImuToBody(matched_imu);
      transformed_imu.header.stamp = cloud_time; // 保持时间一致性
      imu_pub_->publish(transformed_imu);
    }

   // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                      //  "Merged cloud with %d points", custom_msg.point_num);
  }

  void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
  {
    imu_buffer_.push_back(*msg);
    if (imu_buffer_.size() > max_imu_buffer_size_)
      imu_buffer_.pop_front();
  }

  bool findNearestImu(const rclcpp::Time& cloud_time, sensor_msgs::msg::Imu& nearest_imu)
  {
    if (imu_buffer_.empty())
      return false;

    auto closest_it = imu_buffer_.begin();
    double min_diff_sec = 1e6;

    for (auto it = imu_buffer_.begin(); it != imu_buffer_.end(); ++it)
    {
      rclcpp::Duration diff = rclcpp::Time(it->header.stamp) - cloud_time;
      double diff_sec = std::abs(diff.seconds());
      if (diff_sec < min_diff_sec)
      {
        min_diff_sec = diff_sec;
        closest_it = it;
      }
    }

    if (min_diff_sec < 0.05)
    {
      nearest_imu = *closest_it;
      return true;
    }
    return false;
  }

  sensor_msgs::msg::Imu transformImuToBody(const sensor_msgs::msg::Imu& msg)
  {
    sensor_msgs::msg::Imu transformed_imu = msg;
    transformed_imu.header.frame_id = "body";

    Eigen::Quaternionf q_orig(msg.orientation.w,
                              msg.orientation.x,
                              msg.orientation.y,
                              msg.orientation.z);

    Eigen::Matrix3f R_static = (
        Eigen::AngleAxisf(yaw1_, Eigen::Vector3f::UnitZ()) *
        Eigen::AngleAxisf(pitch1_, Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(roll1_, Eigen::Vector3f::UnitX())).toRotationMatrix();

    Eigen::Quaternionf q_static(R_static);
    Eigen::Quaternionf q_transformed = q_static * q_orig;

    transformed_imu.orientation.x = q_transformed.x();
    transformed_imu.orientation.y = q_transformed.y();
    transformed_imu.orientation.z = q_transformed.z();
    transformed_imu.orientation.w = q_transformed.w();

    Eigen::Vector3f angular(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
    angular = R_static * angular;
    transformed_imu.angular_velocity.x = angular.x();
    transformed_imu.angular_velocity.y = angular.y();
    transformed_imu.angular_velocity.z = angular.z();

    Eigen::Vector3f linear(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
    linear = R_static * linear;
    transformed_imu.linear_acceleration.x = linear.x();
    transformed_imu.linear_acceleration.y = linear.y();
    transformed_imu.linear_acceleration.z = linear.z();

    return transformed_imu;
  }

  Eigen::Matrix4f createTransformMatrix(float roll, float pitch, float yaw,
                                        float x, float y, float z)
  {
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f rotation = (
        Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) *
        Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())).matrix();
    transform.block<3, 3>(0, 0) = rotation;
    transform(0, 3) = x;
    transform(1, 3) = y;
    transform(2, 3) = z;
    return transform;
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MergeCloudNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
