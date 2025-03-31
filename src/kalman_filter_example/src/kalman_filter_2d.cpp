/**
 * Author: Karnveer Gill
 * 2D Kalman filter to estimate true GPS pose with IMU aiding
 */

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using Eigen::Vector2d;
using Eigen::Matrix2d;
using namespace std::chrono_literals;

class KalmanFilter2DNode : public rclcpp::Node
{
public:
  // Constructor
  KalmanFilter2DNode() : Node("kalman_filter_2d"),
                         last_imu_t_(0)
  {
    RCLCPP_INFO(this->get_logger(), "CONSTRUCTING KALMAN FILTER 2D NODE");

    // Create subscriptions to GPS and IMU data
    gps_sub_ = this->create_subscription
                        <sensor_msgs::msg::NavSatFix>("gps_pub", 
                                                      10, 
                                                      std::bind(&KalmanFilter2DNode::gps_callback,
                                                                this,
                                                                std::placeholders::_1));
    imu_sub_ = this->create_subscription
                        <sensor_msgs::msg::Imu>("imu_pub",
                                                10,
                                                std::bind(&KalmanFilter2DNode::imu_callback,
                                                          this,
                                                          std::placeholders::_1));

    // Create publisher for filtered position data
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("filtered_pose",
                                                                        10);

    // Initialize kalman filter values
    estimates_ = Vector2d(0, 0); // Set 0,0 we will initialize this with first GPS hit
    estimates_covariance_ = Matrix2d::Identity() * 1;  // Modestly trust our intial estimate
    system_covariance_ = Matrix2d::Identity() * 0.001; // Highly trust our model
    sensor_covariance_ = Matrix2d::Identity() * 0.1;   // Modestly trust our sensor

    RCLCPP_INFO(this->get_logger(), "KALMAN FILTER 2D NODE CONSTRUCTED");
  }

private:
  // Members
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  // State estimates for X and Y position in 2d matrix
  Vector2d estimates_;

  // Uncertainties in our estimates, system process, and sensor noise
  Matrix2d estimates_covariance_;
  Matrix2d system_covariance_;
  Matrix2d sensor_covariance_;

  // Track last time imu received 
  rclcpp::Time last_imu_t_;

  // Flag to initialize position filter with first GPS hit
  bool first_gps_hit_{true};

  // Implementations
  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    // RCLCPP_INFO(this->get_logger(), 
    //             "GPS Callback hit! Received: LAT=%f  LON=%f", 
    //             msg->latitude,
    //             msg->longitude);

    Vector2d pose_update(msg->latitude, msg->longitude);
    if(first_gps_hit_)
    {
      // Initialize position filter with first GPS position
      estimates_ = pose_update;
      first_gps_hit_ = false;
    }
    else
    {
      kalman_update(pose_update);
    }
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // RCLCPP_INFO(this->get_logger(), 
    //             "IMU Callback hit! Received Lin-Accel-X=%f  Lin-Accel-Y=%f",
    //             msg->linear_acceleration.x,
    //             msg->linear_acceleration.y);

    rclcpp::Time curr_t = msg->header.stamp;

    // If first entry, save time and leave
    if(last_imu_t_.nanoseconds() == 0)
    {
      last_imu_t_ = curr_t;
      return;
    }

    double dt = (curr_t - last_imu_t_).seconds();
    last_imu_t_ = curr_t;

    // Avoid invalid timestamp and division by zero
    if (dt <= 0) return;

    // simple integration for position estimate
    estimates_ += dt * dt * Vector2d(msg->linear_acceleration.x,
                                     msg->linear_acceleration.y);
  }

  void kalman_update(const Vector2d &measurment)
  {
    // Update estimate uncertainty
    estimates_covariance_ = estimates_covariance_ + system_covariance_;

    // Calculate kalman gain
    Matrix2d S = estimates_covariance_ + sensor_covariance_;
    Matrix2d K = estimates_covariance_ * S.inverse();

    // Update estimate
    estimates_ = estimates_ + K * (measurment - estimates_);
    estimates_covariance_ = (Matrix2d::Identity() - K) * estimates_covariance_;

    // Publish filtered pose
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->get_clock()->now();
    pose_msg.pose.position.x = estimates_[0];
    pose_msg.pose.position.y = estimates_[1];
    pose_pub_->publish(pose_msg);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KalmanFilter2DNode>());
  rclcpp::shutdown();
  return 0;
}