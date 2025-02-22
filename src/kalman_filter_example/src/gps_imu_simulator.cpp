/**
 * Author: Karnveer Gill
 * Simple ROS2 publisher node that simulates some GPS and IMU data
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include <random> 

class SimulateGpsImu : public rclcpp::Node
{
public:
  // Constructor
  SimulateGpsImu() : Node("sim_gps_imu")
  {
    gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps_pub", 10);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_pub", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                     std::bind(&SimulateGpsImu::publish_data, this));

    // Seed random number generator 
    std::random_device rd;
    gen_ = std::mt19937(rd());
    imu_noise_ = std::normal_distribution(0.0, 0.01); // Noise with stddev 0.01 for imu
    gps_noise_ = std::normal_distribution(0.0, 0.0001); // 0.0001 stddev noise for gps
  }

private:
  // Members
  // Need publishers
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Need some stuff for noise random number generator 
  std::mt19937 gen_;
  std::normal_distribution<> imu_noise_;
  std::normal_distribution<> gps_noise_;  

  // Implementattions
  void publish_data()
  {
    // Contants
    static const double LAT = 37.97154;
    static const double LON = -114.29408;
    static const double ALT = 30.0;
    static const double G = -9.81;

    // Build and publish GPS 
    sensor_msgs::msg::NavSatFix gps; 
    gps.latitude = LAT + gps_noise_(gen_);
    gps.longitude = LON + gps_noise_(gen_);
    gps.altitude = ALT + gps_noise_(gen_);
    gps_pub_->publish(gps);

    // Build and publish IMU
    sensor_msgs::msg::Imu imu;
    imu.linear_acceleration.x = imu_noise_(gen_);
    imu.linear_acceleration.y = imu_noise_(gen_);
    imu.linear_acceleration.z = G + imu_noise_(gen_);  // Gravity
    imu.angular_velocity.x = imu_noise_(gen_);
    imu.angular_velocity.y = imu_noise_(gen_);
    imu.angular_velocity.z = imu_noise_(gen_);
    imu_pub_->publish(imu);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimulateGpsImu>());
  rclcpp::shutdown();
  return 0;
}