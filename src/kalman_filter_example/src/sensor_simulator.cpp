/**
 * Author: Karnveer Gill
 * Sensor Simulator ROS2 node that publishes very simple noisy data.
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <random>

/**
 * @brief Class SensorSimulator node publishes noisy data to a ros2 topic 
 * 'sensor_data' every 100ms
 */
class SensorSimulator : public rclcpp::Node 
{
  public:
    // Constructor
    SensorSimulator() : Node("sensor_simulator"),
                        gen(rd()),
                        noise(0.0, 1.0)
    {
      // Initialize publisher with topic name sensor_data and msg buffer 10
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sensor_data",
                                                                        10);
      
      // Initialize timer callback with publish_data function
      timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                       std::bind(&SensorSimulator::publish_data,this));
    }

  private:
    // Members
    /**
     * @brief ROS2 shared pointer publisher for PointCould2 type messages
     */
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

    /**
     * @brief ROS2 shared pointer timer for running periodic callbacks
     */
    rclcpp::TimerBase::SharedPtr timer_;

    /**
     * @brief Random number used to seed mersenne twister algorithm. 
     * std::random_device is slow at generating so we use it to seed instead.
     */
    std::random_device rd;

    /**
     * @brief Mersenne twister pseudo-random number generator, allows for fast
     * high quality pseudo-random numbers.
     */
    std::mt19937 gen;

    /**
     * @brief Normal distribution for random number to more accurately resemble
     * real noise
     */
    std::normal_distribution<> noise;

    // Implementations
    void publish_data()
    {
      // Create 2D point cloud message
      auto msg = sensor_msgs::msg::PointCloud2();
      msg.header.stamp = this->get_clock()->now();
      msg.width = 1;
      msg.height = 1;

      // Simulate noisy data & publish
      msg.data = {static_cast<uint8_t>(noise(gen) * 100)};
      publisher_->publish(msg);
    }
};

/**
 * Main
 */
int main(int argc, char **argv)
{
  // Initialize, spin up node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorSimulator>());
  rclcpp::shutdown();
  return 0;
}