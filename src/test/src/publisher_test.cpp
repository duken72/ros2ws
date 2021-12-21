#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


using namespace std::chrono_literals;

class PublisherTest : public rclcpp::Node
{
  public:
    PublisherTest()
    : Node("publisher_test")
    {
      const auto g_map_pub_qos = rclcpp::QoS(1).reliability(
        RMW_QOS_POLICY_RELIABILITY_RELIABLE).durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
      publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", g_map_pub_qos);
      timer_ = this->create_wall_timer(500ms, std::bind(&PublisherTest::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      nav_msgs::msg::OccupancyGrid mapMsg;
      mapMsg.info.width = 70;
      mapMsg.info.height = 20;
      mapMsg.info.origin.position.x = 0.0;
      mapMsg.info.origin.position.y = 0.0;
      mapMsg.info.origin.position.z = 0.0;
      mapMsg.info.origin.orientation = geometry_msgs::msg::Quaternion();
    
      const size_t nCells = mapMsg.info.width * mapMsg.info.height;
      mapMsg.data = std::vector<int8_t>(nCells, -1);

      for (size_t occGridIdx = 0; occGridIdx < nCells; occGridIdx++) {
        if (occGridIdx < 90) {
          mapMsg.data.at(occGridIdx) = 100;
        } else {
          mapMsg.data.at(occGridIdx) = 0;
        }
      }

    RCLCPP_INFO(this->get_logger(), "Publishing map");
    publisher_->publish(mapMsg);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PublisherTest>());
  rclcpp::shutdown();
  return 0;
}
