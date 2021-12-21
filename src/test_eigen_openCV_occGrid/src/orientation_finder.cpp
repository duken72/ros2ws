#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


using std::placeholders::_1;
using namespace std::chrono_literals;

class OrientationFinder : public rclcpp::Node
{
  public:
    OrientationFinder()
    : Node("orientation_finder")
    {
      subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, std::bind(&OrientationFinder::topic_callback, this, _1));
      // publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("/dominant_orientation", 10);
      // timer_ = this->create_wall_timer(500ms, std::bind(&OrientationFinder::timer_callback, this));
    }

  private:
    // void timer_callback()
    // {
    //   geometry_msgs::msg::Pose poseMsg;
    //   poseMsg.position.x = 2;
    //   RCLCPP_INFO(this->get_logger(), "Publishing pose: x = '%f'", poseMsg.position.x);
    //   publisher_->publish(poseMsg);
    // }

    void topic_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr mapMsg) const
    {
      RCLCPP_INFO(this->get_logger(), "Getting map message of size: '%i'", mapMsg->info.height);
      // OrientationFinder::height = mapMsg.info.height;
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
    // u_int32_t height;
    // rclcpp::TimerBase::SharedPtr timer_;
    // rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OrientationFinder>());
  rclcpp::shutdown();
  return 0;
}