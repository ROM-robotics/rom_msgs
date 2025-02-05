#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;
std::string target_frame_, source_frame_;
class RobotPoseListener : public rclcpp::Node
{
public:
  RobotPoseListener(): Node("robot_pose_listener")
  {
    // source_frame_ = this->declare_parameter<std::string>("source_frame", "turtle1");
    // target_frame_ = this->declare_parameter<std::string>("target_frame", "turtle2");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    publisher_ = this->create_publisher<geometry_msgs::msg::Pose2D>("robot_pose", 1);

    timer_ = this->create_wall_timer(1s, std::bind(&RobotPoseListener::on_timer, this));
  }

private:
  void on_timer()
  {
   
    std::string fromFrameRel = source_frame_.c_str();
    std::string toFrameRel = target_frame_.c_str();
    
    geometry_msgs::msg::TransformStamped t;
    rclcpp::Time now = this->get_clock()->now();

        // Look up for the transformation between target_frame and turtle2 frames
        // and send velocity commands for turtle2 to reach target_frame
    try 
    {
        t = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel,tf2::TimePointZero);
    } 
    catch (const tf2::TransformException & ex) 
    {
        RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
        return;
    }
    geometry_msgs::msg::Pose2D msg;

    tf2::Quaternion q(
        t.transform.rotation.x,
        t.transform.rotation.y,
        t.transform.rotation.z,
        t.transform.rotation.w);
    
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);  // Convert quaternion to roll, pitch, yaw (in radians)

    roll = roll * (180.0 / M_PI);
    pitch = pitch * (180.0 / M_PI);
    yaw = yaw * (180.0 / M_PI);

    msg.x = t.transform.translation.x;
    msg.y = t.transform.translation.y;
    msg.theta = yaw;

    publisher_->publish(msg);
}
  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr publisher_{nullptr};
  
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  if(argc == 3)
  {
    source_frame_ = argv[1];
    target_frame_ = argv[2];
  }
  else
  {
    source_frame_ = "map";
    target_frame_ = "base_link";
  }
  rclcpp::spin(std::make_shared<RobotPoseListener>());
  rclcpp::shutdown();
  return 0;
}