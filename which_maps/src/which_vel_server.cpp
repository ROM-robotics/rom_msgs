#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rom_interfaces/srv/which_vel.hpp" // Replace with your package's service file if custom

const double linear_vel = 0.090;
const double angular_vel = 0.2;

//#define ROM_DEBUG 1
class CmdVelServiceNode : public rclcpp::Node
{
public:
    CmdVelServiceNode() : Node("which_vel_server")
    {
        // Create a publisher for the cmd_vel topic
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_qt_to_twist", 10);

        // Create a service that accepts a string command
        service_ = this->create_service<rom_interfaces::srv::WhichVel>(
            "which_vel",
            std::bind(&CmdVelServiceNode::handle_command, this, std::placeholders::_1, std::placeholders::_2));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&CmdVelServiceNode::publish_cmd_vel, this));
        #ifdef ROM_DEBUG
            RCLCPP_INFO(this->get_logger(), "CmdVelServiceNode started.");
        #endif
    }

private:
    void handle_command(
        const std::shared_ptr<rom_interfaces::srv::WhichVel::Request> request,
        std::shared_ptr<rom_interfaces::srv::WhichVel::Response> response)
    {
        auto command = request->command;
        geometry_msgs::msg::Twist twist;

        // Parse the command and set twist values accordingly
        if (command == "forward")
        {
            twist.linear.x = linear_vel;
            twist.angular.z = 0.0;
            should_publish_ = true;

        }
        else if (command == "stop")
        {
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            should_publish_ = true;
            should_stop_ = true;
        }
        else if (command == "left")
        {
            twist.linear.x = 0.0;
            twist.angular.z = angular_vel;
            should_publish_ = true;
        }
        else if (command == "right")
        {
            twist.linear.x = 0.0;
            twist.angular.z = -1.0 * (angular_vel);
            should_publish_ = true;
        }
        else
        {
            #ifdef ROM_DEBUG
                RCLCPP_WARN(this->get_logger(), "Unknown command: '%s'", command.c_str());
            #endif
            response->status = -1;
            twist = geometry_msgs::msg::Twist(); 
            should_publish_ = false;
            return;
        }
        current_twist_ = twist;
        response->status = 1;
    }
    void publish_cmd_vel()
    {
        if (should_publish_)
        {
            publisher_->publish(current_twist_);
            #ifdef ROM_DEBUG
                RCLCPP_INFO_ONCE(this->get_logger(), "Publishing cmd_vel...");
            #endif
            if (should_publish_ && should_stop_ ){
                //publisher_->publish(current_twist_);
                should_publish_ = false;
                should_stop_ = false;
            }
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Service<rom_interfaces::srv::WhichVel>::SharedPtr service_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::Twist current_twist_; // Stores the current Twist message
    bool should_publish_ = false; 
    bool should_stop_ = false; 

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelServiceNode>());
    rclcpp::shutdown();
    return 0;
}
