#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <nav2_msgs/action/follow_waypoints.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <fstream>
#include <iostream>
#include <map>

#define ROM_DEBUG 1

// လိုမှ call ခေါ်သုံးပါ။
class WaypointFollower : public rclcpp::Node {
public:
    using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
    using GoalHandle = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

    WaypointFollower(std::vector<std::string> selected_waypoints, bool loop = false)
        : Node("waypoint_follower"), loop_(loop) {

        // Load waypoints from YAML
        load_waypoints();
        if (waypoint_map_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No waypoints loaded! Exiting...");
            return;
        }

        // Filter waypoints based on user selection
        selected_waypoints_ = filter_waypoints(selected_waypoints);

        // Create action client
        client_ = rclcpp_action::create_client<FollowWaypoints>(this, "/follow_waypoints");

        // Create publisher for RViz markers
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/waypoint_markers", 10);

        // Publish markers
        publish_markers();

        // Start sending waypoints
        send_waypoints();
    }

private:
    std::map<std::string, geometry_msgs::msg::PoseStamped> waypoint_map_;
    std::vector<geometry_msgs::msg::PoseStamped> selected_waypoints_;
    bool loop_;
    rclcpp_action::Client<FollowWaypoints>::SharedPtr client_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    void load_waypoints() {
        try {
            std::string package_path = ament_index_cpp::get_package_share_directory("rom_waypoints_provider");
            std::string yaml_path = package_path + "/config/waypoints.yaml";

            YAML::Node config = YAML::LoadFile(yaml_path);
            auto waypoints_yaml = config["waypoints"];

            for (const auto &wp : waypoints_yaml) {
                std::string name = wp["name"].as<std::string>();

                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = wp["frame_id"].as<std::string>();
                pose.pose.position.x = wp["pose"]["position"]["x"].as<double>();
                pose.pose.position.y = wp["pose"]["position"]["y"].as<double>();
                pose.pose.position.z = wp["pose"]["position"]["z"].as<double>();
                pose.pose.orientation.x = wp["pose"]["orientation"]["x"].as<double>();
                pose.pose.orientation.y = wp["pose"]["orientation"]["y"].as<double>();
                pose.pose.orientation.z = wp["pose"]["orientation"]["z"].as<double>();
                pose.pose.orientation.w = wp["pose"]["orientation"]["w"].as<double>();

                waypoint_map_[name] = pose;  // Store waypoint in map
            }

            RCLCPP_INFO(this->get_logger(), "Loaded %lu waypoints.", waypoint_map_.size());
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load waypoints: %s", e.what());
        }
    }

    std::vector<geometry_msgs::msg::PoseStamped> filter_waypoints(std::vector<std::string> selected_names) {
        std::vector<geometry_msgs::msg::PoseStamped> filtered;

        if (selected_names.empty()) {
            RCLCPP_WARN(this->get_logger(), "No specific waypoints selected. Using all.");
            for (const auto &[name, pose] : waypoint_map_) {
                filtered.push_back(pose);
            }
        } else {
            for (const auto &name : selected_names) {
                if (waypoint_map_.count(name)) {
                    filtered.push_back(waypoint_map_[name]);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Waypoint '%s' not found!", name.c_str());
                }
            }
        }

        return filtered;
    }

    void send_waypoints() {
        while (rclcpp::ok()) {
            #ifndef ROM_DEBUG
            if (!client_->wait_for_action_server(std::chrono::seconds(10))) {
                RCLCPP_ERROR(this->get_logger(), "FollowWaypoints action server not available!");
                return;
            }

            auto goal_msg = FollowWaypoints::Goal();
            goal_msg.poses = selected_waypoints_;

            
            RCLCPP_INFO(this->get_logger(), "Sending %lu waypoints...", goal_msg.poses.size());
            auto future_goal = client_->async_send_goal(goal_msg);

            auto result = rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_goal);
            if (result == rclcpp::FutureReturnCode::SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "Waypoints execution completed.");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to send waypoints.");
            }
            #else
                // yaml reading ကို စစ်ဆေးရန်အတွက်သာ
                for (size_t i = 0; i < selected_waypoints_.size(); ++i) 
                {
                    auto pose = selected_waypoints_[i].pose;

                    // Using stringstream to format the log message
                    std::stringstream log_msg;
                    log_msg << "Goal pose[" << i << "] x: " << pose.position.x 
                            << " y: " << pose.position.y;

                    // Log the message
                    RCLCPP_INFO(this->get_logger(), "%s", log_msg.str().c_str());
                }
            #endif

            if (!loop_) break;
            RCLCPP_INFO(this->get_logger(), "Looping waypoints... Restarting in 5 seconds.");
            rclcpp::sleep_for(std::chrono::seconds(5));
        }
    }

    void publish_markers() {
        visualization_msgs::msg::MarkerArray marker_array;
        for (size_t i = 0; i < selected_waypoints_.size(); ++i) {
            auto wp = selected_waypoints_[i];

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.id = static_cast<int>(i);
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose = wp.pose;
            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;

            marker_array.markers.push_back(marker);
        }
        marker_pub_->publish(marker_array);
        RCLCPP_INFO(this->get_logger(), "Published waypoints to RViz.");
    }
};

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);

    // Parse CLI arguments
    bool loop = false;
    std::vector<std::string> selected_waypoints;
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--loop") {
            loop = true;
        } else {
            selected_waypoints.push_back(arg);
        }
    }

    auto node = std::make_shared<WaypointFollower>(selected_waypoints, loop);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}