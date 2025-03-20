#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "rom_interfaces/srv/construct_yaml.hpp"
#include "rom_interfaces/msg/construct_yaml.hpp"
#include <iostream>
#include <filesystem>
#include <fstream>
#include <string>
#include "std_msgs/msg/bool.hpp"

#define ROM_DEBUG 1

#ifndef ROM_DYNAMICS_UNUSED
#define ROM_DYNAMICS_UNUSED(x) (void)(x)
#endif

std::string package_path;

std::string xml_path = "/home/mr_robot/data/tree/default.xml";
std::string yaml_path = "/home/mr_robot/data/waypoints/default.yaml";

// အခြား qt app များအတွက် waypoints list ကို transcient local နဲ့ ပို့ထားဖို့ပါ။
rclcpp::Publisher<rom_interfaces::msg::ConstructYaml>::SharedPtr publisher_;

void construct_xml_file(const std::shared_ptr<rom_interfaces::srv::ConstructYaml::Request> request,
          std::shared_ptr<rom_interfaces::srv::ConstructYaml::Response>      response)
{
  ROM_DYNAMICS_UNUSED(response);
  std::string request_command = request->command;

  rom_interfaces::msg::ConstructYaml message;

  // ၁။ default.xml ဖိုင် မရှိရင် ရပ်မယ်။ ( /path/to/default.xml ဖိုင် ကြိုတင်ဆောက်ပေးထားရန် )
  if (!(std::filesystem::exists(xml_path))) 
  {
    #ifdef ROM_DEBUG
      RCLCPP_INFO_STREAM(rclcpp::get_logger("xml constructor"), "File doesn't exists: creating ...");
      return;
    #endif
  }

  // ၂။ default.xml ရှိရင် ဖွင့်ပါ။ overwrite လုပ်ပါ။
  else 
  {
    // std::ofstream file(xml_path);
    std::ofstream file(xml_path, std::ios::trunc);

    // ဖွင့်မရရင် error ပြ။
    if (!file.is_open()) 
    {
      #ifdef ROM_DEBUG
        RCLCPP_ERROR(rclcpp::get_logger("xml constructor"), "file open error!!");
      #endif
      return;
    }

    // ဖွင့်လို့ရရင် tree ဆောက်မယ်။
    else 
    {
      file << "<root main_tree_to_execute=\"MainTree\">\n";
      file << "  <BehaviorTree ID=\"MainTree\">\n";
      file << "    <RecoveryNode number_of_retries=\"6\" name=\"NavigateRecovery\">\n";
      file << "      <PipelineSequence name=\"NavigateWithReplanning\">\n";
      file << "        <RateController hz=\"0.333\">\n";
      file << "          <RecoveryNode number_of_retries=\"1\" name=\"ComputePathThroughPoses\">\n";
      file << "            <ReactiveSequence>\n";
      file << "              <RemovePassedGoals input_goals=\"{goals}\" output_goals=\"{goals}\" radius=\"0.7\"/>\n";
      file << "              <ComputePathThroughPoses goals=\"{";

      for (size_t i = 0; i < request->scene_poses.size(); ++i) 
      {
        auto pose = request->poses[i]; // actual poses
          double yaw = std::atan2(2.0 * (pose.pose.orientation.w * pose.pose.orientation.z + pose.pose.orientation.x * pose.pose.orientation.y),
                                  1.0 - 2.0 * (pose.pose.orientation.y * pose.pose.orientation.y + pose.pose.orientation.z * pose.pose.orientation.z));
          file  << "{x:" << pose.pose.position.x << ", y:" << pose.pose.position.y << ", theta:" << yaw << "}";
          if (i < request->scene_poses.size() - 1) file << ", ";

          message.pose_names.push_back(request->pose_names[i]);
          message.poses.push_back(request->scene_poses[i]);
      }

      file << "}\" path=\"{path}\" planner_id=\"GridBased\"/>\n";
      file << "            </ReactiveSequence>\n";
      file << "            <ClearEntireCostmap name=\"ClearGlobalCostmap-Context\" service_name=\"global_costmap/clear_entirely_global_costmap\"/>\n";
      file << "          </RecoveryNode>\n";
      file << "        </RateController>\n";
      file << "        <RecoveryNode number_of_retries=\"1\" name=\"FollowPath\">\n";
      file << "          <FollowPath path=\"{path}\" controller_id=\"FollowPath\"/>\n";
      file << "          <ClearEntireCostmap name=\"ClearLocalCostmap-Context\" service_name=\"local_costmap/clear_entirely_local_costmap\"/>\n";
      file << "        </RecoveryNode>\n";
      file << "      </PipelineSequence>\n";
      file << "      <ReactiveFallback name=\"RecoveryFallback\">\n";
      file << "        <GoalUpdated/>\n";
      file << "        <RoundRobin name=\"RecoveryActions\">\n";
      file << "          <Sequence name=\"ClearingActions\">\n";
      file << "            <ClearEntireCostmap name=\"ClearLocalCostmap-Subtree\" service_name=\"local_costmap/clear_entirely_local_costmap\"/>\n";
      file << "            <ClearEntireCostmap name=\"ClearGlobalCostmap-Subtree\" service_name=\"global_costmap/clear_entirely_global_costmap\"/>\n";
      file << "          </Sequence>\n";
      file << "          <Spin spin_dist=\"1.57\"/>\n";
      file << "          <Wait wait_duration=\"5\"/>\n";
      file << "          <BackUp backup_dist=\"0.30\" backup_speed=\"0.05\"/>\n";
      file << "        </RoundRobin>\n";
      file << "      </ReactiveFallback>\n";
      file << "    </RecoveryNode>\n";
      file << "  </BehaviorTree>\n";
      /* ADD TREE MODEL HERE START */

      /* ADD TREE MODEL HERE END   */
      file << "</root>\n";

        // close xml file
      file.flush();
      file.close();
    }

    #ifdef ROM_DEBUG
      RCLCPP_INFO_STREAM(rclcpp::get_logger("xml constructor"), "default.xml created successfully!");
    #endif     

    // publish for other qt apps
    publisher_->publish(message);

    /** CREATE YAML FOR GENERAL PURPOSE START **/
    if (!(std::filesystem::exists(yaml_path))) 
    {
      #ifdef ROM_DEBUG
        RCLCPP_INFO_STREAM(rclcpp::get_logger("xml constructor ( yaml construct )"), "File doesn't exists: ");
      #endif
      return;
    }

    std::ofstream yaml_file(yaml_path, std::ios::trunc);
    if (!file.is_open()) 
    {
      #ifdef ROM_DEBUG
        RCLCPP_ERROR(rclcpp::get_logger("xml constructor ( yaml construct )"), "file open error!!");
      #endif
      return;
    }

    yaml_file << "waypoints:\n";
    for (size_t i = 0; i < request->pose_names.size(); ++i) 
    {
        yaml_file << "  - name: " << request->pose_names[i] << "\n";
        yaml_file << "    frame_id: " << "map" << "\n";
        yaml_file << "    pose:\n";
        yaml_file << "      position:\n";
        yaml_file << "        x: " << request->poses[i].pose.position.x << "\n";
        yaml_file << "        y: " << request->poses[i].pose.position.y << "\n";
        yaml_file << "        z: " << request->poses[i].pose.position.z << "\n";
        yaml_file << "      orientation:\n";
        yaml_file << "        x: " << request->poses[i].pose.orientation.x << "\n";
        yaml_file << "        y: " << request->poses[i].pose.orientation.y << "\n";
        yaml_file << "        z: " << request->poses[i].pose.orientation.z << "\n";
        yaml_file << "        w: " << request->poses[i].pose.orientation.w << "\n";

      #ifdef ROM_DEBUG
        RCLCPP_INFO(rclcpp::get_logger("xml constructor ( yaml construct )"), "%s", request->pose_names[i].c_str());
      #endif
    }

    yaml_file.flush();
    yaml_file.close();

    #ifdef ROM_DEBUG
      RCLCPP_INFO_STREAM(rclcpp::get_logger("xml constructor ( yaml construct )"), "waypoints.yaml created successfully!");
    #endif 
    /**  CREATE YAML FOR GENERAL PURPOSE END   **/
  }
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("construct_bt_xml_server");
  
  auto qos = rclcpp::QoS(1); 
  qos.transient_local(); 

  //အခြား qt app များမှ waypoints များကို ရယူရန် အတွက် publisher တည်ဆောက်ခြင်းဖြစ်သည်။
  publisher_ = node->create_publisher<rom_interfaces::msg::ConstructYaml>("waypoints_list", qos);

  //qt မှ wp များကို behavior tree တည်ဆောက်ပေးရန် အတွက် service တည်ဆောက်ခြင်းဖြစ်သည်။
  rclcpp::Service<rom_interfaces::srv::ConstructYaml>::SharedPtr service = node->create_service<rom_interfaces::srv::ConstructYaml>("construct_yaml_or_bt", &construct_xml_file);

  #ifdef ROM_DEBUG
    RCLCPP_INFO(rclcpp::get_logger("construct_xml"), "Ready to construct default.xml");
  #endif

  rclcpp::spin(node);
  rclcpp::shutdown();
}
// အပေါ်က ကောင်က replanning ပါတယ်။ loop တော့မပါ။


