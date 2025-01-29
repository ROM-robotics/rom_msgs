#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "rom_interfaces/srv/which_maps.hpp"
#include <filesystem>

#include <memory>

#include <std_msgs/msg/string.hpp>
#include <stdexcept>
#include <string>
#include <memory>
#include <cstdlib>
#include <unistd.h>
#include <sys/wait.h>

#define ROM_DEBUG 1

std::string package_name = "which_maps";
bool map_topic_exists = true;

std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> global_publisher;
auto trigger_msg = std_msgs::msg::String();

// switch mode parameters
std::string current_mode = "navi";
pid_t launch_pid = -1;

// Package and launch file names
const std::string robot_name = std::getenv("ROM_ROBOT_MODEL");

const std::string cartographer_pkg = robot_name +"_carto";                 
const std::string carto_mapping_launch = "mapping.launch.py";
    
const std::string carto_localization_launch = "localization.launch.py";
    
const std::string remapping_launch = "something.launch.py";

std::string package_directory = "/home/mr_robot/data/maps/";


// /home/mr_robot/devel_ws/install/rom2109_carto/share/rom2109_carto/launch/localization.launch.py
// /ros2_ws/install/bobo_carto/share/bobo_carto/launch/localization.launch.py

// /home/mr_robot/devel_ws/install/rom2109_nav2/share/rom2109_nav2/config/nav2_params.yaml
// /ros2_ws/install/bobo_nav2/share/bobo_nav2/config/nav2_params.yaml


// switch_mode functions
void startLaunch(const std::string &package, const std::string &launch_file) 
{
      #ifdef ROM_DEBUG
        RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "Starting launch file: %s/%s", package.c_str(), launch_file.c_str());
      #endif
        // Fork a proce  to run the launch file
        launch_pid = fork();
        if (launch_pid == 0) {
            // In child process
            execlp("ros2", "ros2", "launch", package.c_str(), launch_file.c_str(), (char *)NULL);
            perror("execlp failed");
            std::exit(EXIT_FAILURE);
        }

        if (launch_pid < 0) {
            perror("fork failed");
            throw std::runtime_error("Failed to start launch process");
        }
}

void shutdownLaunch() 
{
        if (launch_pid > 0) {
          #ifdef ROM_DEBUG
            RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "Shutting down current launch process (PID: %d)...", launch_pid);
          #endif
            kill(launch_pid, SIGINT);
            int status;
            waitpid(launch_pid, &status, 0);
            launch_pid = -1;
            sleep(3); // Ensure the process has fully terminated
        }
}




// TODO 1. GO TO HOME 'S MAP DIRECTORY OR NOT
// INPUT ( which_map_do_you_have | save_map | select_map | mapping | navi | remapping )

// # -1 [service not ok], 1 [service ok], 
// # 2 [maps exist], 3 [maps do not exist], 
// # 4 [save map ok], 5 [save map not ok]
// # 6 [select map ok], 7 [select map not ok], 

// # 8 [mapping mode ok], 9 [mapping mode not ok],
// # 10 [navi mode ok], 11 [navi mode not ok]
// # 12 [remapping mode ok], 13 [remapping mode not ok]


void which_map_answer(const std::shared_ptr<rom_interfaces::srv::WhichMaps::Request> request,
          std::shared_ptr<rom_interfaces::srv::WhichMaps::Response>      response)
{
  /// ၁။ ဘယ်မြေပုံတွေရှိလဲ?။
  if(request->request_string == "which_maps_do_you_have"){
    // if(check existance of map directory)
    
    //RCLCPP_INFO(rclcpp::get_logger("which_maps"), "%s", package_directory.c_str());
    int yaml_file_count = 0;
    //std::string[] name_array;

    try 
    {
        for (const auto& entry : std::filesystem::directory_iterator(package_directory)) 
        {
            if (entry.is_regular_file() && entry.path().extension() == ".yaml" )//&& entry.path().extension() == ".pgm" && entry.path().extension() == ".pbstream") 
            {
                //std::cout << "YAML File: " << entry.path().filename() << std::endl;
                response->map_names.push_back(entry.path().filename().c_str());

                #ifdef ROM_DEBUG
                  RCLCPP_INFO(rclcpp::get_logger("which_maps"), "%s", entry.path().filename().c_str());
                #endif
                ++yaml_file_count;
            }
        }
        std::cout << "Total .yaml files: " << yaml_file_count << std::endl;
        response->status = 2; // ok
        response->total_maps = yaml_file_count;

        #ifdef ROM_DEBUG
          RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "Sending : Response Status OK");
        #endif 

    } 
    catch (const std::filesystem::filesystem_error& e) 
    {
        std::cerr << "Error accessing directory: " << e.what() << std::endl;
        response->total_maps = 0;
        response->status = 3; // not ok
        #ifdef ROM_DEBUG
          RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "Sending : Response Status not OK");
        #endif 
    }
  }

  
  /// ၂။ မြေပုံ save ပါ။
  else if (request->request_string == "save_map"){
    // map saver
    std::string map_name = request->map_name_to_save;
    
    if (map_topic_exists) 
    {
      #ifdef ROM_DEBUG
        RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "The /map topic exists.");
        RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "map: %s", map_name.c_str());
      #endif 

      // Save the map
      // command စစ်ဆေးရန်။
      std:: string cmd = "cd /home/mr_robot/data/maps/ && touch " + map_name + "_hack; rm " + map_name + "* && ros2 service call /write_state cartographer_ros_msgs/srv/WriteState '{filename: '/home/mr_robot/data/maps/" +  map_name + ".pbstream', include_unfinished_submaps: true}'";

      // first command
      int ret_code = std::system(cmd.c_str());

      if(ret_code == 0) {
        #ifdef ROM_DEBUG
          RCLCPP_INFO(rclcpp::get_logger("which_map_server"), "Map saver command 1 executed successfully.");
        #endif 

        std::string cmd2 = std::string("ros2 run cartographer_ros cartographer_pbstream_to_ros_map ") +
                            "-pbstream_filename /home/mr_robot/data/maps/" + map_name + ".pbstream " +
                            "-map_filename /home/mr_robot/data/maps/" + map_name + ".pgm " +
                            "-yaml_filename /home/mr_robot/data/maps/" + map_name + ".yaml && "  
                            "mv /home/mr_robot/map.pgm /home/mr_robot/data/maps/"  + map_name + ".pgm && "
                            "mv /home/mr_robot/map.yaml /home/mr_robot/data/maps/"  + map_name + ".yaml";
        // second command
        int ret_code2 = std::system(cmd2.c_str());

        if (ret_code2 == 0)
        {
          #ifdef ROM_DEBUG
            RCLCPP_INFO(rclcpp::get_logger("which_map_server"), "Map saver command 2 executed successfully.");
          #endif 

          std::string cmd3 = "sed -i \"s|maps/.*\\.pbstream|maps/" + map_name + ".pbstream|g\" /home/mr_robot/devel_ws/install/rom2109_carto/share/rom2109_carto/launch/localization.launch.py";
          int ret_code3 = std::system(cmd3.c_str());
          if(ret_code3==0)
          {
            #ifdef ROM_DEBUG
              RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "sed command 3 OK");
            #endif 

            std::string cmd4 = "sed -i \"s|maps/.*\\.yaml|maps/" + map_name + ".yaml|g\" /home/mr_robot/devel_ws/install/rom2109_nav2/share/rom2109_nav2/config/nav2_params.yaml";

            int ret_code4 = std::system(cmd4.c_str());

            if(ret_code4==0) 
            { 
              #ifdef ROM_DEBUG
                RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "sed command 4 OK"); 
              #endif 
              std::string cmd5 = "sed -i \"s|^image: .*\\.pgm|image: " + map_name + ".pgm|\" /home/mr_robot/data/maps/" + map_name + ".yaml";
              int ret_code5 = std::system(cmd5.c_str());
              if(ret_code5==0) 
              { 
                #ifdef ROM_DEBUG
                  RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "sed command 5 Ok."); response->status = 4;
                #endif 
              }
              else 
              { 
                #ifdef ROM_DEBUG
                  RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "sed command 5 Fail"); response->status = 5;
                #endif 
                
              } 
            }
            else             
            { 
              #ifdef ROM_DEBUG
                RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "sed command 4 Fail"); 
              #endif
              response->status = 5; 
            }
          }
          else
          {
            response->status = 5; // not ok
            #ifdef ROM_DEBUG
              RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "sed command 3 Fail");
            #endif
            return;
          }
          

        } 
        else 
        {
          #ifdef ROM_DEBUG
            RCLCPP_ERROR(rclcpp::get_logger("which_map_server"), "Map saver command 2 failed with return code: %d", ret_code2);
            RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "Sending : Response Status not OK");
          #endif 
          response->status = 5; // not ok
        }
      }
      else 
      {
        #ifdef ROM_DEBUG
          RCLCPP_ERROR(rclcpp::get_logger("which_map_server"), "Map saver command 1 failed with return code: %d", ret_code);
          RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "Sending : Response Status not OK");
        #endif
        response->status = 5; // not ok
      }    
    } 
    else 
    {
      #ifdef ROM_DEBUG
        RCLCPP_WARN(rclcpp::get_logger("which_maps_server"), "The /map topic does not exist.");
        RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "Sending : Response Status not OK");
      #endif
      response->status = 5; // not ok
    }

    
  }


  /// ၃။ မြေပုံရွေးပါ။
  else if (request->request_string == "select_map") // Select map ထည့်၇န်
  {   
    std::string map_name = request->map_name_to_select;
    if(map_name == "")
    {
      #ifdef ROM_DEBUG
        RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "Sending : No Map Name ");
      #endif
      response->status = 7; // not ok
      // should return or not ???
    }
    else 
    {
      #ifdef ROM_DEBUG
        RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "Sending : Response Status OK");
      #endif

      ////////////////////////////////////////////////////////////////////////////////////////
      // ၁ ၊ name.pgm , name.yaml , name.pbstream ရှိမရှိစစ်ဆေးပါ။ မရှိရင် error (7)
      

      std::string pgm_file = package_directory + map_name + ".pgm";
      std::string yaml_file = package_directory + map_name + ".yaml";
      std::string pbstream_file = package_directory + map_name + ".pbstream";

      #ifdef ROM_DEBUG
        RCLCPP_INFO_STREAM(rclcpp::get_logger("logger_name"), pgm_file);
      #endif

      if (std::filesystem::exists(pgm_file) && std::filesystem::exists(yaml_file) && std::filesystem::exists(pbstream_file)) 
      {
        #ifdef ROM_DEBUG
          RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), " : All required files exist for map:");
        #endif
        // Do Your Work
        // ၂ ၊ nav2_param.yaml နဲ့ carto localization launch မှာ သက်ဆိုင်ရာ yaml နဲ့ pbstream ပြင်ပါ။ မရရင် error(7) 
        std::string cmd1 = "sed -i \"s|maps/.*\\.pbstream|maps/" + map_name + ".pbstream|g\" /home/mr_robot/devel_ws/install/rom2109_carto/share/rom2109_carto/launch/localization.launch.py";
        int ret_code1 = std::system(cmd1.c_str());
        if(ret_code1==0)
        {
          // execute command 2 
          std::string cmd2 = "sed -i \"s|maps/.*\\.yaml|maps/" + map_name + ".yaml|g\" /home/mr_robot/devel_ws/install/rom2109_nav2/share/rom2109_nav2/config/nav2_params.yaml";
          int ret_code2 = std::system(cmd2.c_str());
          if(ret_code2==0)
          {
            #ifdef ROM_DEBUG
              RCLCPP_INFO(rclcpp::get_logger("select_maps"), " : Map Select Ok, Relaunching ... ");
            #endif

             // ၃ ၊ carto နဲ့ navigation  ကို relaunch လုပ်ပါ။
            shutdownLaunch();
            startLaunch(cartographer_pkg, carto_localization_launch);
            #ifdef ROM_DEBUG
              RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), " Map Relaunch OK");
            #endif
            current_mode = "navi";
      
            trigger_msg.data = "navi";
            global_publisher->publish(trigger_msg);
            response->status = 6; // ok
          }
          else 
          {
            #ifdef ROM_DEBUG
              RCLCPP_INFO(rclcpp::get_logger("select_maps"), " : Command 2 Execute Failed ");
            #endif
            response->status = 7; // not ok
            // should return or not ??
          }
        }
        else
        {
          #ifdef ROM_DEBUG
            RCLCPP_INFO(rclcpp::get_logger("select_maps"), " : Command 1 Execute Failed ");
          #endif
          response->status = 7; // not ok
          // should return or not ??
        }
      } 
      else 
      {
        #ifdef ROM_DEBUG
          RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), " : Not Exist ");
        #endif
        response->status = 7; // not ok
        // should return or not ??
      
      } 
    }
  }

  /// ၄။ mapping mode change ပါ။
  else if (request->request_string == "mapping")
  {
    if (current_mode == "mapping" ) 
    {
      #ifdef ROM_DEBUG
        RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "Mode '%s' is already active.", current_mode.c_str());
      #endif
    }
    else 
    {
      shutdownLaunch();


      #ifdef ROM_DEBUG
        RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "Sending : Response Status OK");
      #endif
      response->status = 8; // ok
      startLaunch(cartographer_pkg, carto_mapping_launch );
      current_mode = "mapping";

      trigger_msg.data = "mapping";
      global_publisher->publish(trigger_msg);
    }
  }
  
  /// ၅။ nav mode change ပါ။
  else if (request->request_string == "navi")
  {
    if (current_mode == "navi" ) 
    {
      #ifdef ROM_DEBUG
        RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "Mode '%s' is already active.", current_mode.c_str());
      #endif
    }
    else 
    {
      shutdownLaunch();
      response->status = 10; // ok
      startLaunch(cartographer_pkg, carto_localization_launch);
      #ifdef ROM_DEBUG
        RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "Sending : Response Status OK");
      #endif
      current_mode = "navi";
      
      trigger_msg.data = "navi";
      global_publisher->publish(trigger_msg);
    }
  }
  
  
  /// 6။ remapping mode change ပါ။
  else if (request->request_string == "remapping")
  {
    if (current_mode == "remapping") 
    {
      #ifdef ROM_DEBUG
        RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "Mode '%s' is already active.", request->request_string.c_str());
      #endif
    }
    else 
    {
      shutdownLaunch();
      startLaunch(cartographer_pkg, remapping_launch);
      response->status = 12; // ok
      #ifdef ROM_DEBUG
        RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "Sending : Response Status OK");
      #endif
      current_mode = "remapping";

      trigger_msg.data = "remapping";
      global_publisher->publish(trigger_msg);
    }   
  }

  /// 7. ဘာမှမလုပ်
  else 
  {
    response->total_maps = 0;
    response->status = -1; // not ok
    #ifdef ROM_DEBUG
      RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "Sending : Response Status not OK");
      RCLCPP_WARN(rclcpp::get_logger("which_maps_server"), "Unknown mode '%s'.", request->request_string.c_str());
    #endif
  }

  /// ၇။ ဘာမှမဟုတ်။

}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("which_maps_server");

  rclcpp::Service<rom_interfaces::srv::WhichMaps>::SharedPtr service = node->create_service<rom_interfaces::srv::WhichMaps>("which_maps", &which_map_answer);

  // latch publisher
  global_publisher = node->create_publisher<std_msgs::msg::String>("which_nav", rclcpp::QoS(1).transient_local());

  #ifdef ROM_DEBUG
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to answer maps.");
    RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "Fist Time trigger to Nav mode");
  #endif

  startLaunch(cartographer_pkg, carto_localization_launch);
  
  trigger_msg.data = current_mode;
  global_publisher->publish(trigger_msg);

  rclcpp::spin(node);
  rclcpp::shutdown();
}