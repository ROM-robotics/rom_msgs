#include "cmd_service_client.h"

#define ROM_DEBUG 1

CmdServiceClient::CmdServiceClient(const std::string &service_name, QObject *parent)
    : QObject(parent), Node("cmd_service_client"), service_name_(service_name)
{
    // Create the service client
    client_ = this->create_client<rom_interfaces::srv::WhichVel>(service_name);

    
}

void CmdServiceClient::sendCommand(const QString &command)
{
    // Create a request
    auto request = std::make_shared<rom_interfaces::srv::WhichVel::Request>();
    request->command = command.toStdString();

    // Wait for the service to become available
    while (!client_->wait_for_service(std::chrono::seconds(1)))
    {
        #ifdef ROM_DEBUG 
            RCLCPP_WARN(this->get_logger(), "Waiting for service '%s' to become available...", service_name_.c_str());
        #endif
        emit serviceResponse(false);
        return;
    }
    #ifdef ROM_DEBUG 
        //RCLCPP_INFO(this->get_logger(), "Service client ready for '%s'", service_name_.c_str());
    #endif


    // Send the request asynchronously
    auto future = client_->async_send_request(request);
    //future.wait();


    // Process the response
    try
    {
        auto response = future.get();
        //emit commandResponse(response->success);
        #ifdef ROM_DEBUG 
            RCLCPP_INFO(this->get_logger(), "Command '%s' sent successfully: %s",
                    command.toStdString().c_str(),
                    response->status ? "true" : "false");
        #endif
    }
    catch (const std::exception &e)
    {
        //emit commandResponse(false);
        #ifdef ROM_DEBUG
            RCLCPP_ERROR(this->get_logger(), "Failed to call service: %s", e.what());
        #endif
    }
}


void CmdServiceClient::setForward(){
    sendCommand("forward");
}
void CmdServiceClient::setLeft(){
    sendCommand("left");
}
void CmdServiceClient::setRight(){
    sendCommand("right");
}
void CmdServiceClient::setStop(){
    sendCommand("stop");
}


ConstructYamlServiceClient::ConstructYamlServiceClient(const std::string &service_name, QObject *parent)
    : QObject(parent), Node("construct_yaml_client"), service_name_(service_name)
{
    // Create the service client
    goal_client_ = this->create_client<rom_interfaces::srv::ConstructYaml>(service_name);

    
}

void ConstructYamlServiceClient::onSendWaypoints(std::shared_ptr<std::unordered_map<std::string, geometry_msgs::msg::Pose>> wp_list)
{
    #ifdef ROM_DEBUG 
            RCLCPP_WARN(this->get_logger(), "getting wp_list in slot function.");
        #endif
    // Create a request
    auto request = std::make_shared<rom_interfaces::srv::ConstructYaml::Request>();
    // FOR loop ပါတ်ပြီး request ထဲထည့်ရန် 
    // Iterate over waypoints and add to the request
    for (const auto &pair : *wp_list)  // Dereferencing shared_ptr
    {
        const std::string &wp_name = pair.first;
        const geometry_msgs::msg::Pose &pose = pair.second;
        
        request->pose_names.push_back(wp_name);

        geometry_msgs::msg::PoseStamped pose_stamped;

        pose_stamped.header.frame_id = "map";
        pose_stamped.pose = pose;
        
        request->poses.push_back(pose_stamped);
    }

    // Wait for the service to become available
    while (!goal_client_->wait_for_service(std::chrono::seconds(1)))
    {
        #ifdef ROM_DEBUG 
            RCLCPP_WARN(this->get_logger(), "Waiting for service '%s' to become available...", service_name_.c_str());
        #endif
            //emit serviceResponse(false);
        return;
    }
    #ifdef ROM_DEBUG 
        RCLCPP_INFO(this->get_logger(), "Service client ready for '%s'", service_name_.c_str());
    #endif


    // Send the request asynchronously
    auto future = goal_client_->async_send_request(request);
    future.wait();

}


WaypointListSubscriber::WaypointListSubscriber(const std::string &topic_name) :  Node("wp_receiver") 
{

    rclcpp::QoS qos_profile(rclcpp::KeepLast(5)); 
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    wp_subscription_ = this->create_subscription<rom_interfaces::msg::ConstructYaml>(
        topic_name, qos_profile, std::bind(&WaypointListSubscriber::wpCallback, this, std::placeholders::_1));
}


void WaypointListSubscriber::wpCallback(const rom_interfaces::msg::ConstructYaml::SharedPtr wp_list) 
{
    #ifdef ROM_DEBUG
        qDebug() << "emit updateMap(msg)";
    #endif
    for (size_t i = 0; i < wp_list->pose_names.size(); ++i) 
    {
        wp_names_.emplace_back(wp_list->pose_names[i]);
    }
    //emit updateWpUI(wp_list);
    emit updateWpUI(wp_names_);
}  


SendWaypointsClient::SendWaypointsClient(const std::string &service_name, QObject *parent)
    : QObject(parent), Node("send_wp_client"), service_name_(service_name)
{
    // Create the service client
    wp_goal_client_ = this->create_client<rom_interfaces::srv::ConstructYaml>(service_name);

    
}

void SendWaypointsClient::onSendWaypointsGoal(std::vector<std::string> wp_names)
{
    #ifdef ROM_DEBUG 
            RCLCPP_WARN(this->get_logger(), "getting wp_list goals in slot function.");
    #endif
    // Create a request
    auto request = std::make_shared<rom_interfaces::srv::ConstructYaml::Request>();


    if(wp_names.size()<1) { return; }

    bool state;
    std::string last_element = wp_names.back();

    if(last_element=="true") { state = true; }
    else if(last_element=="false") { state = false; }

    request->loop = state;

    for (size_t i = 0; i < wp_names.size()-1; ++i) 
    {
        request->pose_names[i] = wp_names[i];
    }

    // Wait for the service to become available
    while (!wp_goal_client_->wait_for_service(std::chrono::seconds(1)))
    {
        #ifdef ROM_DEBUG 
            RCLCPP_WARN(this->get_logger(), "Waiting for service '%s' to become available...", service_name_.c_str());
        #endif
            //emit serviceResponse(false);
        return;
    }
    #ifdef ROM_DEBUG 
        RCLCPP_INFO(this->get_logger(), "Service client ready for '%s'", service_name_.c_str());
    #endif


    // Send the request asynchronously
    auto future = wp_goal_client_->async_send_request(request);
    future.wait();

}
