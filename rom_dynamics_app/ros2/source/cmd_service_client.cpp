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
