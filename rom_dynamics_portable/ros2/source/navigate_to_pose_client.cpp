#include "navigate_to_pose_client.h"
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <QObject>
#include <QDebug>

#define ROM_Q_DEBUG 1

NavigateToPoseClient::NavigateToPoseClient(const std::string &action_name) : Node("goal_action_client_node")
{
    goal_msg_ = std::make_shared<nav2_msgs::action::NavigateToPose::Goal>();

    // Create the action client
    action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, action_name);
}

void NavigateToPoseClient::onSendNavigationGoal(const geometry_msgs::msg::Pose::SharedPtr goal_pose)
{
    if (!action_client_->wait_for_action_server(std::chrono::seconds(3)))
    {
        #ifdef ROM_Q_DEBUG
            qDebug() << "[ onSendNavigationGoal()slot ] : Action server not available!";
        #endif
        
        QMetaObject::invokeMethod(this, [this]() {
        emit navigationResult("Action server not available!");
    }, Qt::QueuedConnection);

        //emit navigationResult("Action server not available!");
        return;
    }
    #ifdef ROM_Q_DEBUG
        qDebug() << "[ onSendNavigationGoal() slot] : get pose ";
    #endif
    // Setting the goal message
    goal_msg_->pose.header.stamp = this->get_clock()->now();
    goal_msg_->pose.header.frame_id = "map";
    goal_msg_->pose.pose.position = goal_pose->position;
    goal_msg_->pose.pose.orientation = goal_pose->orientation;

    // Setting options for goal
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions send_goal_options;
    send_goal_options.feedback_callback = [this](auto, auto feedback) {
        handleFeedback(nullptr, feedback);
    };
    send_goal_options.result_callback = [this](const auto &result) {
        handleResult(result);
    };

    // Sending goal
    action_client_->async_send_goal(*goal_msg_, send_goal_options);
}

void NavigateToPoseClient::handleFeedback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
{
    QString feedback_info = QString("Distance remaining: %1 meters").arg(feedback->distance_remaining);
    
    QMetaObject::invokeMethod(this, [this, feedback_info]() {
        emit navigationResult(feedback_info.toStdString());
    }, Qt::QueuedConnection);
    //emit navigationResult(feedback_info.toStdString());

    #ifdef ROM_Q_DEBUG
        qDebug() << "[ handleFeedback() callback  ] : get feedback from action server ";
    #endif
    //rclcpp::shutdown();
}

void NavigateToPoseClient::handleResult(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result)
{
    std::string status;
    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        status = "Goal succeeded!";
        break;
    case rclcpp_action::ResultCode::ABORTED:
        status = "Goal aborted!";
        break;
    case rclcpp_action::ResultCode::CANCELED:
        status = "Goal canceled!";
        break;
    default:
        status = "Unknown result code!";
        break;
    }
    #ifdef ROM_Q_DEBUG
        qDebug() << "[   handleResult() callback  ] : get result from action server ";
    #endif

    QMetaObject::invokeMethod(this, [this, status]() {
        emit navigationResult(status);
    }, Qt::QueuedConnection);
    //emit navigationResult(status);
}
