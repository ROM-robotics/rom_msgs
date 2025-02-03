#include "navigate_to_pose_client.h"
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <QObject>
#include <QDebug>


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
        #ifdef ROM_DEBUG
            qDebug() << "[ onSendNavigationGoal()slot ] : Action server not available!";
        #endif
        
        QMetaObject::invokeMethod(this, [this]() {
        emit navigationResult("Action server not available!");
    }, Qt::QueuedConnection);

        //emit navigationResult("Action server not available!");
        return;
    }
    #ifdef ROM_DEBUG
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
    //--- action_client_->async_send_goal(*goal_msg_, send_goal_options);
    auto future_goal_handle = action_client_->async_send_goal(*goal_msg_, send_goal_options); 

    if(future_goal_handle.get())
    {
        goal_handle_ = future_goal_handle.get();

        #ifdef ROM_DEBUG
            qDebug() << "Goal Accepted";
        #endif

        rclcpp_action::GoalUUID goal_uuid = goal_handle_->get_goal_id();
        //std::string goal_id = goal_handle_->get_goal_id().uuid;

        QMetaObject::invokeMethod(this, [this, goal_uuid]() {
            emit sendGoalId(goal_uuid);
        }, Qt::QueuedConnection);
        //emit sendGoalId(goal_uuid);
    }
    else
    {
        #ifdef ROM_DEBUG
            qDebug() << "Goal Rejected";
        #endif
    }
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

    #ifdef ROM_DEBUG
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
        #ifdef ROM_DEBUG
            qDebug() << "Goal succeeded! for rom dynamics";
        #endif
        break;
    case rclcpp_action::ResultCode::ABORTED:
        status = "Goal aborted!";
        #ifdef ROM_DEBUG
            qDebug() << "Goal aborted! for rom dynamics";
        #endif
        break;
    case rclcpp_action::ResultCode::CANCELED:
        status = "Goal canceled!";
        #ifdef ROM_DEBUG
            qDebug() << "Goal canceled! for rom dynamics";
        #endif
        break;
    default:
        status = "Unknown result code!";
        #ifdef ROM_DEBUG
            qDebug() << "Unknown result code! for rom dynamics";
        #endif
        break;
    }
    #ifdef ROM_DEBUG
        qDebug() << "[   handleResult() callback  ] : get result from action server ";
    #endif

    QMetaObject::invokeMethod(this, [this, status]() {
        emit navigationResult(status);
    }, Qt::QueuedConnection);
    //emit navigationResult(status);
}

void NavigateToPoseClient::onSendCancelGoal(const rclcpp_action::GoalUUID& goal_uuid)
{
    active_goal_uuid_ = goal_uuid;

    //action_client_->cancel_goal_async(active_goal_uuid_);

    if (active_goal_uuid_.empty()) 
    {
        #ifdef ROM_DEBUG
            qDebug() << "No active goal to cancel";
        #endif
        
        QMetaObject::invokeMethod(this, [this]() {
            emit navigationResult("No active goal to cancel");
        }, Qt::QueuedConnection);

        return;
    }

    auto future_cancel = action_client_->async_cancel_all_goals();

    // error တက်တယ် ၊ spin ရှိပြီးသားမို့ မလိုဘူးယူဆတယ်။
    //rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_cancel);

    if (future_cancel.get()) 
    {
        #ifdef ROM_DEBUG
            qDebug() << "Successfully canceled goal";
        #endif
        QMetaObject::invokeMethod(this, [this]() {
            emit navigationResult("Successfully canceled goal");
        }, Qt::QueuedConnection);
    } 
    else 
    {
        #ifdef ROM_DEBUG
            qDebug() << "Failed to cancel goal";
        #endif
        QMetaObject::invokeMethod(this, [this]() {
            emit navigationResult("Failed to cancel goal");
        }, Qt::QueuedConnection);
    }
}