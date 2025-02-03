#ifndef NAVIGATE_TO_POSE_CLIENT_H
#define NAVIGATE_TO_POSE_CLIENT_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <QObject>

#define ROM_DEBUG 1

class NavigateToPoseClient : public QObject, public rclcpp::Node
{
    Q_OBJECT

public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    explicit NavigateToPoseClient(const std::string &action_name);
    ~NavigateToPoseClient() = default;

public slots:
    void onSendNavigationGoal(const geometry_msgs::msg::Pose::SharedPtr goal_pose);
    void onSendCancelGoal(const rclcpp_action::GoalUUID& goal_uuid);

signals:
    void navigationResult(const std::string &result);
    void sendGoalId(const rclcpp_action::GoalUUID& goal_uuid);

private:
    void handleFeedback(
        rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
        const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback);

    void handleResult(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result);

    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
    std::shared_ptr<nav2_msgs::action::NavigateToPose::Goal> goal_msg_;
    //rclcpp::Node::SharedPtr node_;

    GoalHandleNavigateToPose::SharedPtr goal_handle_;
    rclcpp_action::GoalUUID active_goal_uuid_;
};

#endif // NAVIGATE_TO_POSE_CLIENT_H
