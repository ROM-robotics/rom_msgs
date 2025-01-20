#include "navigate_to_pose_client.h"

RosExecutorThread::RosExecutorThread(QObject* parent)
    : QThread(parent),
      executor_(std::make_shared<rclcpp::executors::MultiThreadedExecutor>()),
      node_(std::make_shared<rclcpp::Node>("qt_ros_nav2_node")),
      running_(false)
{
    action_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");
    executor_->add_node(node_);
}

RosExecutorThread::~RosExecutorThread() {
    stopThread();
}

void RosExecutorThread::startThread() {
    if (!running_) {
        running_ = true;
        start(); // Starts the thread
    }
}

void RosExecutorThread::stopThread() {
    if (running_) {
        running_ = false;
        rclcpp::shutdown();
        quit();
        wait();
    }
}

void RosExecutorThread::run() {
    //rclcpp::init(0, nullptr);
    while (running_ && rclcpp::ok()) {
        executor_->spin_some();
        QThread::msleep(10); // Prevent tight looping
    }
    rclcpp::shutdown();
}

void RosExecutorThread::sendNavigationGoal(const geometry_msgs::msg::Pose& goal_pose) {

    if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
        emit navigationResult("Action server not available");
        return;
    }
    // to start thread

    auto goal_msg = NavigateToPose::Goal();

    goal_msg.pose.header.stamp = node_->get_clock()->now();
    goal_msg.pose.header.frame_id = "map";
    
    goal_msg.pose.pose = goal_pose;

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    
    send_goal_options.feedback_callback = [this](auto, auto feedback) {
        handleFeedback(nullptr, feedback);
    };
    send_goal_options.result_callback = [this](const auto& result) {
        handleResult(result);
    };

    action_client_->async_send_goal(goal_msg, send_goal_options);
}

void RosExecutorThread::handleFeedback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback) 
{
    // Process feedback and emit a Qt signal if necessary
    QString feedback_info = QString("Distance remaining: %1 meters")
                                .arg(feedback->distance_remaining);
    emit navigationResult(feedback_info.toStdString());
}

void RosExecutorThread::handleResult(const GoalHandleNavigateToPose::WrappedResult& result) {
    std::string status;
    switch (result.code) {
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
    emit navigationResult(status);

    // stop thread
}
