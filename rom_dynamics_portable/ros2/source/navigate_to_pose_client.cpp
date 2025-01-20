#include "navigate_to_pose_client.h"

#define ROM_QDEBUG 1

RosExecutorThread::RosExecutorThread(QObject* parent)
    : QThread(parent),
      executor_(std::make_shared<rclcpp::executors::MultiThreadedExecutor>()),
      node_(std::make_shared<rclcpp::Node>("qt_ros_nav2_node")),
      running_(false)
{
    action_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");
    executor_->add_node(node_);
}

void RosExecutorThread::startThread() {
    if (!running_) {
        running_ = true;
        start(); 
    }
}

RosExecutorThread::~RosExecutorThread() {
    stopThread();
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

void RosExecutorThread::sendNavigationGoal(const geometry_msgs::msg::Pose::SharedPtr goal_pose) {


    // to start thread
    if (!running_) {
        startThread();  
        #ifdef ROM_QDEBUG
            qDebug() << "started new thread for rom dynamics";
        #endif
    }
    else {
        #ifdef ROM_QDEBUG
            qDebug() << "thread already running for rom dynamics";
        #endif
    }

    if (!action_client_->wait_for_action_server(std::chrono::seconds(3))) {
        emit navigationResult("Action server not available");

        QMetaObject::invokeMethod(this, [this]() 
        {
            if (running_) {
                qDebug() << "Killing Thread!";
                stopThread();
            }
        }, Qt::QueuedConnection);
        
        return;
    }

    auto goal_msg = NavigateToPose::Goal();

    goal_msg.pose.header.stamp = node_->get_clock()->now();
    goal_msg.pose.header.frame_id = "map";
    
    goal_msg.pose.pose.position = goal_pose->position;
    goal_msg.pose.pose.orientation = goal_pose->orientation;

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
            #ifdef ROM_QDEBUG
                qDebug() << "Goal succeeded! for rom dynamics";
            #endif
            break;
        case rclcpp_action::ResultCode::ABORTED:
            status = "Goal aborted!";
            #ifdef ROM_QDEBUG
                qDebug() << "Goal aborted! for rom dynamics";
            #endif
            break;
        case rclcpp_action::ResultCode::CANCELED:
            status = "Goal canceled!";
            #ifdef ROM_QDEBUG
                qDebug() << "Goal canceled! for rom dynamics";
            #endif
            break;
        default:
            status = "Unknown result code!";
            #ifdef ROM_QDEBUG
                qDebug() << "Unknown result code! for rom dynamics";
            #endif
            break;
    }
    emit navigationResult(status);

    // stop thread
    // if (running_) {  
    //     stopThread();
    //     #ifdef ROM_QDEBUG
    //         qDebug() << "stopping thread for rom dynamics";
    //     #endif
    // }

    // Stop the thread asynchronously
    QMetaObject::invokeMethod(this, [this]() {
        if (running_) {
            qDebug() << "Killing Thread!";
            stopThread();
        }
    }, Qt::QueuedConnection);
}

// In your RosExecutorThread class, the run() method is necessary because 
// it is an implementation of QThread::run(). When you call start() on the QThread, 
// it automatically invokes the run() method in a new thread. 
// Therefore, you do need the run() method in your RosExecutorThread class, and 
// it should handle the execution of the ROS 2 executor loop
