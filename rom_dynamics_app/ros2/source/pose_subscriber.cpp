#include "pose_subscriber.h"

using namespace std::chrono_literals;

#define ROM_DEBUG 1
Subscriber::Subscriber(const std::string &topic_name) : Node("qt_robot_pose_publisher"), msgPtr_(nullptr) 
{
    subscription_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
        topic_name, 10, std::bind(&Subscriber::poseCallback, this, std::placeholders::_1));
};


void Subscriber::poseCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg) 
{
    //emit logReceived(msg);
    if(first_time_trigger_)
    {
        msgPtr_.reset();
        msgPtr_ = msg;
        first_time_trigger_ = false;
    }
    else{
        msgPtr_.reset();
        msgPtr_ = msg;
        emit logReceived(msgPtr_);
    }
    
}

void Subscriber::onMapReadyForRobotPoseSubscriber()
{
    if (!msgPtr_) {
        qDebug() << "[ Subscriber::onMapReadyForRobotPoseSubscriber ]: msgPtr_ is nullptr, skipping emit.";
        return;  // Avoid emitting a nullptr
    }
    emit logReceived(msgPtr_);

    #ifdef ROM_DEBUG
        qDebug() << "[ Subscriber::onMapReadyForRobotPoseSubscriber ]: emit logReceived(msg);";
    #endif
}
///-------------------- tf --------------------------------------

TfListener::TfListener(const std::string &node_name) 
    : rclcpp::Node(node_name), 
      tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
{
}

void TfListener::listenForTransform()
{
    rclcpp::WallRate loop_rate(10ms);  // 100Hz loop rate for transform checking

    while (rclcpp::ok()) {
        try {
            // Look up transform between 'map' and 'base_link'
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                "map", "base_link", rclcpp::Time(0), rclcpp::Duration(1, 0));  // 1 second timeout

            // Create a shared pointer to the transform object
            auto transform_ptr = std::make_shared<geometry_msgs::msg::TransformStamped>(transform);

            // Emit signal with the shared pointer to the transform
            emit transformReceived(transform_ptr);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
        }

        loop_rate.sleep();
    }
}
