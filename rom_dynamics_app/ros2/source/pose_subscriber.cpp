#include "pose_subscriber.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
using namespace std::chrono_literals;

//#define ROM_DEBUG 1
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
    #ifdef ROM_DEBUG
        qDebug() << "[  TfListener::TfListener   ] : Constructor";
    #endif

    // Create a timer with a 100ms period
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), 
        std::bind(&TfListener::listenForTransform, this)
    );
}


void TfListener::listenForTransform()
{
        try {
            // Look up transform between 'map' and 'odom'
            geometry_msgs::msg::TransformStamped map_odom_tf = tf_buffer_->lookupTransform("map", "odom", rclcpp::Time(0), rclcpp::Duration(1, 0));  // 1 second timeout
            geometry_msgs::msg::TransformStamped odom_footprint_tf = tf_buffer_->lookupTransform("map", "base_footprint", rclcpp::Time(0), rclcpp::Duration(1, 0));
            
            // Create shared pointers to the transform objects
            auto map_odom_tf_ptr = std::make_shared<geometry_msgs::msg::TransformStamped>(map_odom_tf);
            auto odom_footprint_tf_ptr = std::make_shared<geometry_msgs::msg::TransformStamped>(odom_footprint_tf);

            // Extract yaw from the quaternion rotation
            rtf_.map_odom_x = map_odom_tf_ptr->transform.translation.x;
            rtf_.map_odom_y = map_odom_tf_ptr->transform.translation.y;

        tf2::Quaternion map_odom_quat;
        tf2::fromMsg(map_odom_tf_ptr->transform.rotation, map_odom_quat);  // Convert geometry_msgs::Quaternion to tf2::Quaternion
        tf2::Matrix3x3 mat(map_odom_quat);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);  
        rtf_.map_odom_yaw = yaw;  

            rtf_.odom_base_footprint_x = odom_footprint_tf_ptr->transform.translation.x;
            rtf_.odom_base_footprint_y = odom_footprint_tf_ptr->transform.translation.y;
        
        tf2::Quaternion odom_footprint_quat;
        tf2::fromMsg(odom_footprint_tf_ptr->transform.rotation, odom_footprint_quat);  
        
        tf2::Matrix3x3 mat2(odom_footprint_quat);
        double r, p, y;
        mat2.getRPY(r, p, y); 
        rtf_.odom_base_footprint_yaw = y;  
            
        #ifdef ROM_DEBUG
                qDebug() << "[  TfListener::listenForTransform   ]  :  get transform ";
        #endif
            // Emit signal with the shared pointers to the transforms
            emit transformReceived(rtf_);
        } catch (const tf2::TransformException &ex) {
            #ifdef ROM_DEBUG
                qDebug() << "[  TfListener::listenForTransform   ]  :  Could not get transform ";
            #endif
        }
}
