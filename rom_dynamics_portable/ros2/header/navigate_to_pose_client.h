#ifndef NTP_ACTION_CLIENT_H
#define NTP_ACTION_CLIENT_H


#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <QThread>
#include <atomic>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <QDebug>

class RosExecutorThread : public QThread
{
    Q_OBJECT

public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    explicit RosExecutorThread(QObject* parent = nullptr);
    ~RosExecutorThread();

    void startThread();
    void stopThread();

    bool isRunning() { return running_; }
public slots:
    void sendNavigationGoal(const geometry_msgs::msg::Pose::SharedPtr goal_pose);

signals:
    void navigationResult(const std::string& result_status);

protected:
    void run() override;

private:
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
    std::atomic<bool> running_;

    void handleFeedback(
        GoalHandleNavigateToPose::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback);

    void handleResult(const GoalHandleNavigateToPose::WrappedResult& result);
};


#endif // NTP_ACTION_CLIENT_H
