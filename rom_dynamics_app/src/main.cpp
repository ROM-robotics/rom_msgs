#include "mainwindow.h"
#include <QtWidgets/QApplication>
#include <QObject>
#include "cmd_service_client.h"
#include "pose_subscriber.h"
#include "mode_subscriber.h"
#include "map_subscriber.h"

#include "navigate_to_pose_client.h"

int main(int argc, char *argv[])
{
    /// ROS 2
    setvbuf(stdout, nullptr, _IONBF, BUFSIZ); // stdout IO NO buffer
    rclcpp::init(argc, argv);
    std::shared_ptr<CmdServiceClient> cmd_service_client = nullptr;
    std::shared_ptr<Subscriber> pose_subscriber = nullptr;
    std::shared_ptr<ModeSubscriber> mode_subscriber = nullptr;
    std::shared_ptr<NavigateToPoseClient> goal_action_client = nullptr;
    std::shared_ptr<MapSubscriber> map_subscriber = nullptr;

    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> pose_mode_executor_mt = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> cmd_executor_st = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> action_executor_mt = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    //std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> map_executor_st = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();


    cmd_service_client = std::make_shared<CmdServiceClient>("which_vel");
    pose_subscriber = std::make_shared<Subscriber>("/odom"); 
    mode_subscriber = std::make_shared<ModeSubscriber>("/which_nav"); 
    goal_action_client = std::make_shared<NavigateToPoseClient>("/navigate_to_pose");
    map_subscriber = std::make_shared<MapSubscriber>("/map");
    
    pose_mode_executor_mt->add_node(pose_subscriber);
    pose_mode_executor_mt->add_node(mode_subscriber);

    cmd_executor_st->add_node(cmd_service_client);

    action_executor_mt->add_node(goal_action_client);
    action_executor_mt->add_node(map_subscriber);
    
    std::thread pose_mode_executor_thread([pose_mode_executor_mt](){pose_mode_executor_mt->spin();});
    std::thread cmd_executor_thread([cmd_executor_st](){cmd_executor_st->spin();});
    
    

    // QT APPLICATION //
    QApplication a(argc, argv);
    a.setStyleSheet(
        "QPushButton {"
        "    border: 3px solid #8f8f91;"
        "    border-radius: 25px;" // Make buttons round
        "     font-weight: bold;"
        "}"
        "QLabel {"
        "    border-radius: 25px;" // Make buttons round
        "}"
    );

    MainWindow mainWindow;

    mainWindow.show();

    std::thread action_executor_thread([action_executor_mt](){action_executor_mt->spin();});

    QObject::connect(pose_subscriber.get(), &Subscriber::logReceived, &mainWindow, &MainWindow::displayCurrentPose);
    QObject::connect(mode_subscriber.get(), &ModeSubscriber::modeReceived, &mainWindow, &MainWindow::changeCurrentMode);

   
    // ဒါရဖို့ mainwindow ရဲ့ ui ကို unique_ptr ကနေ shared_ptr ပြောင်းပြီး pointer ကို .getUi() နဲ့ ရယူတယ်။
    QObject::connect(mainWindow.getUi()->btnForward, &QPushButton::clicked, cmd_service_client.get(), &CmdServiceClient::setForward);
    QObject::connect(mainWindow.getUi()->btnForward, &QPushButton::clicked, &mainWindow, &MainWindow::labelEditForSetForward);

    QObject::connect(mainWindow.getUi()->btnLeft, &QPushButton::clicked, cmd_service_client.get(), &CmdServiceClient::setLeft);
    QObject::connect(mainWindow.getUi()->btnLeft, &QPushButton::clicked, &mainWindow, &MainWindow::labelEditForSetLeft);

    QObject::connect(mainWindow.getUi()->btnRight, &QPushButton::clicked, cmd_service_client.get(), &CmdServiceClient::setRight);
    QObject::connect(mainWindow.getUi()->btnRight, &QPushButton::clicked, &mainWindow, &MainWindow::labelEditForSetRight);

    QObject::connect(mainWindow.getUi()->btnStop, &QPushButton::clicked, cmd_service_client.get(), &CmdServiceClient::setStop);
    QObject::connect(mainWindow.getUi()->btnStop, &QPushButton::clicked, &mainWindow, &MainWindow::labelEditForSetStop);

    QObject::connect(cmd_service_client.get(), &CmdServiceClient::serviceResponse, &mainWindow, &MainWindow::onCmdServiceResponse);

    // Action Goal
    QObject::connect(&mainWindow, &MainWindow::sendNavigationGoal, goal_action_client.get(), &NavigateToPoseClient::onSendNavigationGoal);
    QObject::connect(goal_action_client.get(), &NavigateToPoseClient::navigationResult, &mainWindow, &MainWindow::onNavigationResult);
    // Cancel Goal
    QObject::connect(goal_action_client.get(), &NavigateToPoseClient::sendGoalId, &mainWindow, &MainWindow::onSendGoalId);
    QObject::connect(&mainWindow, &MainWindow::sendCancelGoal, goal_action_client.get(), &NavigateToPoseClient::onSendCancelGoal);
    
    //map
    QObject::connect(map_subscriber.get(), &MapSubscriber::updateMap, &mainWindow, &MainWindow::onUpdateMap, Qt::QueuedConnection); // Connect the updateMap signal to onupdateMap, Qt::QueuedConnection);
    
    return a.exec();
}

// void shutdown_thread()
// {
//     // Stop the ROS executor thread safely
//     if (map_executor) {
//         map_executor->cancel(); // Signal the executor to stop
//     }

//     if (map_executor_thread.joinable()) {
//         map_executor_thread.join(); // Wait for the thread to finish
//     }

//     rclcpp::shutdown();
// }