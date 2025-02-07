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
    std::shared_ptr<ConstructYamlServiceClient> yaml_service_client = nullptr;
    std::shared_ptr<Subscriber> pose_subscriber = nullptr;
    std::shared_ptr<ModeSubscriber> mode_subscriber = nullptr;
    std::shared_ptr<NavigateToPoseClient> goal_action_client = nullptr;
    std::shared_ptr<MapSubscriber> map_subscriber = nullptr;
    std::shared_ptr<WaypointListSubscriber> wp_subscriber = nullptr;
    std::shared_ptr<SendWaypointsClient> send_wp_client = nullptr;

    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> pose_mode_executor_mt = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> cmd_executor_st = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> action_executor_mt = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> waypoint_executor_st = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    pose_subscriber = std::make_shared<Subscriber>("/odom"); 
    mode_subscriber = std::make_shared<ModeSubscriber>("/which_nav"); 

    cmd_service_client = std::make_shared<CmdServiceClient>("which_vel");
    yaml_service_client = std::make_shared<ConstructYamlServiceClient>("/construct_yaml");
    wp_subscriber = std::make_shared<WaypointListSubscriber>("/waypoints_list");
    send_wp_client = std::make_shared<SendWaypointsClient>("/waypoints_selected");

    goal_action_client = std::make_shared<NavigateToPoseClient>("/navigate_to_pose");
    map_subscriber = std::make_shared<MapSubscriber>("/map");
    
    pose_mode_executor_mt->add_node(pose_subscriber);
    pose_mode_executor_mt->add_node(mode_subscriber);

    cmd_executor_st->add_node(cmd_service_client);
    cmd_executor_st->add_node(yaml_service_client);
    cmd_executor_st->add_node(wp_subscriber);

    waypoint_executor_st->add_node(send_wp_client);

    action_executor_mt->add_node(goal_action_client);
    action_executor_mt->add_node(map_subscriber);
    
    std::thread pose_mode_executor_thread([pose_mode_executor_mt](){pose_mode_executor_mt->spin();});
    std::thread cmd_executor_thread([cmd_executor_st](){cmd_executor_st->spin();});
    std::thread waypoint_executor_thread([waypoint_executor_st](){waypoint_executor_st->spin();});
    

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
    // construct yaml
    QObject::connect(&mainWindow, &MainWindow::sendWaypoints, yaml_service_client.get(), &ConstructYamlServiceClient::onSendWaypoints);




    // Action Goal
    QObject::connect(&mainWindow, &MainWindow::sendNavigationGoal, goal_action_client.get(), &NavigateToPoseClient::onSendNavigationGoal);
    QObject::connect(goal_action_client.get(), &NavigateToPoseClient::navigationResult, &mainWindow, &MainWindow::onNavigationResult);
    
    // Cancel Goal this need to change waypoints goal
    //QObject::connect(goal_action_client.get(), &NavigateToPoseClient::sendGoalId, &mainWindow, &MainWindow::onSendGoalId);
    //QObject::connect(&mainWindow, &MainWindow::sendCancelGoal, goal_action_client.get(), &NavigateToPoseClient::onSendCancelGoal);
    
    //map
    QObject::connect(map_subscriber.get(), &MapSubscriber::updateMap, &mainWindow, &MainWindow::onUpdateMap, Qt::QueuedConnection); // Connect the updateMap signal to onupdateMap, Qt::QueuedConnection);
    
    // subscribe wp_lists
    QObject::connect(wp_subscriber.get(), &WaypointListSubscriber::updateWpUI, &mainWindow, &MainWindow::onUpdateWpUI);

    // send waypoints goal
    //QObject::connect(mainWindow.getUi()->goAllBtn, &QPushButton::clicked, send_wp_client.get(), &SendWaypointsClient::onSendWaypointsGoalzz, Qt::UniqueConnection);
    QObject::connect(mainWindow.getUi()->goAllBtn, &QPushButton::clicked, &mainWindow, &MainWindow::onGoAllBtnClicked);
    QObject::connect(&mainWindow, &MainWindow::sendWaypointsGoal, send_wp_client.get(), &SendWaypointsClient::onSendWaypointsGoal, Qt::UniqueConnection);
    QObject::connect(send_wp_client.get(), &SendWaypointsClient::serviceWpResponse, &mainWindow, &MainWindow::onWpServiceResponse, Qt::UniqueConnection);

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