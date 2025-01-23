#include "mainwindow.h"
#include <QtWidgets/QApplication>
#include <QObject>
#include "cmd_publisher.h"
#include "pose_subscriber.h"
#include "mode_subscriber.h"

#include "navigate_to_pose_client.h"

int main(int argc, char *argv[])
{
    /// ROS 2
    setvbuf(stdout, nullptr, _IONBF, BUFSIZ); // stdout IO NO buffer
    rclcpp::init(argc, argv);
    std::shared_ptr<Publisher> cmd_publisher = nullptr;
    std::shared_ptr<Subscriber> pose_subscriber = nullptr;
    std::shared_ptr<ModeSubscriber> mode_subscriber = nullptr;
    std::shared_ptr<NavigateToPoseClient> goal_action_client = nullptr;

    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> single_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> action_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    cmd_publisher = std::make_shared<Publisher>("cmd_vel_qt_to_twist");
    pose_subscriber = std::make_shared<Subscriber>("/odom"); 
    mode_subscriber = std::make_shared<ModeSubscriber>("/which_nav"); 
    goal_action_client = std::make_shared<NavigateToPoseClient>("/navigate_to_pose");

    single_executor->add_node(cmd_publisher);
    
    executor->add_node(pose_subscriber);
    executor->add_node(mode_subscriber);

    action_executor->add_node(goal_action_client);
    
    std::thread executor_thread([executor](){executor->spin();});
    std::thread single_executor_thread([single_executor](){single_executor->spin();});
    std::thread action_executor_thread([action_executor](){action_executor->spin();});

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

    QObject::connect(pose_subscriber.get(), &Subscriber::logReceived, &mainWindow, &MainWindow::displayCurrentPose);
    QObject::connect(mode_subscriber.get(), &ModeSubscriber::modeReceived, &mainWindow, &MainWindow::changeCurrentMode);
   
    // ဒါရဖို့ mainwindow ရဲ့ ui ကို unique_ptr ကနေ shared_ptr ပြောင်းပြီး pointer ကို .getUi() နဲ့ ရယူတယ်။
    QObject::connect(mainWindow.getUi()->btnForward, &QPushButton::clicked, cmd_publisher.get(), &Publisher::setForward);
    QObject::connect(mainWindow.getUi()->btnForward, &QPushButton::clicked, &mainWindow, &MainWindow::labelEditForSetForward);

    QObject::connect(mainWindow.getUi()->btnLeft, &QPushButton::clicked, cmd_publisher.get(), &Publisher::setLeft);
    QObject::connect(mainWindow.getUi()->btnLeft, &QPushButton::clicked, &mainWindow, &MainWindow::labelEditForSetLeft);

    QObject::connect(mainWindow.getUi()->btnRight, &QPushButton::clicked, cmd_publisher.get(), &Publisher::setRight);
    QObject::connect(mainWindow.getUi()->btnRight, &QPushButton::clicked, &mainWindow, &MainWindow::labelEditForSetRight);

    QObject::connect(mainWindow.getUi()->btnStop, &QPushButton::clicked, cmd_publisher.get(), &Publisher::setStop);
    QObject::connect(mainWindow.getUi()->btnStop, &QPushButton::clicked, &mainWindow, &MainWindow::labelEditForSetStop);

    // Action Goal
    QObject::connect(&mainWindow, &MainWindow::sendNavigationGoal, goal_action_client.get(), &NavigateToPoseClient::onSendNavigationGoal);
    QObject::connect(goal_action_client.get(), &NavigateToPoseClient::navigationResult, &mainWindow, &MainWindow::onNavigationResult);
    // Cancel Goal
    QObject::connect(goal_action_client.get(), &NavigateToPoseClient::sendGoalId, &mainWindow, &MainWindow::onSendGoalId);
    QObject::connect(&mainWindow, &MainWindow::sendCancelGoal, goal_action_client.get(), &NavigateToPoseClient::onSendCancelGoal);
    
    
    return a.exec();
}

//     // Start ROS thread
//     rosThread.startThread();

//     QObject::connect(&app, &QApplication::aboutToQuit, [&]() {
//         rosThread.stopThread();
//     });

//     return app.exec();