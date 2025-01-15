#include "mainwindow.h"
#include <QtWidgets/QApplication>
#include <QObject>
#include "cmd_publisher.h"
#include "pose_subscriber.h"

int main(int argc, char *argv[])
{
    /// ROS 2
    setvbuf(stdout, nullptr, _IONBF, BUFSIZ); // stdout IO NO buffer
    rclcpp::init(argc, argv);
    std::shared_ptr<Publisher> cmd_publisher = nullptr;
    std::shared_ptr<Subscriber> pose_subscriber = nullptr;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    cmd_publisher = std::make_shared<Publisher>("cmd_vel_from_qt");
    pose_subscriber = std::make_shared<Subscriber>("/odom"); // to delete
    executor->add_node(cmd_publisher);
    executor->add_node(pose_subscriber);
    std::thread executor_thread([executor](){executor->spin();});

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
   
    // ဒါရဖို့ mainwindow ရဲ့ ui ကို unique_ptr ကနေ shared_ptr ပြောင်းပြီး pointer ကို .getUi() နဲ့ ရယူတယ်။
    QObject::connect(mainWindow.getUi()->btnForward, &QPushButton::clicked, cmd_publisher.get(), &Publisher::setForward);
    QObject::connect(mainWindow.getUi()->btnForward, &QPushButton::clicked, &mainWindow, &MainWindow::labelEditForSetForward);

    QObject::connect(mainWindow.getUi()->btnLeft, &QPushButton::clicked, cmd_publisher.get(), &Publisher::setLeft);
    QObject::connect(mainWindow.getUi()->btnLeft, &QPushButton::clicked, &mainWindow, &MainWindow::labelEditForSetLeft);

    QObject::connect(mainWindow.getUi()->btnRight, &QPushButton::clicked, cmd_publisher.get(), &Publisher::setRight);
    QObject::connect(mainWindow.getUi()->btnRight, &QPushButton::clicked, &mainWindow, &MainWindow::labelEditForSetRight);

    QObject::connect(mainWindow.getUi()->btnStop, &QPushButton::clicked, cmd_publisher.get(), &Publisher::setStop);
    QObject::connect(mainWindow.getUi()->btnStop, &QPushButton::clicked, &mainWindow, &MainWindow::labelEditForSetStop);

    
    
    return a.exec();
}

