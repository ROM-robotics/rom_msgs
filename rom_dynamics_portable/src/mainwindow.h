
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtWidgets>
#include <QtWidgets/QMainWindow>
#include <QThread>
#include <QLabel>
#include <QPushButton>
#include <QLineEdit>
#include "ui_mainwindow.h"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <rom_interfaces/srv/which_maps.hpp>

QT_BEGIN_NAMESPACE
namespace Ui
{
    class MainWindow;
}
QT_END_NAMESPACE

class ServiceClient : public QObject {
    Q_OBJECT

public:
    explicit ServiceClient();
    ~ServiceClient();

    void sendRequest(const std::string& request_string, const std::string& optional_param);

signals:
    void responseReceived(int status);

private:
    rclcpp::Node::SharedPtr node;
    
    rclcpp::Client<rom_interfaces::srv::WhichMaps>::SharedPtr client;
    std::thread rosThread;

    void spin();
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

    public:
        explicit MainWindow(QWidget *parent = nullptr);
        ~MainWindow();
        std::shared_ptr<Ui::MainWindow> getUi() { return ui; }

        void showBusyDialog();
        void removeBusyDialog();
        void hideBusyDialog();
        void setButtonsEnabled(bool enabled);
    
    public slots:
        void displayCurrentPose(const nav_msgs::msg::Odometry::SharedPtr msg);
        void changeCurrentMode(const std_msgs::msg::String::SharedPtr msg);
        
        void sendMappingMode();
        void sendNavigationMode();
        void sendRemappingMode();

        void saveMapClicked();
        void openMapClicked();
        void selectMapClicked();

        void labelEditForSetForward();
        void labelEditForSetRight();
        void labelEditForSetLeft();
        void labelEditForSetStop();
        
    private slots:
        void on_shutdownBtn_clicked();
        void on_btnEstop_clicked();

    private slots:
        void onResponseReceived(int sum);  

    protected:
        // for dragging
        void mousePressEvent(QMouseEvent *event) override;
        void mouseMoveEvent(QMouseEvent *event) override;
        void mouseReleaseEvent(QMouseEvent *event) override;
        // end for dragging
    
    private:
        std::shared_ptr<Ui::MainWindow> ui = nullptr;

        QSpinBox *x_spinBoxPtr_;
        QSpinBox *y_spinBoxPtr_;
        QSpinBox *z_spinBoxPtr_;

        QProgressDialog *busyDialog_ = nullptr; // Busy dialog
        
        QPushButton *sendMappingBtnPtr_;
        QPushButton *sendNavigationBtnPtr_;
        QPushButton *sendRemappingBtnPtr_;

        std::string current_mode_;
        QPushButton *saveMapBtnPtr_;
        QPushButton *openMapBtnPtr_;
        QPushButton *selectMapBtnPtr_;

        QLabel* statusLabelPtr_ = nullptr;

        ServiceClient *service_client_;
        QThread *rosThread;

        // for dragging
        bool dragging;
        QPoint dragPosition;
        // end for dragging
};

#endif // MAINWINDOW_H
