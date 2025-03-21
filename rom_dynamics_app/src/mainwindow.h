
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtWidgets>
#include <QtWidgets/QMainWindow>
#include <QThread>
#include <QLabel>
#include <QPushButton>
#include <QLineEdit>
#include <QDial>
#include <QWidget>
#include <QSpinBox>
#include <QVBoxLayout>
#include "ui_mainwindow.h"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <rom_interfaces/srv/which_maps.hpp>
#include <rom_interfaces/srv/construct_yaml.hpp>
#include <rom_interfaces/msg/construct_yaml.hpp>
#include <QPropertyAnimation>
#include <QEasingCurve>

// ---------------------mapping app
#include <QGraphicsView>
#include <QMouseEvent>
#include <QDebug>
#include <unordered_map>
// --------------------mapping app

#include <geometry_msgs/msg/pose2_d.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "rom_structures.h"
//#include "eyeswidget.h"
#include "roboteyeemotionwindow.h"

//#define ROM_DEBUG 1

QT_BEGIN_NAMESPACE
namespace Ui
{
    class MainWindow;
}
QT_END_NAMESPACE

class CustomDialog : public QDialog {
    Q_OBJECT

public:
    explicit CustomDialog(QWidget *parent = nullptr);
    QString getName() const;
    int getNumber() const;

private slots:
    void validateInput();
    
private:
    QLineEdit *nameEdit;
    QSpinBox *numberEdit;
    QDial *dial; 
    QPushButton *okButton;
};
//---------------------------- END OF CUSTOM DIALOG ----------------------------

class ServiceClient : public QObject {
    Q_OBJECT

public:
    explicit ServiceClient();
    ~ServiceClient();

    void sendRequest(const std::string& request_string, const std::string& optional_param);

signals:
    void responseReceived(int status);
    void responseDataReceived(std::shared_ptr<rom_interfaces::srv::WhichMaps::Response> response);

private:
    rclcpp::Node::SharedPtr node;
    
    rclcpp::Client<rom_interfaces::srv::WhichMaps>::SharedPtr client_;
    std::thread rosServiceClientThread_;

    void spin();
};

// ------------------------- END OF SERVICE CLIENT ----------------------------`

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

        void toggleButtonWithAnimation(QPushButton* button, bool show);

        void applyStyles();

        void applyStyleWaypoint();
        void applyStyleWall();
        void applyStyleEraser();
        void applyStyleZoom();
        void applyStyleNormal();
        void applyStyleServiceMode();

        void showSceneOriginCoordinate();
        void showMapOriginCooridinate();

        // ------------------- map app
        //void mousePressEvent(QMouseEvent *event); check protected
        void wheelEvent(QWheelEvent *event);
        
        bool checkGraphicViewAndScene();
        void setDragMode(bool state);
        // ------------------- map app

        void romSendWpGoals();
        void showOdomAndBaseFootprint();
        void showOdom();
        void showBaseFootprint();

        //void processScan(QPointF origin);
        void processScan();

    signals:
        void sendNavigationGoal(const geometry_msgs::msg::Pose::SharedPtr msg);
        //void sendCancelGoal(const rclcpp_action::GoalUUID& goal_uuid);

        void selectMap(std::string map_name);

        void sendWaypoints(std::shared_ptr<std::unordered_map<std::string, geometry_msgs::msg::Pose>> wp_list, std::shared_ptr<std::unordered_map<std::string, geometry_msgs::msg::Pose>> scene_wp_list);
    
        void sendWaypointsGoal(std::vector<std::string> wp_names);
        
        void mapReadyForWaypointsSubscriber();
    
    public slots:
        void displayCurrentPose(const geometry_msgs::msg::Pose2D::SharedPtr msg);
        void changeCurrentMode(const std_msgs::msg::String::SharedPtr msg);
        
        void sendMappingMode();
        void sendNavigationMode();
        void sendRemappingMode();

        void saveMapClicked();
        void openMapClicked();
        void selectMapClicked(std::string map_name);

        void labelEditForSetForward();
        void labelEditForSetRight();
        void labelEditForSetLeft();
        void labelEditForSetStop();

        void onNavigationResult(const std::string& result_status);
        //void onSendGoalId(const rclcpp_action::GoalUUID& goal_uuid);

        void onCmdServiceResponse(bool success);
        void onWpServiceResponse(bool success);

        void onUpdateWpUI(rom_interfaces::msg::ConstructYaml::SharedPtr wplist_ptr);

        // -----------------------for mapping app
        void onUpdateMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
        void onZoomButtonClicked();
        void onWayPointsButtonClicked();
        void onWallButtonClicked();
        void onEraserButtonClicked();
        void onNormalButtonClicked();
        void onServiceModeButtonClicked();
        // -----------------------end for mapping app

        void onGoAllBtnClicked(bool statys);

        void onUpdateLaser(const sensor_msgs::msg::LaserScan::SharedPtr scan);

        void onTransformReceived(const ROMTransform rom_tf);

        void on_waypointBtn_clicked();

        void showEyesWidget();

        // for mapping , assign some position
        void setChargingPoint();
        void setCurrentPointAs();
        void setProductionPoint();
        
    private slots:
        void on_shutdownBtn_clicked();

    private slots:
        void onResponseReceived(int sum);  
        void onResponseDataReceived(std::shared_ptr<rom_interfaces::srv::WhichMaps::Response> response);

        // action goal
        void on_goBtn_clicked();
        
        void on_rthBtn_clicked();


        void resetInactivityTimer();
        void handleInactivityTimeout();

    protected:

        // for dragging
        void mousePressEvent(QMouseEvent *event) override;
        void mouseMoveEvent(QMouseEvent *event) override;
        void mouseReleaseEvent(QMouseEvent *event) override;
        // end for dragging
    
    private:
        std::shared_ptr<Ui::MainWindow> ui = nullptr;
        QTimer *inactivityTimer_;
        RobotEyeEmotionWindow *robotEyesWidgetPtr_ = nullptr;

        QGraphicsEllipseItem *robotItemPtr_ = nullptr;

        // action goal
        QPushButton *btnGoToGoal_;
        QPushButton *btnWaypointGoals_;
        QPushButton *btnReturnToHome_;
        QPushButton *goAllBtnPtr_;

        //rclcpp_action::GoalUUID active_goal_uuid_;
        //bool is_goal_active_ = false;

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
        QPushButton *relocateBtnPtr_;

        QPushButton *grootBtnPtr_;
        QPushButton *generalBtnPtr_;

        QPushButton *setChargingPointBtnPtr_;
        QPushButton *setCurrentPointAsBtnPtr_;
        QPushButton *setProductionPointBtnPtr_;

        QLabel* statusLabelPtr_ = nullptr;

        ServiceClient *service_clientPtr_;
        QThread *rosServiceClientThreadPtr_;

        // for dragging
        bool dragging;
        QPoint dragPosition;
        // end for dragging

        // logging
        QString currentText_;

        // for mapping app--------------------------------
        
        // map
        double map_origin_x_ = 0.000;
        double map_origin_y_ = 0.000;
        double map_resolution_ = 0.000;

        // tf data
        double map_odom_x = 0.0;
        double map_odom_y = 0.0;
        double map_odom_yaw = 0.0;
        double odom_base_footprint_x = 0.0;
        double odom_base_footprint_y = 0.0;
        double odom_base_footprint_yaw = 0.0;

        // 6 modes
        bool zoom_mode_ = false;
        bool waypoints_mode_   = false;
        bool virtual_wall_mode_ = false;
        bool eraser_mode_ = false;
        bool service_mode_ = false;
        bool normal_mode_ = true;

        QPushButton *zoom_btn_ptr_;
        QPushButton *waypoints_btn_ptr_;
        QPushButton *virtual_wall_btn_ptr_;
        QPushButton *eraser_btn_ptr_;
        QPushButton *normal_btn_ptr_;
        QPushButton *service_mode_btn_ptr_;

        // store waypoints
        QList<QGraphicsEllipseItem*> waypoints_;
        QList<QGraphicsLineItem*> waypoints_lines_;
        QList<QGraphicsTextItem*> waypoints_text_;
        QList<double> waypoints_direction_;
        std::unordered_map<std::string, geometry_msgs::msg::Pose> waypoints_map_;
        std::unordered_map<std::string, geometry_msgs::msg::Pose> waypoints_scene_;
        
        bool loop_waypoints_ = false;


        // store virtual wall
        QList<QGraphicsLineItem*> virtual_lines_;
        QList<QGraphicsRectItem*> virtual_lines_points_;

        bool drawing;
        QPoint *obstacleStartPoint_event_pos = nullptr;
        QPoint *obstacleLastPoint_event_pos = nullptr;
        QGraphicsLineItem *obstacleLine_ = nullptr;

        // -----------------------end for mapping app
        ROMTransform rom_tf_;
        double robot_pose_x_ = 0.0;
        double robot_pose_y_ = 0.0;
        double robot_yaw_rad_ = 0.0;

        sensor_msgs::msg::LaserScan::SharedPtr rom_scan_ = nullptr;
};

#endif // MAINWINDOW_H
