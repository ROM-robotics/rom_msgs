#ifndef ROBOTEYEEMOTIONWINDOW_H
#define ROBOTEYEEMOTIONWINDOW_H

#include <QWidget>
#include <QThread>
#include <QPainter>
#include <QMouseEvent>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class ROS2Worker : public QObject
{
    Q_OBJECT

public:
    ROS2Worker();

    void spin();

signals:
    void emotionReceived(const QString &emotion);

private:
    void emotionCallback(const std_msgs::msg::String::SharedPtr msg);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr emotion_subscription_;
};


class RobotEyeEmotionWindow : public QWidget
{
    Q_OBJECT

public:
    RobotEyeEmotionWindow(QWidget *parent = nullptr);
    ~RobotEyeEmotionWindow();

protected:
    void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override; 
    void assignZeros();
    
private slots:
    void onEmotionReceived(const QString &emotion);
    void updatePupilPosition();
    void handleBlinkTimeout();

private:
    void drawNeutralEyes(QPainter &painter);
    //void drawSadEyes(QPainter &painter);
    void drawHappyEyes(QPainter &painter);
    void drawAngryEyes(QPainter &painter);
    void drawBlinkingEyes(QPainter &painter);

    QString current_emotion_;
    ROS2Worker *ros2_worker_;
    QThread *ros2_thread_;
    QTimer *animation_timer_;
    QTimer *delay_timer_;
    QTimer *blink_timer_;  

    int left_pupil_offset_x_;
    int left_pupil_offset_y_;
    int right_pupil_offset_x_;
    int right_pupil_offset_y_;
    int pupil_direction_;
    int animation_speed_;
    bool is_blinking_;
    bool should_loop_; 
};

#endif // ROBOTEYEEMOTIONWINDOW_H
