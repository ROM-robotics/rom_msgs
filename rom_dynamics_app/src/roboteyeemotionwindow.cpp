#include "roboteyeemotionwindow.h"
#include <QCloseEvent>
#include <QMetaType>
#include <QRandomGenerator>

#define ROM_DEBUG 1

#ifdef ROM_DEBUG
    #include <QDebug>
#endif

#define EMOTION_DIALOG_WIDTH 1675
#define EMOTION_DIALOG_HEIGHT 1048

const double PI = 3.1415;

ROS2Worker::ROS2Worker()
{
    node_ = std::make_shared<rclcpp::Node>("robot_eye_emotion_window_node");
    emotion_subscription_ = node_->create_subscription<std_msgs::msg::String>(
        "emotion", 10, std::bind(&ROS2Worker::emotionCallback, this, std::placeholders::_1));
}

void ROS2Worker::spin()
{
    rclcpp::spin(node_);
}

void ROS2Worker::emotionCallback(const std_msgs::msg::String::SharedPtr msg)
{
    emit emotionReceived(QString::fromStdString(msg->data));
}

RobotEyeEmotionWindow::RobotEyeEmotionWindow(QWidget *parent)
    : QWidget(parent), current_emotion_("neutral"),
      left_pupil_offset_x_(0), left_pupil_offset_y_(0),
      right_pupil_offset_x_(0), right_pupil_offset_y_(0),
      pupil_direction_(0), animation_speed_(8), is_blinking_(false), should_loop_(true)
{
    qRegisterMetaType<QString>("QString");

    this->setFixedSize(EMOTION_DIALOG_WIDTH, EMOTION_DIALOG_HEIGHT); 

    // Set the background color 
    this->setAutoFillBackground(true);
    
    // Remove window buttons (minimize, maximize, close)
    setWindowFlags(Qt::Window | Qt::FramelessWindowHint);  // Removes the title bar and buttons

    #ifdef ROM_DEBUG
            qDebug() << "[ RobotEyeEmotionWindow::RobotEyeEmotionWindow ] : Constructing ..";
    #endif  

    // Initialize the emotion window UI here
    this->setStyleSheet("background-color: black;");

    // ROS 2
    ros2_worker_ = new ROS2Worker();
    ros2_thread_ = new QThread(this);

    ros2_worker_->moveToThread(ros2_thread_);

    connect(ros2_worker_, &ROS2Worker::emotionReceived, this, &RobotEyeEmotionWindow::onEmotionReceived);
    connect(ros2_thread_, &QThread::started, ros2_worker_, &ROS2Worker::spin);

    ros2_thread_->start();

    // Animation timer
    animation_timer_ = new QTimer(this);
    connect(animation_timer_, &QTimer::timeout, this, &RobotEyeEmotionWindow::updatePupilPosition);
    animation_timer_->start(100);  // Update every 100 ms

    // Blink timer
    blink_timer_ = new QTimer(this);
    connect(blink_timer_, &QTimer::timeout, this, &RobotEyeEmotionWindow::handleBlinkTimeout);
    blink_timer_->start(4000);  // Blink every 4 seconds
}

RobotEyeEmotionWindow::~RobotEyeEmotionWindow()
{
    ros2_thread_->quit();
    ros2_thread_->wait();
    delete ros2_worker_;
}

void RobotEyeEmotionWindow::mousePressEvent(QMouseEvent *event)
{
    // Ignore the mouse press event to prevent the window from hiding when clicked
    event->ignore();
    this->hide();
}

void RobotEyeEmotionWindow::onEmotionReceived(const QString &emotion)
{
    #ifdef ROM_DEBUG
        qDebug() << "[ RobotEyeEmotionWindow::onEmotionReceived ] Received emotion: " << emotion;
    #endif
    
    if ( emotion == "left")
    {
        should_loop_ = false;
        animation_timer_->stop();
        //assignZeros();
        pupil_direction_ = 1;
        animation_timer_->start(100);
    }
    else if ( emotion == "right")
    {
        should_loop_ = false;
        animation_timer_->stop();
        //assignZeros();
        pupil_direction_ = 2;
        animation_timer_->start(100);
    }
    else if ( emotion == "left_right")
    {
        should_loop_ = false;
        animation_timer_->stop();
        assignZeros();
        pupil_direction_ = 3;
        animation_timer_->start(100);
    }
    else if ( emotion == "top")
    {
        should_loop_ = false;
        animation_timer_->stop();
        assignZeros();
        pupil_direction_ = 10;
        animation_timer_->start(100);
    }
    else if ( emotion == "down")
    {
        should_loop_ = false;
        animation_timer_->stop();
        assignZeros();
        pupil_direction_ = 11;
        animation_timer_->start(100);
    }
    else if ( emotion == "top_down")
    {
        should_loop_ = false;
        animation_timer_->stop();
        assignZeros();
        pupil_direction_ = 12;
        animation_timer_->start(100);
    }
    else {
        should_loop_ = true;
        pupil_direction_ = 0;
        current_emotion_ = emotion;
        animation_timer_->start(100);
    }

    update();  // Trigger a repaint
}

void RobotEyeEmotionWindow::updatePupilPosition()
{
    static int delay_counts = 0;
    // Update the pupil offsets based on the current direction
    switch (pupil_direction_) 
    {
        /* 0 Default */
        case 0:  // Loop
            pupil_direction_ = 3;
            break;
        //--------------------------------------------------------------------
        /* 1 Left */
        case 1:  // Center to left
            left_pupil_offset_x_ -= animation_speed_ * 4;
            right_pupil_offset_x_ -= animation_speed_ * 4;
            if (left_pupil_offset_x_ <= -70) 
            {
                pupil_direction_ = 99;
            }
            break;
        //--------------------------------------------------------------------
        /* 2 Right */
        case 2:  // Center to right
            left_pupil_offset_x_ += animation_speed_ * 4;
            right_pupil_offset_x_ += animation_speed_ * 4;
            if (left_pupil_offset_x_ >= 70) 
            {
                pupil_direction_ = 99;
            }
            break;
        //--------------------------------------------------------------------
        /* 3 Left Right sequence */
        case 3:  // Center to left
            left_pupil_offset_x_ -= animation_speed_;
            right_pupil_offset_x_ -= animation_speed_;
            if (left_pupil_offset_x_ <= -70) 
            {
                pupil_direction_ = 4;
            }
            break;
        case 4:  // Delay
            static int delay_counts_1 = 0;
            if (delay_counts_1 >= 20)
            {
                delay_counts_1 = 0;
                pupil_direction_ = 5;
            }
            delay_counts_1 += 1;
            break; 
        case 5:  // Left to center
            left_pupil_offset_x_ += animation_speed_ * 4;
            right_pupil_offset_x_ += animation_speed_ * 4;
            if (left_pupil_offset_x_ >= 0) 
            {
                pupil_direction_ = 6;
            }
            break;
        case 6:  // Center to right
            left_pupil_offset_x_ += animation_speed_;
            right_pupil_offset_x_ += animation_speed_;
            if (left_pupil_offset_x_ >= 70) 
            {
                pupil_direction_ = 7;
            }
            break;
        case 7:  // Delay
            static int delay_counts_2 = 0;
            if (delay_counts_2 >= 20)
            {
                delay_counts_2 = 0;
                pupil_direction_ = 99;
            }
            delay_counts_2 += 1;
            break; 
        //--------------------------------------------------------------------

        /* 10 Top sequence */
        case 10:  // Center to top
            left_pupil_offset_y_ -= animation_speed_;
            right_pupil_offset_y_ -= animation_speed_;
            if (left_pupil_offset_y_ <= -70) 
            {
                pupil_direction_ = 99;
            }
            break;
        //--------------------------------------------------------------------
        /* 12 Down sequence */
        case 11:  // Center to down
            left_pupil_offset_y_ += animation_speed_;
            right_pupil_offset_y_ += animation_speed_;
            if (left_pupil_offset_y_ >= 70) 
            {
                pupil_direction_ = 99;
            }
            break;
        //--------------------------------------------------------------------

        /* 13 Top Down sequence */
        case 14:  // Center to top
            left_pupil_offset_y_ -= animation_speed_ * 4 ;
            right_pupil_offset_y_ -= animation_speed_ * 4;
            if (left_pupil_offset_y_ <= -70) 
            {
                pupil_direction_ = 15;
            }
            break;
        case 15:  // top to center
            left_pupil_offset_y_ += animation_speed_ * 4;
            right_pupil_offset_y_ += animation_speed_ * 4;
            if (left_pupil_offset_y_ >= 0) 
            {
                pupil_direction_ = 16;
            }
            break;
        case 16:  // Center to down
            left_pupil_offset_y_ += animation_speed_ * 4;
            right_pupil_offset_y_ += animation_speed_ * 4;
            if (left_pupil_offset_y_ >= 70) 
            {
                pupil_direction_ = 17;
            }
            break;
        case 17:  // down to center
            left_pupil_offset_y_ -= animation_speed_ * 4;
            right_pupil_offset_y_ -= animation_speed_ * 4;
            if (left_pupil_offset_y_ <= 0) 
            {
                pupil_direction_ = 99;
            }
            break;
        //--------------------------------------------------------------------
        case 99:  // to center
            delay_counts += 1;
            if (delay_counts >= 20)
            {
                delay_counts = 0;
                pupil_direction_ = 0;
                assignZeros();
                animation_timer_->stop();
                if(should_loop_) { animation_timer_->start(100); }
            }
            break; 
        default:
            //animation_timer_->start(100);
            //pupil_direction_ = 0;
            break;
                   
    }

    update();  // Trigger a repaint
}

void RobotEyeEmotionWindow::handleBlinkTimeout()
{
    is_blinking_ = true;
    update();  // Trigger a repaint

    QTimer::singleShot(200, [this]() {
        is_blinking_ = false;
        update();  // Trigger a repaint
    });
}

void RobotEyeEmotionWindow::assignZeros()
{
    left_pupil_offset_x_ = 0;
    left_pupil_offset_y_ = 0;
    right_pupil_offset_x_ = 0;
    right_pupil_offset_y_ = 0;
}


void RobotEyeEmotionWindow::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);
    QPainter painter(this);

    if (is_blinking_) {
        drawBlinkingEyes(painter);
    } 
    else 
    {
        if (current_emotion_ == "happy") {
            drawHappyEyes(painter);
        } else if (current_emotion_ == "angry") {
            drawAngryEyes(painter);
        } else {
            drawNeutralEyes(painter);
        }
    }
}

void RobotEyeEmotionWindow::drawAngryEyes(QPainter &painter)
{
    int widgetWidth = width();
    int widgetHeight = height();

    int eyeWidth = 200;
    int eyeHeight = 200;
    int pupilWidth = 50;
    int pupilHeight = 50;

    int leftEyeX = widgetWidth / 2 - 300;
    int rightEyeX = widgetWidth / 2 + 100;
    int eyeY = widgetHeight / 2 - 100;

    int mouthX = widgetWidth / 2 - 250;
    int mouthY = widgetHeight / 2 + 100;
    int mouthWidth = 500;
    int mouthHeight = 200;

    painter.setBrush(Qt::white);
    painter.drawEllipse(leftEyeX, eyeY, eyeWidth, eyeHeight);  // Left eye
    painter.drawEllipse(rightEyeX, eyeY, eyeWidth, eyeHeight);  // Right eye

    QColor arcColor(0, 0, 0);  
    painter.setBrush(arcColor); 
    painter.drawArc(mouthX, mouthY, mouthWidth, mouthHeight, 0, 180 * 16);

    QPen mouthPen(Qt::white);
    mouthPen.setWidth(3);  // Optional: Set outline width
    painter.setPen(mouthPen);
    painter.drawArc(mouthX, mouthY, mouthWidth, mouthHeight, 0, 180 * 16);  

    painter.setBrush(Qt::black);
    painter.drawEllipse(leftEyeX + 75 + left_pupil_offset_x_, eyeY + 75 + left_pupil_offset_y_, pupilWidth, pupilHeight);  // Left pupil
    painter.drawEllipse(rightEyeX + 75 + right_pupil_offset_x_, eyeY + 75 + right_pupil_offset_y_, pupilWidth, pupilHeight);  // Right pupil
}

void RobotEyeEmotionWindow::drawHappyEyes(QPainter &painter)
{
    int widgetWidth = width();
    int widgetHeight = height();

    int eyeWidth = 200;
    int eyeHeight = 200;
    int pupilWidth = 50;
    int pupilHeight = 50;

    int leftEyeX = widgetWidth / 2 - 300;
    int rightEyeX = widgetWidth / 2 + 100;
    int eyeY = widgetHeight / 2 - 100;

    int mouthX = widgetWidth / 2 - 250;
    int mouthY = widgetHeight / 2 + 100;
    int mouthWidth = 500;
    int mouthHeight = 200;

    painter.setBrush(Qt::white);
    painter.drawEllipse(leftEyeX, eyeY, eyeWidth, eyeHeight);  // Left eye
    painter.drawEllipse(rightEyeX, eyeY, eyeWidth, eyeHeight);  // Right eye

    QColor arcColor(0, 0, 0);  
    painter.setBrush(arcColor); 
    painter.drawArc(mouthX, mouthY, mouthWidth, mouthHeight, 0, -180 * 16); 

    QPen mouthPen(Qt::white);
    mouthPen.setWidth(3);  // Optional: Set outline width
    painter.setPen(mouthPen);
    painter.drawArc(mouthX, mouthY, mouthWidth, mouthHeight, 0, -180 * 16); 

    painter.setBrush(Qt::black);
    painter.drawEllipse(leftEyeX + 75 + left_pupil_offset_x_, eyeY + 75 + left_pupil_offset_y_, pupilWidth, pupilHeight);  // Left pupil
    painter.drawEllipse(rightEyeX + 75 + right_pupil_offset_x_, eyeY + 75 + right_pupil_offset_y_, pupilWidth, pupilHeight);  // Right pupil
}

void RobotEyeEmotionWindow::drawNeutralEyes(QPainter &painter)
{
    int widgetWidth = width();
    int widgetHeight = height();

    int eyeWidth = 200;
    int eyeHeight = 200;
    int pupilWidth = 50;
    int pupilHeight = 50;

    int leftEyeX = widgetWidth / 2 - 300;
    int rightEyeX = widgetWidth / 2 + 100;
    int eyeY = widgetHeight / 2 - 100;

    int mouthX = widgetWidth / 2 - 250;
    int mouthY = widgetHeight / 2 + 300;
    int mouthWidth = 500;

    painter.setBrush(Qt::white);
    painter.drawEllipse(leftEyeX, eyeY, eyeWidth, eyeHeight);  // Left eye
    painter.drawEllipse(rightEyeX, eyeY, eyeWidth, eyeHeight);  // Right eye

    QColor arcColor(0, 0, 0);  
    painter.setBrush(arcColor); 
    painter.drawLine(mouthX, mouthY, mouthX + mouthWidth, mouthY); 

    QPen mouthPen(Qt::white);
    mouthPen.setWidth(3);  // Optional: Set outline width
    painter.setPen(mouthPen);
    painter.drawLine(mouthX, mouthY, mouthX + mouthWidth, mouthY); 

    painter.setBrush(Qt::black);
    painter.drawEllipse(leftEyeX + 75 + left_pupil_offset_x_, eyeY + 75 + left_pupil_offset_y_, pupilWidth, pupilHeight);  // Left pupil
    painter.drawEllipse(rightEyeX + 75 + right_pupil_offset_x_, eyeY + 75 + right_pupil_offset_y_, pupilWidth, pupilHeight);  // Right pupil
}

void RobotEyeEmotionWindow::drawBlinkingEyes(QPainter &painter)
{
    int widgetWidth = width();
    int widgetHeight = height();

    int eyeWidth = 200;
    int eyeHeight = 50;  // Reduced height to simulate blinking

    int leftEyeX = widgetWidth / 2 - 300;
    int rightEyeX = widgetWidth / 2 + 100;
    int eyeY = widgetHeight / 2 - 100;

    painter.setBrush(Qt::white);
    painter.drawEllipse(leftEyeX, eyeY, eyeWidth, eyeHeight);  // Left eye
    painter.drawEllipse(rightEyeX, eyeY, eyeWidth, eyeHeight);  // Right eye
}