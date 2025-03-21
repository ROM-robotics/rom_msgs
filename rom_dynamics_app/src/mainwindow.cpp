#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <cmath> // For atan2 and M_PI
#include "rom_algorithm.h"

#define ROM_SHOW_TF 1
#define ROM_SHOW_MAP_TF 1

#define ROM_SHOW_BASEFOOTPRINT_TF 1
#define ROM_SHOW_ODOM_TF 1

#define WAYPOINT_DIALOG_WIDTH 400
#define WAYPOINT_DIALOG_HEIGHT 550

extern void shutdown_thread();

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), dragging(false)
{
    setWindowTitle("ROM Dynamics Company's Robot Suite");
    ui = std::make_shared<Ui::MainWindow>();
    ui->setupUi(this);

    setWindowFlags(Qt::Window | Qt::CustomizeWindowHint);

    /* Darker Galaxy */
    ui->centralwidget->setStyleSheet(
    "QWidget {"
    "   background: qradialgradient("
    "       cx: 0.0, cy: 0.0, radius: 0.8, "
    "       fx: 0.7, fy: 0.3, "
    "       stop: 0 rgba(200, 200, 255, 0.5), "   // Dim white center
    "       stop: 0.2 rgba(100, 149, 237, 0.4), " // Faded blue
    "       stop: 0.4 rgba(72, 61, 139, 0.5), "   // Dark purple tint
    "       stop: 0.6 rgba(128, 0, 128, 0.6), "   // Dark magenta
    "       stop: 0.8 rgba(20, 0, 20, 0.6), "    // Dark magenta
    "       stop: 1 rgba(0, 0, 0, 0.7) "           // Deep black edges
    "   );"
    "}"
    );
    setAttribute(Qt::WA_TranslucentBackground, true);

    statusLabelPtr_ = ui->statusLabel;
    statusLabelPtr_->setWordWrap(true); // Enable word wrapping
    statusLabelPtr_->setAlignment(Qt::AlignLeft | Qt::AlignTop);

    sendMappingBtnPtr_ = ui->mappingBtn;
    sendNavigationBtnPtr_ = ui->navigationBtn;
    sendRemappingBtnPtr_ = ui->remappingBtn;

    saveMapBtnPtr_ = ui->saveMapBtn;
    openMapBtnPtr_ = ui->openMapBtn;
    relocateBtnPtr_ = ui->relocateBtn;
    //relocateBtnPtr_->setText("Relocate");
    setChargingPointBtnPtr_ = ui->setChargingPointBtn;
    setCurrentPointAsBtnPtr_ = ui->setCurrentPointAsBtn;
    setProductionPointBtnPtr_ = ui->setProductionPointBtn;

    x_spinBoxPtr_ = ui->xspinBox;
    y_spinBoxPtr_ = ui->yspinBox;
    z_spinBoxPtr_ = ui->zspinBox;
    x_spinBoxPtr_->setMinimum(-100); 
    x_spinBoxPtr_->setMaximum(100);  
    x_spinBoxPtr_->setValue(0);      
    y_spinBoxPtr_->setMinimum(-100); 
    y_spinBoxPtr_->setMaximum(100);  
    y_spinBoxPtr_->setValue(0);      
    z_spinBoxPtr_->setMinimum(-180); 
    z_spinBoxPtr_->setMaximum(180);  
    z_spinBoxPtr_->setValue(0);

    btnGoToGoal_ = ui->goBtn;
    btnWaypointGoals_ = ui->cancelBtn;
    btnReturnToHome_ = ui->rthBtn;
    goAllBtnPtr_ = ui->goAllBtn;

    // eye emotion -----------------------------------------------------------------------------------------
    robotEyesWidgetPtr_ = new RobotEyeEmotionWindow();
    
    //connect(robotEyeWindow, &QWidget::close, this, &MainWindow::on_robot_eye_window_closed);

    grootBtnPtr_ = ui->grootBtn;

    // mapping ---------------------------------------------------------------------------------------------
    
    zoom_btn_ptr_ = ui->zoomBtn;
    waypoints_btn_ptr_ = ui->addWaypointBtn;
    virtual_wall_btn_ptr_ = ui->addWallBtn;
    eraser_btn_ptr_ = ui->eraserBtn;
    normal_btn_ptr_ = ui->normalBtn;
    service_mode_btn_ptr_ = ui->serviceBtn;
    generalBtnPtr_ = ui->generalBtn;

    connect(generalBtnPtr_, &QPushButton::clicked, this, &MainWindow::showEyesWidget);
    connect(zoom_btn_ptr_, &QPushButton::clicked, this, &MainWindow::onZoomButtonClicked);
    connect(waypoints_btn_ptr_, &QPushButton::clicked, this, &MainWindow::onWayPointsButtonClicked);
    connect(virtual_wall_btn_ptr_, &QPushButton::clicked, this, &MainWindow::onWallButtonClicked);
    connect(eraser_btn_ptr_, &QPushButton::clicked, this, &MainWindow::onEraserButtonClicked);
    connect(normal_btn_ptr_, &QPushButton::clicked, this, &MainWindow::onNormalButtonClicked);
    connect(service_mode_btn_ptr_, &QPushButton::clicked, this, &MainWindow::onServiceModeButtonClicked);

    // mapping end ------------------------------------------------------------------------------------------`    

    // Initialize the ROS Worker
    service_clientPtr_ = new ServiceClient();
    rosServiceClientThreadPtr_ = new QThread();
    service_clientPtr_->moveToThread(rosServiceClientThreadPtr_);

    connect(sendMappingBtnPtr_, &QPushButton::clicked, this, &MainWindow::sendMappingMode);
    connect(sendNavigationBtnPtr_, &QPushButton::clicked, this, &MainWindow::sendNavigationMode);
    connect(sendRemappingBtnPtr_, &QPushButton::clicked, this, &MainWindow::sendRemappingMode);

    connect(saveMapBtnPtr_, &QPushButton::clicked, this, &MainWindow::saveMapClicked);
    connect(openMapBtnPtr_, &QPushButton::clicked, this, &MainWindow::openMapClicked);
    // connect(relocateBtnPtr_, &QPushButton::clicked, this, &MainWindow::selectMapClicked);
    connect(this, &MainWindow::selectMap, this, &MainWindow::selectMapClicked);

    connect(btnGoToGoal_, &QPushButton::clicked, this, &MainWindow::on_goBtn_clicked);
    connect(btnWaypointGoals_, &QPushButton::clicked, this, &MainWindow::on_waypointBtn_clicked);
    connect(btnReturnToHome_, &QPushButton::clicked, this, &MainWindow::on_rthBtn_clicked);
    

    // Open map အတွက် responseReceived, onResponseReceived အသစ်ပြန်ရေးရန်
    connect(service_clientPtr_, &ServiceClient::responseReceived, this, &MainWindow::onResponseReceived);
    connect(service_clientPtr_, &ServiceClient::responseDataReceived, this, &MainWindow::onResponseDataReceived);

    rosServiceClientThreadPtr_->start();

    applyStyles();

    current_mode_ = "navi";

    /* signal ပို့လို့ရတဲ့ meta object ဖြစ်အောင်လို့ ၊ ဒါမရှိရင် error မရှိသော်လည်း subscribe လုပ်မရ */
    qRegisterMetaType<nav_msgs::msg::Odometry::SharedPtr>("nav_msgs::msg::Odometry::SharedPtr");
    qRegisterMetaType<std_msgs::msg::String::SharedPtr>("std_msgs::msg::String::SharedPtr");
    qRegisterMetaType<geometry_msgs::msg::Pose>("geometry_msgs::msg::Pose");
    // qRegisterMetaType<rom_interfaces::srv::WhichMaps::Response::SharedPtr>("rom_interfaces::srv::WhichMaps::Response::SharedPtr");
    qRegisterMetaType<std::shared_ptr<rom_interfaces::srv::WhichMaps::Response>>("std::shared_ptr<rom_interfaces::srv::WhichMaps::Response>");
    // map
    qRegisterMetaType<nav_msgs::msg::OccupancyGrid::SharedPtr>("nav_msgs::msg::OccupancyGrid::SharedPtr");
    // waypoints list
    qRegisterMetaType<std::shared_ptr<std::unordered_map<std::string, geometry_msgs::msg::Pose>>>("std::shared_ptr<std::unordered_map<std::string, geometry_msgs::msg::Pose>>");
    qRegisterMetaType<std::vector<std::string>>("std::vector<std::string>");
    qRegisterMetaType<rom_interfaces::msg::ConstructYaml::SharedPtr>("rom_interfaces::msg::ConstructYaml::SharedPtr");
    qRegisterMetaType<geometry_msgs::msg::Pose2D::SharedPtr>("geometry_msgs::msg::Pose2D::SharedPtr");
    // laser scan
    qRegisterMetaType<sensor_msgs::msg::LaserScan::SharedPtr>("sensor_msgs::msg::LaserScan::SharedPtr");
    // tf
    qRegisterMetaType<geometry_msgs::msg::TransformStamped::SharedPtr>("geometry_msgs::msg::TransformStamped::SharedPtr");
    qRegisterMetaType<ROMTransform>("ROMTransform");
    
    statusLabelPtr_->setText("App အား အသုံးပြုဖို့အတွက် အောက်ပါ ROS2 humble package နှစ်ခုကို install လုပ်ပါ။။\n      - rom_interfaces\n      - which_maps\n\n $ ros2 run which_maps which_maps_server\n # map save ရန် lifecycle လို/မလို စစ်ဆေးပါ။\n");

    ui->saveMapBtn->hide();//setEnabled(false);
    ui->openMapBtn->show();//setEnabled(true);
    ui->relocateBtn->show();//setEnabled(true);
    ui->grootBtn->show();
    ui->setChargingPointBtn->hide();
    ui->setCurrentPointAsBtn->show();
    ui->setProductionPointBtn->show();

    // for mapping mode and assign some position
    connect(setChargingPointBtnPtr_, &QPushButton::clicked, this, &MainWindow::setChargingPoint);
    connect(setCurrentPointAsBtnPtr_, &QPushButton::clicked, this, &MainWindow::setCurrentPointAs);
    connect(setProductionPointBtnPtr_, &QPushButton::clicked, this, &MainWindow::setProductionPoint);

    // Initialize the inactivity timer
    inactivityTimer_ = new QTimer(this);
    inactivityTimer_->setInterval(60000);  // 1 min
    connect(inactivityTimer_, &QTimer::timeout, this, &MainWindow::handleInactivityTimeout);
    inactivityTimer_->start();
    
    ui->normalBtn->click();
}


MainWindow::~MainWindow()
{
    rclcpp::shutdown();

    rosServiceClientThreadPtr_->quit();
    rosServiceClientThreadPtr_->wait();

    delete service_clientPtr_;
    delete rosServiceClientThreadPtr_;

    removeBusyDialog(); 

    //delete ui; // no need shared_pointer automaticall handle lifecycle of ui
    //delete robotEyeWindow;
}


void MainWindow::displayCurrentPose(const geometry_msgs::msg::Pose2D::SharedPtr msg) 
{
    // ဒီကောင့်ကို battery status ပြောင်းရန်
    #ifdef ROM_DEBUG
        qDebug() << "[ MainWindow::displayCurrentPose ]: get robot Pose";
    #endif

    // robot pose ကို label ဖြင့်ပြရန်
    // tf showBaseFootprint() မှာပြတယ်။
    /* Display */
    // ui->xValueLabel->setText(QString("%1").arg(x_feet, 0, 'f', 1));
    // ui->yValueLabel->setText(QString("%1").arg(y_feet, 0, 'f', 1));
    // ui->phiValueLabel->setText(QString("%1").arg(theta_degree, 0, 'f', 1));

    // robot pose ကို scene ထဲမှာပြရန်
    // tf showBaseFootprint() မှာပြတယ်။
    
}

void MainWindow::changeCurrentMode(const std_msgs::msg::String::SharedPtr msg)
{
    //std::string msg_data = msg->data;
    QString msg_data = QString::fromStdString(msg->data);
    statusLabelPtr_->setText(tr("\n... Received current mode from Robot.\n       String: \"%1\"\n").arg(msg_data));

    if (msg_data == "navi")
    {
        //sendNavigationBtnPtr_->click();
        sendNavigationMode();
    } 
    else if (msg_data == "mapping")
    {
        //sendMappingBtnPtr_->click();
        sendMappingMode();
    } 
    else if (msg_data == "remapping")
    {
        //sendRemappingBtnPtr_->click();
        sendRemappingMode();
    }
}

void MainWindow::sendMappingMode() {

    if( current_mode_ == "mapping" ) 
    {
        return;
    } else 
    {
        current_mode_ = "mapping";

        //int a = 99; //inputA->text().toInt();
        //int b = 1;  //inputB->text().toInt();
        const std::string a = current_mode_;
        const std::string b = "";
        QMetaObject::invokeMethod(service_clientPtr_, [a, b, this]() { service_clientPtr_->sendRequest(a, b); });

        statusLabelPtr_->setText("Changing Mapping Mode...\nSending \"mapping\" mode...\n");
        //ui->mappingBtn->setStyleSheet("background-color: green;");
        sendMappingBtnPtr_->setStyleSheet(
            "QPushButton {"
            "   background: qlineargradient(x1: 0, y1: 0, x2: 1, y2: 1, "
            "                                stop: 0 #1e3c72, "  
            "                                stop: 1 #2a5298); "    
            "  color: white;"          
            "}"
            "QPushButton:hover {"
            "   background: qlineargradient(x1: 0, y1: 0, x2: 1, y2: 1, "
            "                                stop: 0 #1E90FF, "
            "                                stop: 1 #87CEEB);"
            "  color: white;"
            "}" 
        );
        sendNavigationBtnPtr_->setStyleSheet(
            "QPushButton {"
            "   background-color: #ffffff;" 
            "   color: black;"               
            "}"
            "QPushButton:hover {"
            "   background: qlineargradient(x1: 0, y1: 0, x2: 1, y2: 1, "
            "                                stop: 0 #1E90FF, "
            "                                stop: 1 #87CEEB);"
            "  color: white;"
            "}" 
        ); 
        sendRemappingBtnPtr_->setStyleSheet(
            "QPushButton {"
            "   background-color: #ffffff;" 
            "   color: black;"               
            "}"
            "QPushButton:hover {"
            "   background: qlineargradient(x1: 0, y1: 0, x2: 1, y2: 1, "
            "                                stop: 0 #1E90FF, "
            "                                stop: 1 #87CEEB);"
            "  color: white;"
            "}" 
        );

        showBusyDialog();
        setButtonsEnabled(false);

        ui->saveMapBtn->show();
        ui->openMapBtn->hide();
        ui->relocateBtn->hide();
        ui->grootBtn->hide();
        ui->setChargingPointBtn->show();
        ui->setCurrentPointAsBtn->show();
        ui->setProductionPointBtn->show();
        // ui->saveMapBtn->setEnabled(true);
        // ui->openMapBtn->setEnabled(false);
        // ui->relocateBtn->setEnabled(false);
        zoom_btn_ptr_->hide();
        waypoints_btn_ptr_->hide();
        virtual_wall_btn_ptr_->hide();
        eraser_btn_ptr_->hide();    
        normal_btn_ptr_->hide();    
        service_mode_btn_ptr_->hide();
        generalBtnPtr_->hide();
    }
}


void MainWindow::sendNavigationMode() {
    if( current_mode_ == "navi" ) {
        return;
    } else 
    {
        current_mode_ = "navi";

        std::string a = current_mode_;
        std::string b = "";
        QMetaObject::invokeMethod(service_clientPtr_, [a, b, this]() { service_clientPtr_->sendRequest(a, b); });

        statusLabelPtr_->setText("Changing Mapping Mode...\nSending \"navi\" mode...\n");
        
        // ui->navigationBtn->setStyleSheet("background-color: green;");
        sendNavigationBtnPtr_->setStyleSheet(
            "QPushButton {"
            "   background: qlineargradient(x1: 0, y1: 0, x2: 1, y2: 1, "
            "                                stop: 0 #1e3c72, "  
            "                                stop: 1 #2a5298); "
            "  color: white;"              
            "}"
            "QPushButton:hover {"
            "   background: qlineargradient(x1: 0, y1: 0, x2: 1, y2: 1, "
            "                                stop: 0 #1E90FF, "
            "                                stop: 1 #87CEEB);"
            "  color: white;"
            "}" 
        );
        sendMappingBtnPtr_->setStyleSheet(
            "QPushButton {"
            "   background-color: #ffffff;" 
            "   color: black;"               
            "}"
            "QPushButton:hover {"
            "   background: qlineargradient(x1: 0, y1: 0, x2: 1, y2: 1, "
            "                                stop: 0 #1E90FF, "
            "                                stop: 1 #87CEEB);"
            "  color: white;"
            "}" 
        );
        sendRemappingBtnPtr_->setStyleSheet(
            "QPushButton {"
            "   background-color: #ffffff;" 
            "   color: black;"               
            "}"
            "QPushButton:hover {"
            "   background: qlineargradient(x1: 0, y1: 0, x2: 1, y2: 1, "
            "                                stop: 0 #1E90FF, "
            "                                stop: 1 #87CEEB);"
            "  color: white;"
            "}" 
        );

        showBusyDialog();
        setButtonsEnabled(false);

        ui->saveMapBtn->hide();
        ui->openMapBtn->show();
        ui->relocateBtn->show();
        ui->grootBtn->show();
        ui->setCurrentPointAsBtn->show();
        ui->setProductionPointBtn->show();
        ui->setChargingPointBtn->hide();
        // ui->saveMapBtn->setEnabled(false);
        // ui->openMapBtn->setEnabled(true);
        // ui->relocateBtn->setEnabled(true);
        zoom_btn_ptr_->show();
        waypoints_btn_ptr_->show();
        virtual_wall_btn_ptr_->show();
        eraser_btn_ptr_->show();    
        normal_btn_ptr_->show();    
        service_mode_btn_ptr_->show();
        generalBtnPtr_->show();
    }
}


void MainWindow::sendRemappingMode() {
    if(current_mode_ == "remapping") {
        return;
    } else 
    {
        current_mode_ = "remapping";
    
        std::string a = current_mode_;
        std::string b = "";
        QMetaObject::invokeMethod(service_clientPtr_, [a, b, this]() { service_clientPtr_->sendRequest(a, b); });
        
        statusLabelPtr_->setText("Changing Mapping Mode...\nSending \"remapping\" mode...\n");
        sendRemappingBtnPtr_->setStyleSheet(
            "QPushButton {"
            "   background: qlineargradient(x1: 0, y1: 0, x2: 1, y2: 1, "
            "                                stop: 0 #1e3c72, "  
            "                                stop: 1 #2a5298); "  
            "  color: white;"            
            "}"
            "QPushButton:hover {"
            "   background: qlineargradient(x1: 0, y1: 0, x2: 1, y2: 1, "
            "                                stop: 0 #1E90FF, "
            "                                stop: 1 #87CEEB);"
            "  color: white;"
            "}" 
        );
        sendMappingBtnPtr_->setStyleSheet(
            "QPushButton {"
            "   background-color: #ffffff;" 
            "   color: black;"               
            "}"
            "QPushButton:hover {"
            "   background: qlineargradient(x1: 0, y1: 0, x2: 1, y2: 1, "
            "                                stop: 0 #1E90FF, "
            "                                stop: 1 #87CEEB);"
            "  color: white;"
            "}" 
        );
        sendNavigationBtnPtr_->setStyleSheet(
            "QPushButton {"
            "   background-color: #ffffff;" 
            "   color: black;"               
            "}"
            "QPushButton:hover {"
            "   background: qlineargradient(x1: 0, y1: 0, x2: 1, y2: 1, "
            "                                stop: 0 #1E90FF, "
            "                                stop: 1 #87CEEB);"
            "  color: white;"
            "}" 
        ); 

        showBusyDialog();
        setButtonsEnabled(false);

        ui->saveMapBtn->show();
        ui->openMapBtn->show();
        ui->relocateBtn->show();
        // ui->saveMapBtn->setEnabled(true);
        // ui->openMapBtn->setEnabled(true);
        // ui->relocateBtn->setEnabled(true);
        zoom_btn_ptr_->hide();
        waypoints_btn_ptr_->hide();
        virtual_wall_btn_ptr_->hide();
        eraser_btn_ptr_->hide();    
        normal_btn_ptr_->hide();    
        service_mode_btn_ptr_->hide();
        generalBtnPtr_->hide();
    }
}


void MainWindow::on_shutdownBtn_clicked()
{
    statusLabelPtr_->setText("\nShutdown 0 ...\n");

    rclcpp::shutdown(); // Stop the ROS 2 node and executor

    QApplication::quit(); // Exit the application
}


void MainWindow::onResponseReceived(int service_status) {

// # -1 [service not ok], 1 [service ok], 
// # 2 [maps exist], 3 [maps do not exist], 
// # 4 [save map ok], 5 [save map not ok]
// # 6 [select map ok], 7 [select map not ok], 

// # 8 [mapping mode ok], 9 [mapping mode not ok],
// # 10 [navi mode ok], 11 [navi mode not ok]
// # 12 [remapping mode ok], 13 [remapping mode not ok]

    QString currentText = statusLabelPtr_->text();

    if (service_status == -1) {
        statusLabelPtr_->setText(currentText + "\n" + "Error: Service not available or failed.\nReceiving not ok.\n");
    } else {
        //statusLabelPtr_->setText(QString("Result: %1").arg(sum));
        statusLabelPtr_->setText(currentText + "\n" + "Receiving response..\nIts ok.\n");

        if (service_status == 2)
        {
            QString currentText = statusLabelPtr_->text();
            statusLabelPtr_->setText(currentText + "\n" + "Map is exist.\n");
            // trigger to select map.
        }
        else if (service_status == 3)
        {
            QString currentText = statusLabelPtr_->text();
            statusLabelPtr_->setText(currentText + "\n" + "Map doesn't exist.\n");
        }
        else if (service_status == 4)
        {
            //saveMapBtnPtr_->setStyleSheet("background-color: white;"); 
            QString currentText = statusLabelPtr_->text();
            statusLabelPtr_->setText(currentText + "\n" + "Save Map ok.\n");

        }
        else if (service_status == 5)
        {
            QString currentText = statusLabelPtr_->text();
            statusLabelPtr_->setText(currentText + "\n" + "Save Map Not ok.\n");
        }
        else if (service_status == 6)
        {
            QString currentText = statusLabelPtr_->text();
            statusLabelPtr_->setText(currentText + "\n" + "Select map ok.\n");
        }
        else if (service_status == 7)
        {
            QString currentText = statusLabelPtr_->text();
            statusLabelPtr_->setText(currentText + "\n" + "Select map Not ok.\n");
        }
        else if (service_status == 8)
        {
            QString currentText = statusLabelPtr_->text();
            statusLabelPtr_->setText(currentText + "\n" + "Mapping Mode Ready.\n");
        }
        else if (service_status == 9)
        {
            QString currentText = statusLabelPtr_->text();
        }
        else if (service_status == 10)
        {
            QString currentText = statusLabelPtr_->text();
            statusLabelPtr_->setText(currentText + "\n" + "Navi Mode Ready.\n");
        }
        else if (service_status == 11)
        {
            QString currentText = statusLabelPtr_->text();
        }
        else if (service_status == 12)
        {
            QString currentText = statusLabelPtr_->text();
            statusLabelPtr_->setText(currentText + "\n" + "Remapping Mode Ok.\n");
        }
        else if (service_status == 13)
        {
            QString currentText = statusLabelPtr_->text();
        }

    }
    //saveMapBtnPtr_->setStyleSheet("background-color: white;"); 

    //openMapBtnPtr_->setStyleSheet("background-color: none;");
    //relocateBtnPtr_->setStyleSheet("background-color: none;");

    hideBusyDialog();
    setButtonsEnabled(true);
}

void MainWindow::saveMapClicked()
{
    showBusyDialog();
    setButtonsEnabled(false);
    
    bool ok;

    QRegExp regex("^[^_]+$"); // Regular expression: no underscores allowed

    QString mapName = QInputDialog::getText(this, tr("Save Map"),
                                            tr("Enter map name:"), QLineEdit::Normal,
                                            tr("default"), &ok);

    if (ok && !mapName.isEmpty()) {
        // Update the status label and send the save map request
        statusLabelPtr_->setText(tr("\nမြေပုံအား '%1' အမည်ဖြင့်သိမ်းဆည်းခြင်းနေပါသည်။ ... \n").arg(mapName));
        
        //saveMapBtnPtr_->setStyleSheet("background: qlineargradient(x1: 0, y1: 0, x2: 1, y2: 1, stop: 0 #1f406e, stop: 1 #132742);");
        
        std::string a = "save_map";
        std::string b = mapName.toStdString();
        QMetaObject::invokeMethod(service_clientPtr_, [a, b, this]() { service_clientPtr_->sendRequest(a, b); });

        statusLabelPtr_->setText(tr("Sending save map request...\nString: \"save_map\"\nmap_name: \"%1\"\n").arg(mapName));
    } 
    else 
    {
        statusLabelPtr_->setText("\nSave map canceled by user.\n");

        hideBusyDialog();
        setButtonsEnabled(true);
    }
    
}


void MainWindow::openMapClicked()
{

        const std::string a = "which_maps_do_you_have";
        const std::string b = "";
        QMetaObject::invokeMethod(service_clientPtr_, [a, b, this]() { service_clientPtr_->sendRequest(a, b); });

        statusLabelPtr_->setText("Inquery Which Maps Do You Have ...\nSending \"request\" ...\n");
        //ui->mappingBtn->setStyleSheet("background-color: green;");
        // ui->mappingBtn->setStyleSheet("background: qlineargradient(x1: 0, y1: 0, x2: 1, y2: 1, stop: 0 #1f406e,stop: 0.8 #87CEEB, stop: 1 #132742);");
        // ui->navigationBtn->setStyleSheet("background-color: white;");
        // ui->remappingBtn->setStyleSheet("background-color: white;");

        showBusyDialog();
        setButtonsEnabled(false);

        // ui->saveMapBtn->show();
        // ui->openMapBtn->hide();
        // ui->relocateBtn->hide();
        // ui->saveMapBtn->setEnabled(true);
        // ui->openMapBtn->setEnabled(false);
        // ui->relocateBtn->setEnabled(false);
    

    
}


void MainWindow::selectMapClicked(std::string map_name)
{
    QString statusText = QString(QString::fromStdString(map_name) + " ရွေးထားပါသည် ။ \n");
    statusLabelPtr_->setText(statusText);
    
    // service client 
    // string request_string # which_map_do_you_have | save_map | select_map | mapping | navi | remapping
    // string map_name_to_save
    // string map_name_to_select
        const std::string a = "select_map";
        std::string name_only = map_name.substr(0, map_name.find_last_of('.'));
        const std::string b = name_only;
        QMetaObject::invokeMethod(service_clientPtr_, [a, b, this]() { service_clientPtr_->sendRequest(a, b); });

        QString updateText = QString(statusText + " sending select map request ... \n");
        statusLabelPtr_->setText(updateText);
     

        showBusyDialog();
        setButtonsEnabled(false);


}


void MainWindow::labelEditForSetForward()
{
    QString statusText = QString("\nConstant Speed with 10Hz.\n\nForward:\n     Linear velocity    : 0.4   m/s\n     Angular velocity : 0.0 rad/s\n");
    statusLabelPtr_->setText(statusText);
    currentText_ = statusText;
    ui->btnForward->setStyleSheet("color: white; background-color: none; border: 4px solid white;");
    ui->btnRight->setStyleSheet("color: #979ba1; background-color: none; border: 2px solid gray;");
    ui->btnLeft->setStyleSheet("color: #979ba1; background-color: none; border: 2px solid gray;");
    ui->btnStop->setStyleSheet("color: #979ba1; background-color: none; border: 2px solid gray;");
}

void MainWindow::labelEditForSetRight()
{
    QString statusText = QString("\nConstant Speed with 10Hz.\n\nRight:\n     Linear velocity    : 0.0   m/s\n     Angular velocity : 0.4   rad/s\n");
    statusLabelPtr_->setText(statusText);
    currentText_ = statusText;
    ui->btnForward->setStyleSheet("color: #979ba1; background-color: none; border: 2px solid gray;");
    ui->btnRight->setStyleSheet("color: white; background-color: none; border: 4px solid white;");
    ui->btnLeft->setStyleSheet("color: #979ba1; background-color: none; border: 2px solid gray;");
    ui->btnStop->setStyleSheet("color: #979ba1; background-color: none; border: 2px solid gray;");
}

void MainWindow::labelEditForSetLeft()
{
    QString statusText = QString("\nConstant Speed with 10Hz.\n\nLeft:\n     Linear velocity    : 0.0   m/s\n     Angular velocity : -0.4 rad/s\n");
    statusLabelPtr_->setText(statusText);
    currentText_ = statusText;
    ui->btnForward->setStyleSheet("color: #979ba1; background-color: none; border: 2px solid gray;");
    ui->btnRight->setStyleSheet("color: #979ba1; background-color: none; border: 2px solid gray;");
    ui->btnLeft->setStyleSheet("color: white; background-color: none; border: 4px solid white;");
    ui->btnStop->setStyleSheet("color: #979ba1; background-color: none; border: 2px solid gray;");
}

void MainWindow::labelEditForSetStop()
{
    QString statusText = QString("\nConstant Speed with 10Hz.\n\nStop:\n     Linear velocity    : 0.0   m/s\n     Angular velocity : 0.0   rad/s\n");
    statusLabelPtr_->setText(statusText);
    currentText_ = statusText;
    ui->btnForward->setStyleSheet("color: #979ba1; background-color: none; border: 2px solid gray;");
    ui->btnRight->setStyleSheet("color: #979ba1; background-color: none; border: 2px solid gray;");
    ui->btnLeft->setStyleSheet("color: #979ba1; background-color: none; border: 2px solid gray;");
    ui->btnStop->setStyleSheet("color: white; background-color: none; border: 4px solid white;");
}


/// for dragging
void MainWindow::mousePressEvent(QMouseEvent *event)
{
    resetInactivityTimer();
    // 1. Default mode
    if(normal_mode_)
    {
        setDragMode(false);

        if (event->button() == Qt::LeftButton) 
        {
            // When the left mouse button is pressed, store the offset relative to the top-left corner
            dragging = true;
            dragPosition = event->globalPos() - frameGeometry().topLeft();
            event->accept();
        } 
    }
    else
    {
        if(!checkGraphicViewAndScene()) {ui->statusLabel->setText("checkGraphicViewAndScene() not ok."); return; }

        // Get the position of the click relative to the QGraphicsView
        QPoint viewPoint = ui->graphicsView->mapFrom(this, event->pos());

        // Map the QGraphicsView coordinates to the QGraphicsScene coordinates
        QPointF scenePoint = ui->graphicsView->mapToScene(viewPoint);

        // Check if the scenePoint is within the scene's bounds
        QRectF sceneBounds = ui->graphicsView->scene()->sceneRect();

        if (!sceneBounds.contains(scenePoint)) 
        {
            // If the point is outside the scene, ignore it
            ui->statusLabel->setText("Click outside scene bounds ignored.");
            return;
        }
        
        // secene ထဲ မှာရှိရင်
        else 
        {   // if the point inside the scene, do your work
            ui->graphicsView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
            ui->graphicsView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);

            // 1. zoom mode
            if(zoom_mode_)
            {
                if (event->button() == Qt::LeftButton) 
                {
                    ui->graphicsView->setDragMode(QGraphicsView::ScrollHandDrag);
                }
                QMainWindow::mousePressEvent(event);

                /* mouse scroll */
                setDragMode(true);

                return;
            }

            // 2. waypoints mode 
            else if(waypoints_mode_)
            {   
                setDragMode(false);

                ui->statusLabel->setText("Click inside scene bounds.");

                // Convert the scene coordinates to map coordinates
                double mapX = (scenePoint.x() * this->map_resolution_) + this->map_origin_x_;
                double mapY = (scenePoint.y() * this->map_resolution_) + this->map_origin_y_;

                /* reverse */
                //double sceneX = (mapX - this->map_origin_x_) / this->map_resolution_;
                //double sceneY = (mapY - this->map_origin_y_) / this->map_resolution_;

                // y-axis qt နဲ့ map မတူခဲ့ရင် ဒါသုံး
                /*
                double mapX = (scenePoint.x() * this->map_resolution_) + this->map_origin_x_;
                double mapY = ((scene_height - scenePoint.y()) * this->map_resolution_) + this->map_origin_y_;
                */
                

                /********************** add points to points list ***************************/
                if (event->button() == Qt::LeftButton) 
                {
                    int radius = 10;
                    QRectF circle(scenePoint.x() - radius, scenePoint.y() - radius, radius * 2, radius * 2);
                
                    QGraphicsEllipseItem* circleItem = new QGraphicsEllipseItem(circle);
                    // Set properties
                    circleItem->setPen(QPen(Qt::green, 2));  
                    circleItem->setBrush(QBrush(Qt::green)); 

                    // create Dialog Box-----------------------------------------------
                    QString wp_name; double wp_heading;

                    CustomDialog dialog;
                    if (dialog.exec() == QDialog::Accepted) {
                        wp_name = dialog.getName();
                        wp_heading = dialog.getNumber() + 90.0; // This will return the value from the QDial
                        
                        // Use wp_name and wp_number in your code
                        for(int i = 0; i < waypoints_text_.size(); i++)
                        {
                            if(waypoints_text_[i]->toPlainText() == wp_name)
                            {
                                QMessageBox::information(this, tr("Cancelled"), tr("Waypoint name already exists."));
                                return;
                            }
                            // and go to end Dialog Box
                        }
                    }
                    else {
                        return;
                    }
                    // end Dialog Box----------------------------------------------------

                    QGraphicsTextItem *textItem = ui->graphicsView->scene()->addText(wp_name);
                        textItem->setFont(QFont("Arial", 16));  // Set font and size
                        textItem->setDefaultTextColor(QColor("#067832"));  // Set text color
                        textItem->setPos(scenePoint.x()-50, scenePoint.y()-65); 

                    waypoints_.append(circleItem);
                    waypoints_text_.append(textItem);
                    waypoints_direction_.append(wp_heading);

                    // add to std::map<string, geometry_msgs::msg::Pose> waypoints_map
                    geometry_msgs::msg::Pose pose;
                    geometry_msgs::msg::Pose scene_pose;

                    pose.orientation.w = 1.0;  // Ensure a valid quaternion
                    scene_pose.orientation.w = 1.0;

                    // convert scene coordinate to map coordinate
                    double x_in_map = mapX;
                    double y_in_map = mapY;
                    double z_in_quaternion = 0.0;
                    double w_in_quaternion = 1.0;
                    // convert degree to radian and euler
                    double yaw_in_map = (wp_heading*0.017453292519943295);
                    yaw_to_quaternion(yaw_in_map, z_in_quaternion, w_in_quaternion);
                    pose.position.x = x_in_map;
                    pose.position.y = y_in_map;
                    pose.orientation.z = z_in_quaternion;
                    pose.orientation.w = w_in_quaternion;

                    scene_pose.position.x = scenePoint.x();
                    scene_pose.position.y = scenePoint.y();
                    scene_pose.orientation.x = wp_heading; // temp store ( degree )
                    scene_pose.orientation.z = z_in_quaternion;
                    scene_pose.orientation.w = w_in_quaternion;

                    // append 
                    waypoints_map_[wp_name.toStdString()] = pose;
                    waypoints_scene_[wp_name.toStdString()] = scene_pose;

                    // Add the rectangle to the scene
                    ui->graphicsView->scene()->addItem(circleItem);
                    
                        // **Draw the Radius Line**
                        QPointF center(scenePoint.x(), scenePoint.y());
                        QPointF edge(scenePoint.x() + radius * cos(yaw_in_map), scenePoint.y() + radius * sin(yaw_in_map));
    
                        QGraphicsLineItem* radiusLine = new QGraphicsLineItem(QLineF(center, edge));
                        radiusLine->setPen(QPen(Qt::red, 2));
                        ui->graphicsView->scene()->addItem(radiusLine);
                        waypoints_lines_.append(radiusLine);
                }

                /********************* remove points to points list ***************************/
                // else if (event->button() == Qt::RightButton) 
                // {
                //     if (!waypoints_.isEmpty()) 
                //     {
                //         QGraphicsEllipseItem *tmpItem = waypoints_.takeLast(); // Get the last item and remove from list
                //         ui->graphicsView->scene()->removeItem(tmpItem);
                //         delete tmpItem;  // Free memory

                //         QGraphicsTextItem *textItem = waypoints_text_.takeLast();
                //         QString text = textItem->toPlainText();
                //         std::string textStd = text.toStdString();
                //         delete textItem; 

                //         QGraphicsLineItem *lineItem = waypoints_lines_.takeLast(); // Get the last item and remove from list
                //         ui->graphicsView->scene()->removeItem(lineItem);
                //         delete lineItem;  // Free memory

                //         waypoints_.removeLast();
                //         waypoints_lines_.removeLast();
                //         waypoints_text_.removeLast();
                //         waypoints_direction_.removeLast();

                //         waypoints_map_.erase(textStd); // should removeLast or not
                //         waypoints_scene_.erase(textStd);
                        
                //         /************* emit data to transmit **************/
                //         // emit waypoints
                //     }
                // } အိုကေပေမဲ့ နောက်ဆုံးနှစ်ခုမှာ မဖျက်တဲ့ ပြသနာရှိတယ်။

                /*********** emit data to transmit **************/
                // point ထည့် / ဖြုတ်ပြီးတိုင်း romWaypoints_ ကို transmit လုပ်ပေးရမယ်။
                // click နှိပ်မှပဲ emit လုပ်တော့မယ်။
                

                // Update the status label with the calculated position
                ui->statusLabel->setText(QString::asprintf("map x: %.5f, y: %.5f, yaw: %d", mapX, mapY, 0));
                return;
            }   

            // 3. Create Virtual Wall mode 
            else if (virtual_wall_mode_) 
            {
                if (event->button() == Qt::LeftButton) 
                {
                    if(!drawing)
                    {
                        drawing = true;
                        obstacleStartPoint_event_pos = new QPoint(scenePoint.x(), scenePoint.y());

                        // ---------------reddot-----------------
                        int radius = 5;
                        QRectF redDot1(scenePoint.x() - radius, scenePoint.y() - radius, radius * 3, radius * 3);
                        QGraphicsRectItem* redDot1Item = new QGraphicsRectItem(redDot1);
                        redDot1Item->setPen(QPen(Qt::red, 2));  
                        redDot1Item->setBrush(QBrush(Qt::red)); 
                        ui->graphicsView->scene()->addItem(redDot1Item);

                        virtual_lines_points_.append(redDot1Item);
                    }
                    else
                    {
                        // ---------------reddot-----------------
                        int radius = 5;
                        QRectF redDot2(scenePoint.x() - radius, scenePoint.y() - radius, radius * 3, radius * 3);
                        QGraphicsRectItem* redDot2Item = new QGraphicsRectItem(redDot2);
                        redDot2Item->setPen(QPen(Qt::red, 2));  
                        redDot2Item->setBrush(QBrush(Qt::red)); 
                        ui->graphicsView->scene()->addItem(redDot2Item);
                    
                        obstacleLastPoint_event_pos = new QPoint(scenePoint.x(), scenePoint.y());
                        QGraphicsLineItem *line = new QGraphicsLineItem( QLine(*obstacleStartPoint_event_pos, *obstacleLastPoint_event_pos) );
                        QPen pen(Qt::red, 10, Qt::DashLine); 
                        line->setPen(pen);
                        ui->graphicsView->scene()->addItem(line);

                        // Store in the QList
                        virtual_lines_.append(line);  
                        virtual_lines_points_.append(redDot2Item);

                        drawing = false;

                        /************* emit data to transmit **************/
                        // virtual_lines_ ကို transmit လုပ်ပေးရမယ်။
                    }
                    #ifdef ROM_DEBUG
                        //qDebug() << "[    MainWindow::mousePressEvent ]: obstacleStartPoint_event_pos : " << obstacleStartPoint_event_pos;
                    #endif
                    return;
                }
                /********************* remove line to lines list ***************************/
                else if (event->button() == Qt::RightButton) 
                {
                    if (!virtual_lines_.isEmpty()) 
                    {
                        QGraphicsLineItem *line = virtual_lines_.takeLast(); // Get the last item and remove from list
                        ui->graphicsView->scene()->removeItem(line);
                        delete line;  // Free memory

                        QGraphicsRectItem *redDot2 = virtual_lines_points_.takeLast();
                        delete redDot2;  

                        QGraphicsRectItem *redDot1 = virtual_lines_points_.takeLast();
                        delete redDot1;  

                        /************* emit data to transmit **************/
                        // virtual_lines_ ကို transmit လုပ်ပေးရမယ်။
                    }
                }
            }

            // 4. Delete landmark mode 
            else if(eraser_mode_)
            {
                setDragMode(false);

                /********************* remove points to points list ***************************/
                if (event->button() == Qt::LeftButton) 
                {
                    for (QGraphicsItem* item : ui->graphicsView->scene()->items()) 
                    {
                        if (dynamic_cast<QGraphicsLineItem*>(item)) 
                        {  // Check if it's a line
                            ui->graphicsView->scene()->removeItem(item);
                            delete item;
                        }
                        else if (dynamic_cast<QGraphicsEllipseItem*>(item)) 
                        {  // Check if it's a Rect Point
                            ui->graphicsView->scene()->removeItem(item);
                            delete item;
                        }
                        else if (dynamic_cast<QGraphicsTextItem*>(item)) 
                        {  // Check if it's a Rect Point
                            ui->graphicsView->scene()->removeItem(item);
                            delete item;
                        }
                        else if (dynamic_cast<QGraphicsRectItem*>(item)) 
                        {       // Check if it's a Rect Point
                            ui->graphicsView->scene()->removeItem(item);
                            delete item;
                        }
                    }
                    // clear lists
                    virtual_lines_.clear();
                    virtual_lines_points_.clear();

                    waypoints_.clear();
                    waypoints_lines_.clear();
                    waypoints_text_.clear();
                    waypoints_direction_.clear();

                    waypoints_map_.clear();
                    waypoints_scene_.clear();
                }

                /************* emit data to transmit **************/
                // romWaypoints_ ကို transmit လုပ်ပေးရမယ်။
                // virtual_lines_ ကို transmit လုပ်ပေးရမယ်။

                return;
            }

        }    // scene ထဲမှာရှိရင် END
       
    }
    
}

void MainWindow::mouseMoveEvent(QMouseEvent *event)
{
    resetInactivityTimer();
    if (dragging && (event->buttons() & Qt::LeftButton)) {
        // Move the window based on the offset calculated in mousePressEvent
        move(event->globalPos() - dragPosition);
        event->accept();
    }
}

void MainWindow::mouseReleaseEvent(QMouseEvent *event)
{
    resetInactivityTimer();
    if (event->button() == Qt::LeftButton) {
        // Stop dragging when the left mouse button is released
        dragging = false;
        event->accept();
    }
}
/// end dragging

void MainWindow::setButtonsEnabled(bool enabled) {
    QList<QPushButton*> buttons = findChildren<QPushButton*>();
    for (QPushButton* button : buttons) {
        button->setEnabled(enabled);
    }
    ui->shutdownBtn->setEnabled(true);
}

void MainWindow::hideBusyDialog() {
    if (busyDialog_->isVisible()) {
        busyDialog_->hide();
    }
}

void MainWindow::removeBusyDialog() {
    if (busyDialog_ != nullptr) {
        delete busyDialog_;  // Free the memory
        busyDialog_ = nullptr;
    }
}

void MainWindow::showBusyDialog() {
    if (busyDialog_ == nullptr) {

        busyDialog_ = new QProgressDialog("Please wait...", QString(), 0, 0, this);
        busyDialog_->setWindowFlags(Qt::FramelessWindowHint);
        busyDialog_->setFixedSize(400, 150); 
        busyDialog_->setWindowModality(Qt::WindowModal);
        busyDialog_->setCancelButton(nullptr); // Optional: Disable cancel button

        // busyDialog_->move((this->width() - busyDialog_->width()) / 2,
        //           (this->height() - busyDialog_->height()) / 2);

        busyDialog_->move(600, 1080); 
        busyDialog_->setStyleSheet(
    "QProgressBar {"
    "   border: 2px solid black;"
    "   border-radius: 5px;"
    "   text-align: center;"
    "   height: 10px;"
    "}"
    "QProgressBar::chunk {"
    "   background: qlineargradient("
    "       x1: 0, y1: 0, x2: 1, y2: 0, "   // Horizontal gradient
    "       stop: 0 rgba(0, 0, 0, 1), "     // Black at the start (comet)
    "       stop: 0.3 rgba(0, 0, 0, 0.8), " // Slight fade for the comet trail
    "       stop: 0.6 rgba(169, 169, 169, 0.6), " // Gray transition
    "       stop: 1 rgba(255, 255, 255, 1)"            // White at the end
    "   );"
    "   border-radius: 5px;"                    // Match the progress bar corners
    "}"
);

        //busyDialog_->hide();
    }

    // Show the dialog
    busyDialog_->show();
}

void MainWindow::on_goBtn_clicked()
{
    if(x_spinBoxPtr_->value() == 0 && y_spinBoxPtr_->value() == 0 && z_spinBoxPtr_->value() == 0)
    {
        statusLabelPtr_->setText("\n   x , y, theta တန်ဖိုးများမရှိပါ။\n");
          
        return;
    }

    double x    = ( x_spinBoxPtr_->value() * foot_to_meter_constant );  // to meters
    double y    = ( y_spinBoxPtr_->value() * foot_to_meter_constant );  // to meters
    double theta= ( z_spinBoxPtr_->value() * degree_to_radian_constant);// to radian
    
    // Display
    QString statusText = QString("Sending Action Goal  .......\n            X       : %1 meters\n            Y       : %2 meters\n    Heading : %3 radians\n")
                        .arg(x)
                        .arg(y)
                        .arg(theta);
    statusLabelPtr_->setText(statusText);
    currentText_ = statusText;

    showBusyDialog();
    QApplication::processEvents();  // Ensure dialog is displayed

    setButtonsEnabled(false);
    
    // btnWaypointGoals_->setEnabled(true);
    // btnReturnToHome_->setEnabled(true);
    
    auto pose = geometry_msgs::msg::Pose::SharedPtr(new geometry_msgs::msg::Pose());
    pose->position.x = x;
    pose->position.y = y;

    yaw_to_quaternion(theta, pose->orientation.z, pose->orientation.w);
    #ifdef ROM_DEBUG 
        qDebug() << "[  MainWindow::on_goBtn_clicked  ] : sending pose";
    #endif

    //emit sendNavigationGoal(pose);
    
    //Emit the navigation goal
    QMetaObject::invokeMethod(this, [this, pose]() {
        emit sendNavigationGoal(pose);
    }, Qt::QueuedConnection);
}


void MainWindow::on_waypointBtn_clicked()
{
    emit sendWaypoints(std::make_shared<std::unordered_map<std::string, geometry_msgs::msg::Pose>>(waypoints_map_), std::make_shared<std::unordered_map<std::string, geometry_msgs::msg::Pose>>(waypoints_scene_));
//void onSendWaypoints(std::shared_ptr<std::unordered_map<std::string, geometry_msgs::msg::Pose>> wp_list);
}


void MainWindow::on_rthBtn_clicked()
{
    double x    = 0.3;
    double y    = 0.0;
    double theta= 0.0;
    
    QString statusText = QString("Sending Action Goal  .......\n            X       : %1 meters\n            Y       : %2 meters\n    Heading : %3 radians\n")
                        .arg(x)
                        .arg(y)
                        .arg(theta);
    statusLabelPtr_->setText(statusText);
    currentText_ = statusText;

    showBusyDialog();
    QApplication::processEvents();  // Ensure dialog is displayed

    setButtonsEnabled(false);
    
    // btnWaypointGoals_->setEnabled(true);
    
    auto pose = geometry_msgs::msg::Pose::SharedPtr(new geometry_msgs::msg::Pose());
    pose->position.x = x;
    pose->position.y = y;

    yaw_to_quaternion(theta, pose->orientation.z, pose->orientation.w);

    
    //emit sendNavigationGoal(pose);

    // Emit the navigation goal signal for different threads
    QMetaObject::invokeMethod(this, [this, pose]() {
        emit sendNavigationGoal(pose);
    }, Qt::QueuedConnection);
}


void MainWindow::onNavigationResult(const std::string& result_status)
{
    QString statusText = QString("Navigation Result : %1\n").arg(QString::fromStdString(result_status));

    QString updateText = currentText_ + statusText;
    statusLabelPtr_->setText(updateText);

    #ifdef ROM_DEBUG 
        qDebug() << "[ MainWindow::onNavigationResult  ] : get Navigation Result " << QString::fromStdString(result_status);
    #endif
    
    setButtonsEnabled(true);
    hideBusyDialog();
}


void MainWindow::toggleButtonWithAnimation(QPushButton* button, bool show) {
    // Create a property animation
    QPropertyAnimation* animation = new QPropertyAnimation(button, "geometry", this);

    QRect startGeometry = button->geometry();
    QRect endGeometry = startGeometry;

    if (show) {
        // Show the button and expand its geometry from zero height
        button->show();
        endGeometry.setHeight(startGeometry.height());
    } else {
        // Animate to zero height for hiding
        endGeometry.setHeight(0);
    }

    animation->setDuration(300); // Duration of the animation in milliseconds
    animation->setStartValue(startGeometry);
    animation->setEndValue(endGeometry);
    animation->setEasingCurve(QEasingCurve::InOutCubic); // Optional: Smooth easing curve

    connect(animation, &QPropertyAnimation::finished, [button, show]() {
        if (!show) {
            button->hide(); // Ensure it is fully hidden after the animation
        }
    });

    // Start the animation
    animation->start(QAbstractAnimation::DeleteWhenStopped);
}

void MainWindow::onCmdServiceResponse(bool success)
{
    if(!success)
    {
        QString statusText = QString("Service not found !\n");
        QString updateText = currentText_ + statusText;
        statusLabelPtr_->setText(updateText);

        #ifdef ROM_DEBUG 
            qDebug() << "[  MainWindow::onCmdServiceResponse  ] : Service not found";
        #endif

        // ui->btnForward->setStyleSheet("color: #979ba1; background-color: none; border: 2px solid gray;");
        // ui->btnRight->setStyleSheet("color: #979ba1; background-color: none; border: 2px solid gray;");
        // ui->btnLeft->setStyleSheet("color: #979ba1; background-color: none; border: 2px solid gray;");
        // ui->btnStop->setStyleSheet("color: #979ba1; background-color: none; border: 2px solid gray;");

    }else 
    {
        #ifdef ROM_DEBUG 
            qDebug() << "[  MainWindow::onCmdServiceResponse  ] : Service OK !";
        #endif
    }
}


void MainWindow::onWpServiceResponse(bool success){
    //this->blockSignals(true);  
    //this->blockSignals(false);  
    if(!success)
    {
        QString statusText = QString("Waypoint Service not found !\n");
        QString updateText = currentText_ + statusText;
        statusLabelPtr_->setText(updateText);

        #ifdef ROM_DEBUG 
            qDebug() << "[  MainWindow::onWpServiceResponse   ] : Service not found";
        #endif

        // ui->btnForward->setStyleSheet("color: #979ba1; background-color: none; border: 2px solid gray;");
        // ui->btnRight->setStyleSheet("color: #979ba1; background-color: none; border: 2px solid gray;");
        // ui->btnLeft->setStyleSheet("color: #979ba1; background-color: none; border: 2px solid gray;");
        // ui->btnStop->setStyleSheet("color: #979ba1; background-color: none; border: 2px solid gray;");

    }else 
    {
        #ifdef ROM_DEBUG 
            qDebug() << "[  MainWindow::onWpServiceResponse   ] : Service OK !";
        #endif
    }
    
}


void MainWindow::onResponseDataReceived(std::shared_ptr<rom_interfaces::srv::WhichMaps::Response> response)
{
    if (!response || response->total_maps <= 0 || response->map_names.empty()) 
    {
        #ifdef ROM_DEBUG
            qDebug() << "[  MainWindow::onResponseDataReceived  ] : Invalid or empty response received!";
        #endif
        return;
    }

    // Create a new dialog for displaying the map names
    QDialog *dialog = new QDialog(this);
    dialog->setWindowTitle("Select a Map");
    dialog->setModal(true);

    // Populate the QListWidget with map names
    QListWidget *listWidget = new QListWidget(dialog);
    QStringList list;
    for (int i = 0; i < response->total_maps; ++i) {
        list.append(QString::fromStdString(response->map_names[i]));
    }
    listWidget->addItems(list);

    // Add OK and Cancel buttons
    QPushButton *okButton = new QPushButton("OK", dialog);
    QPushButton *cancelButton = new QPushButton("Cancel", dialog);

    // Layout setup
    QVBoxLayout *layout = new QVBoxLayout(dialog);
    layout->addWidget(listWidget);
    layout->addWidget(okButton);
    layout->addWidget(cancelButton);
    dialog->setLayout(layout);

    // Handle button actions
    connect(okButton, &QPushButton::clicked, [dialog, listWidget, this]() {
        QListWidgetItem *selectedItem = listWidget->currentItem();
        if (selectedItem) {
            #ifdef ROM_DEBUG 
                qDebug() << "[  MainWindow::onResponseDataReceived  ] : Selected map:" << selectedItem->text();
            #endif
            //QMessageBox::information(this, "Map Selected", "You selected: " + selectedItem->text());

            // to trigger select map
            emit selectMap( selectedItem->text().toStdString() );

        } else {
            QMessageBox::warning(this, "No Selection", "Please select a map.");
        }
        dialog->accept();
    });

    connect(cancelButton, &QPushButton::clicked, dialog, &QDialog::reject);

    // Show the dialog
    dialog->exec();

    // Clean up
    delete dialog;

}


void MainWindow::applyStyles()
{
    ui->companyLabel->setStyleSheet(
    "QLabel {"
    "   color: #dde7ed;"
    "}"
    );
    x_spinBoxPtr_->setStyleSheet(
    "QSpinBox {"
    "   background-color: gray;"
    "   color: white;" 
    "}"
    "QSpinBox::up-button, QSpinBox::down-button {"
    "   background-color: none;"  // Optional: To ensure the buttons are transparent or styled accordingly
    "}"
    );
    y_spinBoxPtr_->setStyleSheet(
    "QSpinBox {"
    "   background-color: black;"
    "   color: white;" 
    "}"
    );
    z_spinBoxPtr_->setStyleSheet(
    "QSpinBox {"
    "   background-color: gray;"
    "   color: white;" 
    "}"
    );
    ui->shutdownBtn->setStyleSheet(
    "QPushButton:pressed {"
    "    background-color: rgb(255, 200, 200);"        // Background color when pressed
    "}"
    );
    ui->btnStop->setToolTip("stop");
    ui->btnStop->setStyleSheet(
    "QPushButton {"
    "   border: 4px solid white;"
    "   background-color: none;"
    "   color: white;"
    "}"
    "QPushButton:pressed {"
    "    background-color: rgb(255, 200, 200);"       
    "}"
    );
    ui->btnForward->setToolTip("forward");
    ui->btnForward->setStyleSheet(
    "QPushButton {"
    "   background-color: none;"
    "   color: #979ba1;"
    "}"
    "QPushButton:pressed {"
    "    background-color: rgb(200, 255, 200);"      
    "}"
    );
    //--------------------------------------------------mapping app
    ui->graphicsView->setBackgroundBrush(Qt::gray);
    
    //--------------------------------------------------------
    ui->btnLeft->setToolTip("left");
    ui->btnLeft->setStyleSheet(
    "QPushButton {"
    "   background-color: none;"
    "   color: #979ba1;"
    "}"
    "QPushButton:pressed {"
    "    background-color: rgb(200, 255, 200);"       
    "}"
    );
    ui->btnRight->setToolTip("right");
    ui->btnRight->setStyleSheet(
    "QPushButton {"
    "   background-color: none;"
    "   color: #979ba1;"
    "}"
    "QPushButton:pressed {"
    "    background-color: rgb(200, 255, 200);"        
    "}"
    );
    // ------------------
    sendMappingBtnPtr_->setStyleSheet(
        "QPushButton {"
        "   background-color: #ffffff;" 
        "   color: black;"               
        "}"
        "QPushButton:hover {"
        "   background: qlineargradient(x1: 0, y1: 0, x2: 1, y2: 1, "
        "                                stop: 0 #1E90FF, "
        "                                stop: 1 #87CEEB);"
        "   color: white;"
        "}" 
    );
    sendNavigationBtnPtr_->setStyleSheet(
        "QPushButton {"
        "   background: qlineargradient(x1: 0, y1: 0, x2: 1, y2: 1, "
        "                                stop: 0 #1e3c72, "  
        "                                stop: 1 #2a5298); "
        "   color: white;"
        "}"
        "QPushButton:hover {"
        "   background: qlineargradient(x1: 0, y1: 0, x2: 1, y2: 1, "
        "                                stop: 0 #1E90FF, "
        "                                stop: 1 #87CEEB);"
        "   color: white;"
        "}" 
    ); 
    sendRemappingBtnPtr_->setStyleSheet(
        "QPushButton {"
        "   background-color: #ffffff;" 
        "   color: black;"               
        "}"
        "QPushButton:hover {"
        "   background: qlineargradient(x1: 0, y1: 0, x2: 1, y2: 1, "
        "                                stop: 0 #1E90FF, "
        "                                stop: 1 #87CEEB);"
        "   color: white;"
        "}" 
    );
    // ------------------
    saveMapBtnPtr_->setStyleSheet(
        "QPushButton {"
        "   background-color: #ffffff;" 
        "   color: black;"               
        "}"
        "QPushButton:hover {"
        "   background: qlineargradient(x1: 0, y1: 0, x2: 1, y2: 1, "
        "                                stop: 0 #004d00, "
        "                                stop: 1 #006400);"
        "   color: white;"
        "}" 
    );
    openMapBtnPtr_->setStyleSheet(
        "QPushButton {"
        "   background-color: #ffffff;" 
        "   color: black;"               
        "}"
        "QPushButton:hover {"
        "   background: qlineargradient(x1: 0, y1: 0, x2: 1, y2: 1, "
        "                                stop: 0 #004d00, "
        "                                stop: 1 #006400);"
        "   color: white;"
        "}" 
    ); 
    relocateBtnPtr_->setStyleSheet(
        "QPushButton {"
        "   background-color: #ffffff;" 
        "   color: black;"               
        "}"
        "QPushButton:hover {"
        "   background: qlineargradient(x1: 0, y1: 0, x2: 1, y2: 1, "
        "                                stop: 0 #004d00, "
        "                                stop: 1 #006400);"
        "   color: white;"
        "}" 
    );
    setChargingPointBtnPtr_->setStyleSheet(
        "QPushButton {"
        "   background-color: #ffffff;" 
        "   color: black;"               
        "}"
        "QPushButton:hover {"
        "   background: qlineargradient(x1: 0, y1: 0, x2: 1, y2: 1, "
        "                                stop: 0 #004d00, "
        "                                stop: 1 #006400);"
        "   color: white;"
        "}" 
    );
    setCurrentPointAsBtnPtr_->setStyleSheet(
        "QPushButton {"
        "   background-color: #ffffff;" 
        "   color: black;"               
        "}"
        "QPushButton:hover {"
        "   background: qlineargradient(x1: 0, y1: 0, x2: 1, y2: 1, "
        "                                stop: 0 #004d00, "
        "                                stop: 1 #006400);"
        "   color: white;"
        "}" 
    );
    setProductionPointBtnPtr_->setStyleSheet(
        "QPushButton {"
        "   background-color: #ffffff;" 
        "   color: black;"               
        "}"
        "QPushButton:hover {"
        "   background: qlineargradient(x1: 0, y1: 0, x2: 1, y2: 1, "
        "                                stop: 0 #004d00, "
        "                                stop: 1 #006400);"
        "   color: white;"
        "}" 
    );
    grootBtnPtr_->setStyleSheet(
        "QPushButton {"
        "   background-color: #ffffff;"  
        "   color: black;"              
        "}"
        "QPushButton:hover {"
        "   background: qlineargradient(x1: 0, y1: 0, x2: 1, y2: 1, "
        "                                stop: 0 #ffeb3b, "  
        "                                stop: 1 #fbc02d); " 
        "   color: white;"
        "}" 
    );
    // ------------------
    ui->shutdownBtn->setStyleSheet(
    "QPushButton {"
    "   border-radius: 25px;"
    "   border: 1px solid gray;"
    "   background-color: white;"
    "   color: red;"
    "   font-size: 36px;"
    "}"
    "QPushButton:hover {"
    "   background: qlineargradient("
    "       x1: 0, y1: 0, x2: 1, y2: 1," // Diagonal gradient
    "       stop: 0 yellow,"             // Bright yellow
    "       stop: 0.4 #ff9a00,"            // Bright yellow-orange
    "       stop: 0.6 #ff4500,"          // Deep orange
    "       stop: 0.9 #d30000,"          // Fiery red
    "       stop: 1 #800000"             // Dark red        
    "   );"
    "   color: white;"
    "}"
    );
    ui->goBtn->setStyleSheet(
    "QPushButton {"
    "   border-radius: 35px;"
    "   border: 2px solid #1B1F4F;" // Dark outer edge like Neptune's shadow
    "   background: qlineargradient("
    "       x1: 0, y1: 0, x2: 1, y2: 1, " // Diagonal gradient
    "       stop: 0 #000080, "            // Deep blue (core Neptune color)
    "       stop: 0.5 #1E90FF, "         // Bright cyan for methane atmosphere
    "       stop: 0.8 #87CEEB, "         // Sky blue highlights
    "       stop: 1 #FFFFFF"             // White for clouds
    "   );"
    "   color: black;"          
    "   font-size: 33px;"
    "}"
    "QPushButton:hover {"
    "   background: qlineargradient("
    "       x1: 0, y1: 0, x2: 1, y2: 1, " // Diagonal gradient
    "       stop: 0 #191970, "            // Darker blue for depth
    "       stop: 0.5 #4682B4, "         // Steel blue for variation
    "       stop: 0.8 #5F9EA0, "         // Cadet blue highlights
    "       stop: 1 #F0FFFF"             // Light cyan-white for soft clouds
    "   );"
    "   color: white;"  
    "}"
    );
    ui->rthBtn->setStyleSheet(
    "QPushButton {"
    "   border-radius: 50px;"
    "   border: 2px solid #3E8E41;" // Earth-tone border
    "   background: qlineargradient("
    "       x1: 0, y1: 0, x2: 1, y2: 1, " // Diagonal gradient
    "       stop: 0 #1E90FF, "            // Ocean Blue (stronger at start)
    "       stop: 0.3 #87CEFA, "         // Sky Blue (transitional)
    "       stop: 0.6 #32CD32, "         // Land Green
    "       stop: 1 #FFFFFF"             // Cloud White
    "   );"
    "   color: black;"
    "   font-size: 33px;"
    "   font-weight: bold;"
    "}"
    "QPushButton:hover {"
    "   background: qlineargradient("
    "       x1: 0, y1: 0, x2: 1, y2: 1, " // Diagonal gradient
    "       stop: 0 #4682B4, "            // Darker Ocean Blue
    "       stop: 0.3 #5F9EA0, "         // Cadet Blue (transitional)
    "       stop: 0.6 #228B22, "         // Darker Land Green
    "       stop: 1 #F0F8FF"             // Lighter Cloud White
    "   );"
    "   color: white;"
    "}"
    );
    btnWaypointGoals_->setStyleSheet(
    "QPushButton {"
    "   border-radius: 70px;"
    "   border: 2px solid #FF8C00;" // Warm orange border
    "   background: qlineargradient("
    "       x1: 0, y1: 0, x2: 1, y2: 1, " // Diagonal gradient
    "       stop: 0 #FFFF00, "            // Bright Yellow (center of the sun)
    "       stop: 0.4 #FFA500, "         // Orange (middle glow)
    "       stop: 1 #FF4500"             // Red-Orange (outer glow)
    "   );"
    "   color: black;"
    "   font-size: 24px;"
    "   font-weight: bold;"
    "}"
    "QPushButton:hover {"
    "   background: qlineargradient("
    "       x1: 0, y1: 0, x2: 1, y2: 1, " // Diagonal gradient
    "       stop: 0 #FFD700, "            // Golden Yellow
    "       stop: 0.4 #FF8C00, "         // Darker Orange
    "       stop: 1 #FF6347"             // Tomato Red (darker outer glow)
    "   );"
    "   color: white;"
    "}"
    );
    goAllBtnPtr_->setStyleSheet(
    "QPushButton {"
    "   border-radius: 35px;"
    "   border: 2px solid #FF8C00;" 
    "   background: qlineargradient("
    "       x1: 0, y1: 0, x2: 1, y2: 1, "
    "       stop: 0.0 #FFD700, "
    "       stop: 0.4 #B8860B,"
    "       stop: 0.7 #8B4513, "
    "       stop: 1.0 #4B3621  "         
    "   );"
    "   color: black;"
    "   font-size: 22px;"
    "   font-weight: bold;"
    "}"
    "QPushButton:hover {"
    "   background: qlineargradient("
    "       x1: 0, y1: 0, x2: 1, y2: 1, "
    "       stop: 0.0 #FFD700, "
    "       stop: 0.4 #DAA520, "
    "       stop: 0.7 #A0522D,"
    "       stop: 1.0 #000000"             
    "   );"
    "   color: white;"
    "}"
    );

    ui->xspinBox->setStyleSheet(
    "QSpinBox {"
    "   border: 2px solid red;" 
    "   border-radius: 10px;"        // Rounding the corners
    "   background-color: #f0f0f0;"
    "   color: black;"
    "}"
    );
    ui->yspinBox->setStyleSheet(
    "QSpinBox {"
    "   border: 2px solid green;" 
    "   border-radius: 10px;"        // Rounding the corners
    "   background-color: #f0f0f0;"
    "   color: black;"
    "}"
    );
    ui->zspinBox->setStyleSheet(
    "QSpinBox {"
    "   border: 2px solid blue;" 
    "   border-radius: 10px;"        // Rounding the corners
    "   background-color: #f0f0f0;"
    "   color: black;"
    "}"
    );
}


void MainWindow::onZoomButtonClicked()
{
    #ifdef ROM_DEBUG
        qDebug() << "[  MainWindow::onZoomButtonClicked ] : zoom mode btn clicked";
    #endif
    ui->statusLabel->setText("Zoom Button Clicked.");
    zoom_mode_ = true;
    waypoints_mode_   = false;
    virtual_wall_mode_ = false;
    eraser_mode_ = false;
    normal_mode_ = false;
    service_mode_ = false;

    applyStyleZoom();
}
void MainWindow::onWayPointsButtonClicked()
{
    #ifdef ROM_DEBUG
        qDebug() << "[  MainWindow::onWayPointsButtonClicked    ] : waypoint mode btn clicked";
    #endif
    ui->statusLabel->setText("Waypoints Button Clicked.");
    zoom_mode_ = false;
    waypoints_mode_   = true;
    virtual_wall_mode_ = false;
    eraser_mode_ = false;
    normal_mode_ = false;
    service_mode_ = false;

    applyStyleWaypoint();
}
void MainWindow::onWallButtonClicked()
{
    #ifdef ROM_DEBUG
        qDebug() << "[  MainWindow::onWallButtonClicked ]: wall mode btn clicked";
    #endif
    ui->statusLabel->setText("Wall Button Clicked.");
    zoom_mode_ = false;
    waypoints_mode_   = false;
    virtual_wall_mode_ = true;
    eraser_mode_ = false;
    normal_mode_ = false;
    service_mode_ = false;

    applyStyleWall();
}
void MainWindow::onEraserButtonClicked()
{
    #ifdef ROM_DEBUG
        qDebug() << "[  MainWindow::onEraserButtonClicked   ]: Eraser mode btn clicked";
    #endif
    ui->statusLabel->setText("Eraser Button Clicked.");
    zoom_mode_ = false;
    waypoints_mode_   = false;
    virtual_wall_mode_ = false;
    eraser_mode_ = true;
    normal_mode_ = false;
    service_mode_ = false;

    applyStyleEraser();
}
void MainWindow::onNormalButtonClicked()
{
    #ifdef ROM_DEBUG
        qDebug() << "[  MainWindow::onNormalButtonClicked   ]: Normal mode btn clicked";
    #endif
    ui->statusLabel->setText("Normal Button Clicked.");
    zoom_mode_ = false;
    waypoints_mode_   = false;
    virtual_wall_mode_ = false;
    eraser_mode_ = false;
    normal_mode_ = true;
    service_mode_ = false;

    applyStyleNormal();
}

void MainWindow::onServiceModeButtonClicked()
{
    #ifdef ROM_DEBUG
        qDebug() << "[  MainWindow::onServiceModeButtonClicked   ]: Service mode btn clicked";
    #endif
    ui->statusLabel->setText("Service mode Clicked.");

    zoom_mode_ = false;
    waypoints_mode_   = false;
    virtual_wall_mode_ = false;
    eraser_mode_ = false;
    service_mode_ = true;
    normal_mode_ = false;

    applyStyleServiceMode();
}

void MainWindow::onUpdateMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) 
{
    #ifdef ROM_DEBUG
        qDebug() << "[  MainWindow::onUpdateMap ]: onUpdateMap slot function";
    #endif

    ui->graphicsView->viewport()->update();

    double map_origin_x   = msg->info.origin.position.x;
    double map_origin_y   = msg->info.origin.position.y;
    double map_resolution = msg->info.resolution;

    // Convert OccupancyGrid to an image
    QImage mapImage(msg->info.width, msg->info.height, QImage::Format_RGB888);
    for (size_t y = 0; y < msg->info.height; ++y) 
    {
        //int inverted_y = msg->info.height - 1 - y;

        for (size_t x = 0; x < msg->info.width; ++x) 
        {
            int index = y * msg->info.width + x;
            int value = msg->data[index];
            QColor color = (value == 0) ? Qt::white : (value == 100) ? Qt::black : Qt::gray;

            //mapImage.setPixel(x, inverted_y, color.rgb());
            mapImage.setPixel(x, y, color.rgb());
        }
    }
    this->map_resolution_ = map_resolution; 
    this->map_origin_x_ = map_origin_x; // catesian coordinate
    this->map_origin_y_ = map_origin_y;

    #ifdef ROM_DEBUG
        qDebug() << "[ MainWindow::onUpdateMap ] : map_resolution = " << map_resolution;
        qDebug() << "[ MainWindow::onUpdateMap ] : map_origin.x = " << map_origin_x;
        qDebug() << "[ MainWindow::onUpdateMap ] : map_origin.y = " << map_origin_y;
    #endif

    QGraphicsScene *scene = new QGraphicsScene(this);
    scene->addPixmap(QPixmap::fromImage(mapImage));
    ui->graphicsView->setScene(scene);
    
    ui->graphicsView->fitInView(scene->sceneRect(), Qt::KeepAspectRatio);
    #ifdef ROM_DEBUG
        qDebug() << "[  MainWindow::onUpdateMap ]: onUpdateMap finished";
    #endif

    showSceneOriginCoordinate();

    #ifdef ROM_SHOW_TF
        #ifdef ROM_SHOW_MAP_TF
            showMapOriginCooridinate();
        #endif
    #endif
    //showOdomAndBaseFootprint();
    emit mapReadyForWaypointsSubscriber();
}

bool MainWindow::checkGraphicViewAndScene()
{
    if (!ui->graphicsView) {
        ui->statusLabel->setText("GraphicsView is null.");
        return false;
    }

    if (!ui->graphicsView->scene()) {
        ui->statusLabel->setText("GraphicsView has no scene."); 
        return false;
    }
    else {
        return true;
    }
}

void MainWindow::wheelEvent(QWheelEvent *event) 
{
    if(zoom_mode_)
    {
        if (ui->graphicsView->scene()) 
        {
            constexpr double scaleFactor = 1.2;  // Zoom factor

            if (event->angleDelta().y() > 0) {
                ui->graphicsView->scale(scaleFactor, scaleFactor);  // Zoom in
            } else {
                ui->graphicsView->scale(1.0 / scaleFactor, 1.0 / scaleFactor);  // Zoom out
            }
        }
    }
    
}

void MainWindow::setDragMode(bool state)
{
    if(state)
    {
        /* mouse scroll */
        //ui->graphicsView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
        //ui->graphicsView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
        ui->graphicsView->setDragMode(QGraphicsView::ScrollHandDrag);
    }
    else
    {
        /* if not zoom_mode */
        //ui->graphicsView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        //ui->graphicsView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        ui->graphicsView->setDragMode(QGraphicsView::NoDrag);
    }
}

void MainWindow::applyStyleWaypoint()
{
    ui->addWaypointBtn->setStyleSheet(
    "QPushButton {"
    "   background-image: url(/home/dolores/Desktop/Git/rom_msgs/rom_dynamics_app/ico/waypoint.png);"
    "   background-repeat: no-repeat;"
    "   background-position: center;"
    "   border: 4px solid white;"
    "}");
    ui->addWallBtn->setStyleSheet(
    "QPushButton {"
    "   background-image: url(/home/dolores/Desktop/Git/rom_msgs/rom_dynamics_app/ico/wall.png);"
    "   background-repeat: no-repeat;"
    "   background-position: center;"
    "   border: 2px solid #979ba1;"
    "}");
    ui->eraserBtn->setStyleSheet(
    "QPushButton {"
    "   background-image: url(/home/dolores/Desktop/Git/rom_msgs/rom_dynamics_app/ico/eraser.png);"
    "   background-repeat: no-repeat;"
    "   background-position: center;"
    "   border: 2px solid #979ba1;"
    "}");
    ui->zoomBtn->setStyleSheet(
    "QPushButton {"
    "   background-image: url(/home/dolores/Desktop/Git/rom_msgs/rom_dynamics_app/ico/zoom.png);"
    "   background-repeat: no-repeat;"
    "   background-position: center;"
    "   border: 2px solid #979ba1;"
    "}");
    ui->normalBtn->setStyleSheet(
    "QPushButton {"
    "   background-image: url(/home/dolores/Desktop/Git/rom_msgs/rom_dynamics_app/ico/normal.png);"
    "   background-repeat: no-repeat;"
    "   background-position: center;"
    "   border: 2px solid #979ba1;"
    "}");
    service_mode_btn_ptr_->setStyleSheet(
    "QPushButton {"
    "   background-image: url(/home/dolores/Desktop/Git/rom_msgs/rom_dynamics_app/ico/robot_waiter_75.png);"
    "   background-repeat: no-repeat;"
    "   background-position: center;"
    "   border: 2px solid #979ba1;"
    "}");
}
void MainWindow::applyStyleWall()
{
    ui->addWaypointBtn->setStyleSheet(
    "QPushButton {"
    "   background-image: url(/home/dolores/Desktop/Git/rom_msgs/rom_dynamics_app/ico/waypoint.png);"
    "   background-repeat: no-repeat;"
    "   background-position: center;"
    "   border: 2px solid #979ba1;"
    "}");
    ui->addWallBtn->setStyleSheet(
    "QPushButton {"
    "   background-image: url(/home/dolores/Desktop/Git/rom_msgs/rom_dynamics_app/ico/wall.png);"
    "   background-repeat: no-repeat;"
    "   background-position: center;"
    "   border: 4px solid white;"
    "}");
    ui->eraserBtn->setStyleSheet(
    "QPushButton {"
    "   background-image: url(/home/dolores/Desktop/Git/rom_msgs/rom_dynamics_app/ico/eraser.png);"
    "   background-repeat: no-repeat;"
    "   background-position: center;"
    "   border: 2px solid #979ba1;"
    "}");
    ui->zoomBtn->setStyleSheet(
    "QPushButton {"
    "   background-image: url(/home/dolores/Desktop/Git/rom_msgs/rom_dynamics_app/ico/zoom.png);"
    "   background-repeat: no-repeat;"
    "   background-position: center;"
    "   border: 2px solid #979ba1;"
    "}");
    ui->normalBtn->setStyleSheet(
    "QPushButton {"
    "   background-image: url(/home/dolores/Desktop/Git/rom_msgs/rom_dynamics_app/ico/normal.png);"
    "   background-repeat: no-repeat;"
    "   background-position: center;"
    "   border: 2px solid #979ba1;"
    "}");
    service_mode_btn_ptr_->setStyleSheet(
    "QPushButton {"
    "   background-image: url(/home/dolores/Desktop/Git/rom_msgs/rom_dynamics_app/ico/robot_waiter_75.png);"
    "   background-repeat: no-repeat;"
    "   background-position: center;"
    "   border: 2px solid #979ba1;"
    "}");
}
void MainWindow::applyStyleEraser()
{
    ui->addWaypointBtn->setStyleSheet(
    "QPushButton {"
    "   background-image: url(/home/dolores/Desktop/Git/rom_msgs/rom_dynamics_app/ico/waypoint.png);"
    "   background-repeat: no-repeat;"
    "   background-position: center;"
    "   border: 2px solid #979ba1;"
    "}");
    ui->addWallBtn->setStyleSheet(
    "QPushButton {"
    "   background-image: url(/home/dolores/Desktop/Git/rom_msgs/rom_dynamics_app/ico/wall.png);"
    "   background-repeat: no-repeat;"
    "   background-position: center;"
    "   border: 2px solid #979ba1;"
    "}");
    ui->eraserBtn->setStyleSheet(
    "QPushButton {"
    "   background-image: url(/home/dolores/Desktop/Git/rom_msgs/rom_dynamics_app/ico/eraser.png);"
    "   background-repeat: no-repeat;"
    "   background-position: center;"
    "   border: 4px solid white;"
    "}");
    ui->zoomBtn->setStyleSheet(
    "QPushButton {"
    "   background-image: url(/home/dolores/Desktop/Git/rom_msgs/rom_dynamics_app/ico/zoom.png);"
    "   background-repeat: no-repeat;"
    "   background-position: center;"
    "   border: 2px solid #979ba1;"
    "}");
    ui->normalBtn->setStyleSheet(
    "QPushButton {"
    "   background-image: url(/home/dolores/Desktop/Git/rom_msgs/rom_dynamics_app/ico/normal.png);"
    "   background-repeat: no-repeat;"
    "   background-position: center;"
    "   border: 2px solid #979ba1;"
    "}");
    service_mode_btn_ptr_->setStyleSheet(
    "QPushButton {"
    "   background-image: url(/home/dolores/Desktop/Git/rom_msgs/rom_dynamics_app/ico/robot_waiter_75.png);"
    "   background-repeat: no-repeat;"
    "   background-position: center;"
    "   border: 2px solid #979ba1;"
    "}");
}
void MainWindow::applyStyleZoom()
{
    ui->addWaypointBtn->setStyleSheet(
    "QPushButton {"
    "   background-image: url(/home/dolores/Desktop/Git/rom_msgs/rom_dynamics_app/ico/waypoint.png);"
    "   background-repeat: no-repeat;"
    "   background-position: center;"
    "   border: 2px solid #979ba1;"
    "}");
    ui->addWallBtn->setStyleSheet(
    "QPushButton {"
    "   background-image: url(/home/dolores/Desktop/Git/rom_msgs/rom_dynamics_app/ico/wall.png);"
    "   background-repeat: no-repeat;"
    "   background-position: center;"
    "   border: 2px solid #979ba1;"
    "}");
    ui->eraserBtn->setStyleSheet(
    "QPushButton {"
    "   background-image: url(/home/dolores/Desktop/Git/rom_msgs/rom_dynamics_app/ico/eraser.png);"
    "   background-repeat: no-repeat;"
    "   background-position: center;"
    "   border: 2px solid #979ba1;"
    "}");
    ui->zoomBtn->setStyleSheet(
    "QPushButton {"
    "   background-image: url(/home/dolores/Desktop/Git/rom_msgs/rom_dynamics_app/ico/zoom.png);"
    "   background-repeat: no-repeat;"
    "   background-position: center;"
    "   border: 4px solid white;"
    "}");
    ui->normalBtn->setStyleSheet(
    "QPushButton {"
    "   background-image: url(/home/dolores/Desktop/Git/rom_msgs/rom_dynamics_app/ico/normal.png);"
    "   background-repeat: no-repeat;"
    "   background-position: center;"
    "   border: 2px solid #979ba1;"
    "}");
    service_mode_btn_ptr_->setStyleSheet(
    "QPushButton {"
    "   background-image: url(/home/dolores/Desktop/Git/rom_msgs/rom_dynamics_app/ico/robot_waiter_75.png);"
    "   background-repeat: no-repeat;"
    "   background-position: center;"
    "   border: 2px solid #979ba1;"
    "}");
}
void MainWindow::applyStyleNormal()
{
    QFont tooltipFont = QToolTip::font();
    tooltipFont.setPointSize(12); 
    QToolTip::setFont(tooltipFont);
    
    QPalette tooltipPalette = QToolTip::palette();
    tooltipPalette.setColor(QPalette::ToolTipBase, QColor(0, 0, 0));   
    tooltipPalette.setColor(QPalette::ToolTipText, QColor(255, 255, 255)); 
    QToolTip::setPalette(tooltipPalette);
    zoom_btn_ptr_->setToolTip("zoom");
    waypoints_btn_ptr_->setToolTip("waypoints");
    virtual_wall_btn_ptr_->setToolTip("virtual walls");
    eraser_btn_ptr_->setToolTip("eraser");
    normal_btn_ptr_->setToolTip("normal");
    service_mode_btn_ptr_->setToolTip("service");
    generalBtnPtr_->setToolTip("emotion");

    ui->addWaypointBtn->setStyleSheet(
    "QPushButton {"
    "   background-image: url(/home/dolores/Desktop/Git/rom_msgs/rom_dynamics_app/ico/waypoint.png);"
    "   background-repeat: no-repeat;"
    "   background-position: center;"
    "   border: 2px solid #979ba1;"
    "}");
    ui->addWallBtn->setStyleSheet(
    "QPushButton {"
    "   background-image: url(/home/dolores/Desktop/Git/rom_msgs/rom_dynamics_app/ico/wall.png);"
    "   background-repeat: no-repeat;"
    "   background-position: center;"
    "   border: 2px solid #979ba1;"
    "}");
    ui->eraserBtn->setStyleSheet(
    "QPushButton {"
    "   background-image: url(/home/dolores/Desktop/Git/rom_msgs/rom_dynamics_app/ico/eraser.png);"
    "   background-repeat: no-repeat;"
    "   background-position: center;"
    "   border: 2px solid #979ba1;"
    "}");
    ui->zoomBtn->setStyleSheet(
    "QPushButton {"
    "   background-image: url(/home/dolores/Desktop/Git/rom_msgs/rom_dynamics_app/ico/zoom.png);"
    "   background-repeat: no-repeat;"
    "   background-position: center;"
    "   border: 2px solid #979ba1;"
    "}");
    ui->normalBtn->setStyleSheet(
    "QPushButton {"
    "   background-image: url(/home/dolores/Desktop/Git/rom_msgs/rom_dynamics_app/ico/normal.png);"
    "   background-repeat: no-repeat;"
    "   background-position: center;"
    "   border: 4px solid white;"
    "}");
    service_mode_btn_ptr_->setStyleSheet(
    "QPushButton {"
    "   background-image: url(/home/dolores/Desktop/Git/rom_msgs/rom_dynamics_app/ico/robot_waiter_75.png);"
    "   background-repeat: no-repeat;"
    "   background-position: center;"
    "   border: 2px solid #979ba1;"
    "}");
}

void MainWindow::applyStyleServiceMode()
{
    ui->addWaypointBtn->setStyleSheet(
        "QPushButton {"
        "   background-image: url(/home/dolores/Desktop/Git/rom_msgs/rom_dynamics_app/ico/waypoint.png);"
        "   background-repeat: no-repeat;"
        "   background-position: center;"
        "   border: 2px solid #979ba1;"
        "}");
        ui->addWallBtn->setStyleSheet(
        "QPushButton {"
        "   background-image: url(/home/dolores/Desktop/Git/rom_msgs/rom_dynamics_app/ico/wall.png);"
        "   background-repeat: no-repeat;"
        "   background-position: center;"
        "   border: 2px solid #979ba1;"
        "}");
        ui->eraserBtn->setStyleSheet(
        "QPushButton {"
        "   background-image: url(/home/dolores/Desktop/Git/rom_msgs/rom_dynamics_app/ico/eraser.png);"
        "   background-repeat: no-repeat;"
        "   background-position: center;"
        "   border: 2px solid #979ba1;"
        "}");
        ui->zoomBtn->setStyleSheet(
        "QPushButton {"
        "   background-image: url(/home/dolores/Desktop/Git/rom_msgs/rom_dynamics_app/ico/zoom.png);"
        "   background-repeat: no-repeat;"
        "   background-position: center;"
        "   border: 2px solid #979ba1;"
        "}");
        ui->normalBtn->setStyleSheet(
        "QPushButton {"
        "   background-image: url(/home/dolores/Desktop/Git/rom_msgs/rom_dynamics_app/ico/normal.png);"
        "   background-repeat: no-repeat;"
        "   background-position: center;"
        "   border: 2px solid #979ba1;"
        "}");
        service_mode_btn_ptr_->setStyleSheet(
        "QPushButton {"
        "   background-image: url(/home/dolores/Desktop/Git/rom_msgs/rom_dynamics_app/ico/robot_waiter_100.png);"
        "   background-repeat: no-repeat;"
        "   background-position: center;"
        "   border: 4px solid white;"
        "}");
}
void MainWindow::onUpdateWpUI(rom_interfaces::msg::ConstructYaml::SharedPtr wplist_ptr)
{
    if (!wplist_ptr) {
        #ifdef ROM_DEBUG
            qDebug() << "[  MainWindow::onUpdateWpUI    ] : wplist_ptr is null!";
        #endif
        return;
    } else if (wplist_ptr->poses.empty()) {
        #ifdef ROM_DEBUG
            qDebug() << "[  MainWindow::onUpdateWpUI    ] : wplist_ptr is initialized but contains no data!";
        #endif
        return;
    } else {
        #ifdef ROM_DEBUG
            qDebug() << "[  MainWindow::onUpdateWpUI    ] : topic received from onUpdateWpUI()";
        #endif
    }
    
    

    // delete original waypoints on scene
    // delete  lists
    // add data to lists
    // update Ui

    // LISTS
        // waypoints_;
        // waypoints_text_;
        // waypoints_direction_;

        // std::unordered_map<std::string, geometry_msgs::msg::Pose> 
        // waypoints_map_;
        // waypoints_scene_;

        waypoints_.clear();
        waypoints_text_.clear();
        waypoints_direction_.clear();
        waypoints_map_.clear();
        waypoints_scene_.clear();

        #ifdef ROM_DEBUG 
        qDebug() << "[  MainWindow::onUpdateWpUI    ] : updating wplist_ptr ....";
        #endif

        for (QGraphicsItem* item : ui->graphicsView->scene()->items()) 
        {
            if (dynamic_cast<QGraphicsEllipseItem*>(item)) 
            {  // Check if it's a Rect Point
                ui->graphicsView->scene()->removeItem(item);
                delete item;
            }
            else if (dynamic_cast<QGraphicsTextItem*>(item)) 
            {  // Check if it's a Rect Point
                ui->graphicsView->scene()->removeItem(item);
                delete item;
            }
            // else if (dynamic_cast<QGraphicsLineItem*>(item)) 
            // {  // Check if it's a line
            //     ui->graphicsView->scene()->removeItem(item);
            //     delete item;
            // }
            // else if (dynamic_cast<QGraphicsRectItem*>(item)) 
            // {       // Check if it's a Rect Point
            //     ui->graphicsView->scene()->removeItem(item);
            //     delete item;
            // }
        }
        
        int radius = 10;
        

        for (size_t i=0; i<wplist_ptr->pose_names.size(); i++)
        {
                QGraphicsTextItem *textItem = ui->graphicsView->scene()->addText(QString::fromStdString(wplist_ptr->pose_names[i]));
                    textItem->setFont(QFont("Arial", 16));  // Set font and size
                    textItem->setDefaultTextColor(QColor("#000000"));  // Set text color
                    textItem->setPos(wplist_ptr->poses[i].position.x -50,wplist_ptr->poses[i].position.y -65);

                QRectF circle(wplist_ptr->poses[i].position.x - radius, wplist_ptr->poses[i].position.y - radius, radius * 3, radius * 3);
                
                QGraphicsEllipseItem* circleItem = new QGraphicsEllipseItem(circle);
                    // Set properties
                    //circleItem->setPen(QPen(Qt::red, 2));
                    //circleItem->setBrush(QBrush(Qt::green)); 

                QRadialGradient radialGradient(circle.center().x(), circle.center().y(), radius * 1.5);
                    // Earth-like colors
                    radialGradient.setColorAt(0, QColor(34, 139, 34));  // Forest Green (center)
                    radialGradient.setColorAt(0.4, QColor(50, 205, 50)); // Lime Green (midway)
                    radialGradient.setColorAt(0.7, QColor(0, 191, 255)); // Deep Sky Blue (near edge)
                    radialGradient.setColorAt(1, QColor(25, 25, 112));  // Midnight Blue (outer edge)

                    // Apply gradient to brush
                    circleItem->setBrush(QBrush(radialGradient));
                    circleItem->setPen(QPen(Qt::black, 2)); // Black outline for contrast

            ui->graphicsView->scene()->addItem(circleItem);

            waypoints_.append(circleItem);
            waypoints_text_.append(textItem);
            waypoints_direction_.append(wplist_ptr->poses[i].orientation.x);

            double mapX = (wplist_ptr->poses[i].position.x * this->map_resolution_) + this->map_origin_x_;
            double mapY = (wplist_ptr->poses[i].position.y * this->map_resolution_) + this->map_origin_y_;

            geometry_msgs::msg::Pose tmp;

            tmp.position.x = mapX;
            tmp.position.y = mapY;

            double wp_heading = wplist_ptr->poses[i].orientation.x;
            double z_in_quaternion;
                    double w_in_quaternion;

            double yaw_in_map = (wp_heading*0.017453292519943295);
            yaw_to_quaternion(yaw_in_map, z_in_quaternion, w_in_quaternion);
                    
            tmp.orientation.z = z_in_quaternion;
            tmp.orientation.w = w_in_quaternion;
                
            waypoints_map_[wplist_ptr->pose_names[i]] = tmp;
                    
                // gui
            waypoints_scene_[wplist_ptr->pose_names[i]] = wplist_ptr->poses[i];
            waypoints_scene_[wplist_ptr->pose_names[i]].orientation.x = 0.000; 
        }
}

void MainWindow::onGoAllBtnClicked(bool status)
{
    #ifdef ROM_DEBUG 
        qDebug() << "[    MainWindow::onGoAllBtnClicked()      ] : ";
    #endif
    ROM_DYNAMICS_UNUSED(status);
    statusLabelPtr_->setText("\n waypoints လွှတ်ပြီ ...\n");
    
        std::vector<std::string> selected_wp_names;

        // ပုံမှန်အားဖြင့်တော့ waypoints အကုန်လုံးပဲ။ waypoints ရွေးပေးချင်ရင် ဒီမှာ hack ။
        for (const auto &pair : waypoints_map_)
        {
            selected_wp_names.push_back( pair.first );

            #ifdef ROM_DEBUG 
                qDebug() << "[    MainWindow::onGoAllBtnClicked()      ] : " << pair.first.c_str();
            #endif
        }
        
        selected_wp_names.push_back("all_goals"); // or ("custom_goals")

        loop_waypoints_ = ui->loopCheckBox->isChecked();

        // btn trigger loop_waypoints_ to true or false; default အားဖြင့် false
        if(loop_waypoints_)
        { 
            selected_wp_names.emplace_back("true");
        }
        else 
        { 
            selected_wp_names.emplace_back("false");
        }
        
        
        if (!selected_wp_names.empty()) 
        {
            //this->blockSignals(false);  
            //qDebug() << "[    MainWindow::onGoAllBtnClicked()      ] : Emitting sendWaypointsGoal...";    
            emit sendWaypointsGoal(selected_wp_names);
            //qDebug() << "[    MainWindow::onGoAllBtnClicked()      ] : sendWaypointsGoal emitted";
            //this->blockSignals(true);    
        }
}


void MainWindow::onUpdateLaser(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    #ifdef ROM_DEBUG 
        qDebug() << "[ MainWindow::onUpdateLaser ] : get laser";
    #endif
    rom_scan_ = scan;
}

// void MainWindow::processScan(QPointF origin)
void MainWindow::processScan()
{
    // Static pointer to store previously added points
    static std::vector<QGraphicsEllipseItem*> ellipseList;

    // First call: initialize and add ellipses
    if (ellipseList.empty()) 
    {
        // Extract the data from the LaserScan message
        std::vector<float> ranges = rom_scan_->ranges;
        float angle_min = rom_scan_->angle_min;
        float angle_increment = rom_scan_->angle_increment;
        int num_readings = ranges.size();

        for (int i = 0; i < num_readings; ++i) 
        {
            float range = ranges[i];
            if (range >= rom_scan_->range_min && range <= rom_scan_->range_max) 
            {
                // Convert polar (range, angle) to Cartesian (x, y)
                float angle = ( angle_min + i * angle_increment )+robot_yaw_rad_; // radian
                // float x = (range * cos(angle))+robot_pose_x_; // meter
                // float y = (range * sin(angle))+robot_pose_y_; // meter

                float x = (range * cos(angle)); // meter
                float y = (range * sin(angle)); // meter

                 // meter to pixel
                double _x = ( ( ( robot_pose_x_+x )- this->map_origin_x_) / this->map_resolution_ ); // * -1  
                double _y = ( ( ( robot_pose_y_+y ) - this->map_origin_y_) / this->map_resolution_ ); // * -1  
                

                // Add ellipse to scene and store pointer
                QGraphicsEllipseItem* ellipse = ui->graphicsView->scene()->addEllipse(
                    _x, _y, 1.2, 1.2, 
                    QPen(Qt::red), QBrush(Qt::red)
                );
                ellipseList.push_back(ellipse);
            }
        }
    }
    else 
    {
        // Update existing ellipses
        std::vector<float> ranges = rom_scan_->ranges;
        float angle_min = rom_scan_->angle_min;
        float angle_increment = rom_scan_->angle_increment;
        int num_readings = std::min(static_cast<int>(ellipseList.size()), static_cast<int>(ranges.size()));

        for (int i = 0; i < num_readings; ++i) 
        {
            float range = ranges[i];
            if (range >= rom_scan_->range_min && range <= rom_scan_->range_max) 
            {
                float angle = ( angle_min + i * angle_increment )+robot_yaw_rad_; // radian
                // float x = (range * cos(angle))+robot_pose_x_;
                // float y = (range * sin(angle))+robot_pose_y_;

                float x = (range * cos(angle)); // meter
                float y = (range * sin(angle)); // meter
                
                // meter to pixel
                double _x = ( ( ( robot_pose_x_+x )- this->map_origin_x_) / this->map_resolution_ ); // * -1  
                double _y = ( ( ( robot_pose_y_+y ) - this->map_origin_y_) / this->map_resolution_ ); // * -1    

                // Update ellipse position (make sure it's still in the scene)
                ellipseList[i]->setRect(_x, _y, 1.2, 1.2);
            }
        }

        // If there are more points in the scan than existing ellipses, add new ones to the scene
        // for (int i = ellipseList.size(); i < ranges.size(); ++i)
        for (size_t i = ellipseList.size(); i < ranges.size(); ++i)
        {
            float range = ranges[i];
            if (range >= rom_scan_->range_min && range <= rom_scan_->range_max) 
            {
                float angle = ( angle_min + i * angle_increment )+robot_yaw_rad_; // radian
                // float x = (range * cos(angle))+robot_pose_x_;
                // float y = (range * sin(angle))+robot_pose_y_;

                float x = (range * cos(angle)); // meter
                float y = (range * sin(angle)); // meter

               // meter to pixel
               double _x = ( ( ( robot_pose_x_+x )- this->map_origin_x_) / this->map_resolution_ ); // * -1  
               double _y = ( ( ( robot_pose_y_+y ) - this->map_origin_y_) / this->map_resolution_ ); // * -1  

                // Add new ellipse to the scene
                QGraphicsEllipseItem* ellipse = ui->graphicsView->scene()->addEllipse(
                    _x, _y, 1.2, 1.2, 
                    QPen(Qt::red), QBrush(Qt::red)
                );
                ellipseList.push_back(ellipse);
            }
        }
    }
}

void MainWindow::onTransformReceived(const ROMTransform rom_tf)
{
    // #ifdef ROM_DEBUG
    //     qDebug() << "[  MainWindow::onTransformReceived    ] : get rom_tf";
    // #endif
    rom_tf_ = rom_tf;
    #ifdef ROM_SHOW_TF
        showOdomAndBaseFootprint();
    #endif
}

void MainWindow::showSceneOriginCoordinate()
{
    QPointF origin(0, 0);

    // Create and add the X-axis (red)
    QGraphicsLineItem *xAxis = ui->graphicsView->scene()->addLine(0, 0, 30, 0);
    QPen xAxisPen(QColor(255, 0, 0), 2);  // Red color and thickness 2
    xAxis->setPen(xAxisPen);
    // Create and add the Y-axis (green)
    QGraphicsLineItem *yAxis = ui->graphicsView->scene()->addLine(0, 0, 0, 30);
    QPen yAxisPen(QColor(0, 255, 0), 2);  // Green color and thickness 2
    yAxis->setPen(yAxisPen);

    // Add arrowheads to the X-axis (red arrow)
    QPolygonF xArrow;
    xArrow << QPointF(30, 0) << QPointF(20, -10) << QPointF(20, 10); // Triangle shape
    QGraphicsPolygonItem *xArrowItem = ui->graphicsView->scene()->addPolygon(xArrow);
    xArrowItem->setBrush(QColor(255, 0, 0)); // Red color
    xArrowItem->setPos(0, 0); // Position at the end of the X-axis

    // Add arrowheads to the Y-axis (green arrow)
    QPolygonF yArrow;
    yArrow << QPointF(0, 30) << QPointF(-10, 20) << QPointF(10, 20); // Triangle shape
    QGraphicsPolygonItem *yArrowItem = ui->graphicsView->scene()->addPolygon(yArrow);
    yArrowItem->setBrush(QColor(0, 255, 0)); // Green color
    yArrowItem->setPos(0, 0); // Position at the end of the Y-axis

    // Add text at the origin (0, 0)
    QGraphicsTextItem *originText = ui->graphicsView->scene()->addText("scene_origin");
    originText->setPos(5, 5);  // Adjust position slightly to avoid overlap with axes
    originText->setDefaultTextColor(Qt::black);  // Set text color to black
    originText->setFont(QFont("Arial", 12));  // Set font to Arial with size 12
}

void MainWindow::showMapOriginCooridinate()
{
        // Convert world coordinates to scene coordinates (adjust as needed)
        // scaleFactor converts meters to scene pixels
        double sceneX_map = (0.0 - this->map_origin_x_) / this->map_resolution_; // * -1  
        double sceneY_map = (0.0 - this->map_origin_y_) / this->map_resolution_; // * -1  

        // Define axis length
        double axis_length = 5.0;

        // Compute rotated X-axis using yaw
        //double yaw_rad = rom_tf_.map_odom_yaw * M_PI / 180.0;  // Convert degrees to radians
        //double yaw_rad = rom_tf_.map_odom_yaw;
        double yaw_rad = 0.0;
        double end_x_x = sceneX_map + axis_length * cos(yaw_rad);
        double end_x_y = sceneY_map + axis_length * sin(yaw_rad);
        QPointF end_point_x(end_x_x, end_x_y);

        // Y-axis perpendicular to X-axis (90-degree rotation)
        double end_y_x = sceneX_map - axis_length * sin(yaw_rad);
        double end_y_y = sceneY_map + axis_length * cos(yaw_rad);
        QPointF end_point_y(end_y_x, end_y_y);

        QPointF origin(sceneX_map, sceneY_map);

        // Create and add the X-axis (red)
        static QGraphicsLineItem *xAxis = nullptr;
        if(!xAxis)
        {
            xAxis = ui->graphicsView->scene()->addLine(QLineF(origin, end_point_x));
            QPen xAxisPen(QColor(255, 0, 0), 1);  // Red color and thickness 2
            xAxis->setPen(xAxisPen);
        }else{
            xAxis->setLine(QLineF(origin, end_point_x));
        }

        // Create and add the Y-axis (green)
        static QGraphicsLineItem *yAxis = nullptr;
        if(!yAxis)
        {
            yAxis = ui->graphicsView->scene()->addLine(QLineF(origin, end_point_y));
            QPen yAxisPen(QColor(0, 255, 0), 1);  // Red color and thickness 2
            yAxis->setPen(yAxisPen);
        }else{
            yAxis->setLine(QLineF(origin, end_point_y));
        }

        // Add text at the origin (0, 0)
        static QGraphicsTextItem *originText = nullptr;
        if(!originText)
        {
            originText = ui->graphicsView->scene()->addText("map");
            originText->setPos(sceneX_map, sceneY_map); 
            originText->setDefaultTextColor(Qt::black);  
            originText->setFont(QFont("Arial", 2));  
        }
        else{
            originText->setPos(sceneX_map, sceneY_map); 
        }
        // QGraphicsLineItem *xAxis = ui->graphicsView->scene()->addLine(QLineF(origin, end_point_x));
        // QPen xAxisPen(QColor(255, 0, 0), 1);  // Red color and thickness 2
        // xAxis->setPen(xAxisPen);
        // xAxis->setData(0, 221);

        // // Create and add the Y-axis (green)
        // QGraphicsLineItem *yAxis = ui->graphicsView->scene()->addLine(QLineF(origin, end_point_y));
        // QPen yAxisPen(QColor(0, 255, 0), 1);  // Green color and thickness 2
        // yAxis->setPen(yAxisPen);
        // yAxis->setData(0, 221);

        // // Add text at the origin (0, 0)
        // QGraphicsTextItem *originText = ui->graphicsView->scene()->addText("map");
        // originText->setPos(sceneX_map, sceneY_map);  // Adjust position slightly to avoid overlap with axes
        // originText->setDefaultTextColor(Qt::black);  // Set text color to black
        // originText->setFont(QFont("Arial", 2));  // Set font to Arial with size 12
        // originText->setData(0, 221);
}

void MainWindow::showOdomAndBaseFootprint()
{
    #ifdef ROM_SHOW_ODOM_TF
        showOdom();
    #endif
    #ifdef ROM_SHOW_BASEFOOTPRINT_TF
        showBaseFootprint();
    #endif
}

void MainWindow::showOdom()
{
        // Convert world coordinates to scene coordinates (adjust as needed)
        // scaleFactor converts meters to scene pixels
        double sceneX_map = (rom_tf_.map_odom_x - this->map_origin_x_) / this->map_resolution_; // * -1  
        double sceneY_map = (rom_tf_.map_odom_y - this->map_origin_y_) / this->map_resolution_; // * -1  

        // Define axis length
        double axis_length = 5.0;

        // Compute rotated X-axis using yaw
        //double yaw_rad = rom_tf_.map_odom_yaw * M_PI / 180.0;  // Convert degrees to radians
        double yaw_rad = rom_tf_.map_odom_yaw;
        double end_x_x = sceneX_map + axis_length * cos(yaw_rad);
        double end_x_y = sceneY_map + axis_length * sin(yaw_rad);
        QPointF end_point_x(end_x_x, end_x_y);

        // Y-axis perpendicular to X-axis (90-degree rotation)
        double end_y_x = sceneX_map - axis_length * sin(yaw_rad);
        double end_y_y = sceneY_map + axis_length * cos(yaw_rad);
        QPointF end_point_y(end_y_x, end_y_y);

        QPointF origin(sceneX_map, sceneY_map);

        // Create and add the X-axis (red)
        static QGraphicsLineItem *xAxis = nullptr;
        if(!xAxis)
        {
            xAxis = ui->graphicsView->scene()->addLine(QLineF(origin, end_point_x));
            QPen xAxisPen(QColor(255, 0, 0), 1);  // Red color and thickness 2
            xAxis->setPen(xAxisPen);
        }else{
            xAxis->setLine(QLineF(origin, end_point_x));
        }

        // Create and add the Y-axis (green)
        static QGraphicsLineItem *yAxis = nullptr;
        if(!yAxis)
        {
            yAxis = ui->graphicsView->scene()->addLine(QLineF(origin, end_point_y));
            QPen yAxisPen(QColor(0, 255, 0), 1);  // Red color and thickness 2
            yAxis->setPen(yAxisPen);
        }else{
            yAxis->setLine(QLineF(origin, end_point_y));
        }

        // Add text at the origin (0, 0)
        // static QGraphicsTextItem *originText = nullptr;
        // if(!originText)
        // {
        //     originText = ui->graphicsView->scene()->addText("odom");
        //     originText->setPos(sceneX_map, sceneY_map); 
        //     originText->setDefaultTextColor(Qt::black);  
        //     originText->setFont(QFont("Arial", 2));  
        // }
        // else{
        //     originText->setPos(sceneX_map, sceneY_map); 
        // }
}

void MainWindow::showBaseFootprint()
{
        // Convert world coordinates to scene coordinates (adjust as needed)
        // scaleFactor converts meters to scene pixels
        double sceneX_map = (rom_tf_.odom_base_footprint_x - this->map_origin_x_) / this->map_resolution_; // * -1  
        double sceneY_map = (rom_tf_.odom_base_footprint_y - this->map_origin_y_) / this->map_resolution_; // * -1  


        // Define axis length
        double axis_length = 5.0;

        // Compute rotated X-axis using yaw
        //double yaw_rad = rom_tf_.map_odom_yaw * M_PI / 180.0;  // Convert degrees to radians
        double yaw_rad = rom_tf_.odom_base_footprint_yaw;        // radian
        double end_x_x = sceneX_map + axis_length * cos(yaw_rad);
        double end_x_y = sceneY_map + axis_length * sin(yaw_rad);
        QPointF end_point_x(end_x_x, end_x_y);

        // save robot pose
        robot_pose_x_ = rom_tf_.odom_base_footprint_x;
        robot_pose_y_ = rom_tf_.odom_base_footprint_y;   
        robot_yaw_rad_ = yaw_rad;

        // Display Robot Pose
        double x_feet = (robot_pose_x_ * meter_to_foot_constant);
        double y_feet = (robot_pose_y_ * meter_to_foot_constant);
        double theta_degree = (robot_yaw_rad_ * 180.0 / M_PI); //convert radian to degrees 
        ui->label->setText(QString("Robot Pose( ပေ ၊ ဒီဂရီ )"));
        ui->xValueLabel->setText(QString("%1").arg(x_feet, 0, 'f', 1));
        ui->yValueLabel->setText(QString("%1").arg(y_feet, 0, 'f', 1));
        ui->phiValueLabel->setText(QString("%1").arg(theta_degree, 0, 'f', 1));


        // Y-axis perpendicular to X-axis (90-degree rotation)
        double end_y_x = sceneX_map - axis_length * sin(yaw_rad);
        double end_y_y = sceneY_map + axis_length * cos(yaw_rad);
        QPointF end_point_y(end_y_x, end_y_y);
        
        QPointF origin(sceneX_map, sceneY_map);
        
        // --- Add TF axis ---
        static QGraphicsLineItem *xAxis = nullptr;
        if(!xAxis)
        {
            xAxis = ui->graphicsView->scene()->addLine(QLineF(origin, end_point_x));
            QPen xAxisPen(QColor(255, 0, 0), 1);  // Red color and thickness 2
            xAxis->setPen(xAxisPen);
        }else{
            xAxis->setLine(QLineF(origin, end_point_x));
        }
        // to delete xAxis->setData(0, 223);

        // Create and add the Y-axis (green)
        // QGraphicsLineItem *yAxis = ui->graphicsView->scene()->addLine(QLineF(origin, end_point_y));
        // QPen yAxisPen(QColor(0, 255, 0), 1);  // Green color and thickness 2
        // yAxis->setPen(yAxisPen);
        // yAxis->setData(0, 223);

        // Add text at the origin (0, 0)
        // static QGraphicsTextItem *originText = nullptr;
        // if(!originText)
        // {
        //     originText = ui->graphicsView->scene()->addText("base_footprint");
        //     originText->setPos(sceneX_map, sceneY_map); 
        //     originText->setDefaultTextColor(Qt::black);  // Set text color to black
        //     originText->setFont(QFont("Arial", 2));  // Set font to Arial with size 12
        // }
        // else{
        //     originText->setPos(sceneX_map, sceneY_map); 
        // }

        // add robot 
        QPolygonF arrowShape;
        arrowShape << QPointF(0, -5)  // Tip of the arrow
                   << QPointF(-2.5, 5)  // Bottom left
                   << QPointF(0, 2.5)    // Inner left
                   << QPointF(2.5, 5)   // Bottom right
                   << QPointF(0, -5); // Back to tip to close the shape
    
        // Create a polygon item for the arrow
        static QGraphicsPolygonItem *arrowItem = nullptr;
        if (!arrowItem)
        {
            arrowItem = new QGraphicsPolygonItem(arrowShape);
            arrowItem->setPen(QPen(Qt::blue, 1));
            arrowItem->setBrush(Qt::NoBrush);
            ui->graphicsView->scene()->addItem(arrowItem);
        }
    
        // Update position
        arrowItem->setPos(sceneX_map, sceneY_map); // ok
    
        // Rotate the arrow based on yaw (in degrees)
        double yaw_deg = rom_tf_.odom_base_footprint_yaw * 180.0 / M_PI;
        arrowItem->setRotation(yaw_deg+90.0);
        // --------------------- end add robot

        
        // laser scan ၁၀ ကြိမ်တကြိမ်
        static int count = 0;
        if (count%10 == 0)
        {
            //processScan(origin);
            processScan();
        }
        count++;
       if (count == 109)
       {
         count = 0;
       }
}

void MainWindow::showEyesWidget()
{
    robotEyesWidgetPtr_->move(this->pos().x(), this->pos().y());
    robotEyesWidgetPtr_->show(); 
}

void MainWindow::resetInactivityTimer()
{
    inactivityTimer_->start();
}

void MainWindow::handleInactivityTimeout()
{
    showEyesWidget();
}

void MainWindow::setChargingPoint()
{
    // ???? to solve ==> mapping မှာ robot နဲ့ tf ဘာကြောင့်မပေါ်သလဲ။ robot pose topic အလုပ်လုပ်/မလုပ် ။ laser ပေါ်သင့်။
    // ???? waypoints ui များကို ဘယ်လိုနည်းနဲ့ update လုပ်မှာလဲ။
    // ???? eraser mode မှာ waypoint ဖျက်ရင် robot ရဲ့ yaml ကို ရောဖျက်သင့် မဖျက်သင့်။ ?
    // ???? mapping mode မှာ new wp ထားပေးလို့ ရသင့် / မရသင့်။ wp, eraser, wall တို့ hide သင့် မ hide သင့်။
    // ???? reeman ဘယ်လိုလုပ်သလဲ။
    // mapping mode ပြောင်ရင် auto rebuild လုပ်သင့် / မလုပ်သင့်။


    // ၀။ Dialog box ဖြင့် confirm တောင်းပါ။

    // ၁။ လက်ရှိ robot pose ကို ယူပါ။

    // ၂။ charging_point အမည်ဖြင့်  wp lists များထဲထည့်ပါ။

    // ၃။ Ui မှာ display ပြဖို့ လုပ်ပါ။
}

void MainWindow::setCurrentPointAs()
{
    // ၀။ Dialog box ဖြင့် pose name တောင်းပါ။
    
    // ၁။ လက်ရှိ robot pose ကို ယူပါ။

    // ၂။ pose name အမည်ဖြင့်  wp lists များထဲထည့်ပါ။

    // ၃။ Ui မှာ display ပြဖို့ လုပ်ပါ။
}

void MainWindow::setProductionPoint()
{
    // ၀။ Dialog box ဖြင့် confirm တောင်းပါ။
    
    // ၁။ လက်ရှိ robot pose ကို ယူပါ။

    // ၂။ production_point အမည်ဖြင့်  wp lists များထဲထည့်ပါ။

    // ၃။ Ui မှာ display ပြဖို့ လုပ်ပါ။
}

//---------------------------- SERVICE CLIENT ----------------------------
ServiceClient::ServiceClient() {
    //rclcpp::init(0, nullptr);
    node = rclcpp::Node::make_shared("qt_service_client");
    client_ = node->create_client<rom_interfaces::srv::WhichMaps>("/which_maps");

    // Start a separate thread for the ROS 2 spinning
    rosServiceClientThread_ = std::thread(&ServiceClient::spin, this);
}

/// SERVICE CLIENT
ServiceClient::~ServiceClient() {
    rclcpp::shutdown();
    if (rosServiceClientThread_.joinable()) {
        rosServiceClientThread_.join();
    }
}

void ServiceClient::spin() {
    rclcpp::spin(node);
}


void ServiceClient::sendRequest(const std::string& request_string, const std::string& optional_param = "") {

// # -1 [service not ok], 1 [service ok], 
// # 2 [maps exist], 3 [maps do not exist], 
// # 4 [save map ok], 5 [save map not ok]
// # 6 [select map ok], 7 [select map not ok], 

// # 8 [mapping mode ok], 9 [mapping mode not ok],
// # 10 [navi mode ok], 11 [navi mode not ok]
// # 12 [remapping mode ok], 13 [remapping mode not ok]

    auto request = std::make_shared<rom_interfaces::srv::WhichMaps::Request>();
    
    #ifdef ROM_DEBUG
        qDebug() << "[  ServiceClient::sendRequest  ] : mode change";
    #endif
    
    request->request_string = request_string;

    // reconfigure for save_map and select_map
    if (request_string == "save_map") {
        request->map_name_to_save = optional_param;
    } 
    else if (request_string == "select_map") {
        request->map_name_to_select = optional_param;
    }
    
    // check service        
    if (!client_->wait_for_service(std::chrono::seconds(5))) {
        emit responseReceived(-1); // Error: service not available
        return;
    }

    // send request
    auto future = client_->async_send_request(request);
    future.wait();

    // get response
    try 
    {
        auto response = future.get();

        // response for label updating
        emit responseReceived(response->status);

        // response for map exist or not
        if (response->status == 2)
        {
            emit responseDataReceived(response);
        }
        
    // response for error
    } catch (const std::exception &e) {
        emit responseReceived(-1); // Error: response failure
    }
}


//---------------------------- CUSTOM DIALOG ----------------------------

CustomDialog::CustomDialog(QWidget *parent) : QDialog(parent) {
    QVBoxLayout *layout = new QVBoxLayout;

    QLabel *nameLabel = new QLabel("Waypoint Name:");
    nameEdit = new QLineEdit;
    layout->addWidget(nameLabel);
    layout->addWidget(nameEdit);

    // QLabel *numberLabel = new QLabel("Waypoint Number:");
    // numberEdit = new QSpinBox;
    // layout->addWidget(numberLabel);
    // layout->addWidget(numberEdit);

    // Create QDial for number selection (useful for selecting headings or other ranges)
    QLabel *dialLabel = new QLabel("Direction (0-360):");
    dial = new QDial;
    dial->setRange(0, 360);  // Set the range of the dial (for example, 0 to 360 for degrees)
    dial->setValue(180);  // Default value is 0
    dial->setFixedSize(200,200);
    layout->addWidget(dialLabel);
    //layout->addWidget(dial);
    //layout->addWidget(dialLabel, 0, Qt::AlignHCenter);  // Align label to center
    layout->addWidget(dial, 0, Qt::AlignHCenter);  

    okButton = new QPushButton("OK");
    okButton->setEnabled(false);
    layout->addWidget(okButton);
    
    connect(nameEdit, &QLineEdit::textChanged, this, &CustomDialog::validateInput);
    connect(okButton, &QPushButton::clicked, this, &CustomDialog::accept);

    setLayout(layout);
    setFixedSize(WAYPOINT_DIALOG_WIDTH, WAYPOINT_DIALOG_HEIGHT);
    // Initial validation
    validateInput();
}

void CustomDialog::validateInput() {
    // Enable or disable OK button based on whether the QLineEdit text is empty
    okButton->setEnabled(!nameEdit->text().isEmpty());
}

QString CustomDialog::getName() const {
    return nameEdit->text();
}

int CustomDialog::getNumber() const {
    //return numberEdit->value();
    return dial->value();
}


// file_path = /home/dolores/Desktop/Git/rom_msgs/rom_dynamics_app/ico/normal.png