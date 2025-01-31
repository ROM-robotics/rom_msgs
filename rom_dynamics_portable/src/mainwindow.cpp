#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <cmath> // For atan2 and M_PI
#include "rom_algorithm.h"

extern void shutdown_thread();

#define ROM_Q_DEBUG 1

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
    selectMapBtnPtr_ = ui->selectMapBtn;
    selectMapBtnPtr_->setText("Relocate");

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
    btnCancelGoal_ = ui->cancelBtn;
    btnReturnToHome_ = ui->rthBtn;


    // Initialize the ROS Worker
    service_clientPtr_ = new ServiceClient();
    rosServiceClientThreadPtr_ = new QThread();
    service_clientPtr_->moveToThread(rosServiceClientThreadPtr_);

    connect(sendMappingBtnPtr_, &QPushButton::clicked, this, &MainWindow::sendMappingMode);
    connect(sendNavigationBtnPtr_, &QPushButton::clicked, this, &MainWindow::sendNavigationMode);
    connect(sendRemappingBtnPtr_, &QPushButton::clicked, this, &MainWindow::sendRemappingMode);

    connect(saveMapBtnPtr_, &QPushButton::clicked, this, &MainWindow::saveMapClicked);
    connect(openMapBtnPtr_, &QPushButton::clicked, this, &MainWindow::openMapClicked);
    // connect(selectMapBtnPtr_, &QPushButton::clicked, this, &MainWindow::selectMapClicked);
    connect(this, &MainWindow::selectMap, this, &MainWindow::selectMapClicked);

    connect(btnGoToGoal_, &QPushButton::clicked, this, &MainWindow::on_goBtn_clicked);
    connect(btnCancelGoal_, &QPushButton::clicked, this, &MainWindow::on_cancelBtn_clicked);
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
    //qRegisterMetaType<rom_interfaces::srv::WhichMaps::Response::SharedPtr>("rom_interfaces::srv::WhichMaps::Response::SharedPtr");
    qRegisterMetaType<std::shared_ptr<rom_interfaces::srv::WhichMaps::Response>>("std::shared_ptr<rom_interfaces::srv::WhichMaps::Response>");

    
    statusLabelPtr_->setText("App အား အသုံးပြုဖို့အတွက် အောက်ပါ ROS2 humble package နှစ်ခုကို install လုပ်ပါ။။\n      - rom_interfaces\n      - which_maps\n\n $ ros2 run which_maps which_maps_server\n # map save ရန် lifecycle လို/မလို စစ်ဆေးပါ။\n");

    ui->saveMapBtn->hide();//setEnabled(false);
    ui->openMapBtn->show();//setEnabled(true);
    ui->selectMapBtn->show();//setEnabled(true);
}


MainWindow::~MainWindow()
{
    rclcpp::shutdown();

    rosServiceClientThreadPtr_->quit();
    rosServiceClientThreadPtr_->wait();

    delete service_clientPtr_;
    delete rosServiceClientThreadPtr_;
    removeBusyDialog(); 
}


void MainWindow::displayCurrentPose(const nav_msgs::msg::Odometry::SharedPtr msg) 
{
    double x = (msg->pose.pose.position.x * meter_to_foot_constant);
    double y = (msg->pose.pose.position.y * meter_to_foot_constant);

    double theta = quaternion_to_euler_yaw(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    double theta_degree = theta * radian_to_degree_constant;

    ui->xValueLabel->setText(QString("%1").arg(x, 0, 'f', 1));
    ui->yValueLabel->setText(QString("%1").arg(y, 0, 'f', 1));
    ui->phiValueLabel->setText(QString("%1").arg(theta_degree, 0, 'f', 1));
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
        ui->mappingBtn->setStyleSheet("background: qlineargradient(x1: 0, y1: 0, x2: 1, y2: 1, stop: 0 #1f406e,stop: 0.8 #87CEEB, stop: 1 #132742);");
        ui->navigationBtn->setStyleSheet("background-color: white;");
        ui->remappingBtn->setStyleSheet("background-color: white;");

        showBusyDialog();
        setButtonsEnabled(false);

        ui->saveMapBtn->show();
        ui->openMapBtn->hide();
        ui->selectMapBtn->hide();
        // ui->saveMapBtn->setEnabled(true);
        // ui->openMapBtn->setEnabled(false);
        // ui->selectMapBtn->setEnabled(false);
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
        ui->mappingBtn->setStyleSheet("background-color: white;");
        // ui->navigationBtn->setStyleSheet("background-color: green;");
        ui->navigationBtn->setStyleSheet("background: qlineargradient(x1: 0, y1: 0, x2: 1, y2: 1, stop: 0 #1f406e,stop: 0.8 #87CEEB, stop: 1 #132742);");
        ui->remappingBtn->setStyleSheet("background-color: white;");

        showBusyDialog();
        setButtonsEnabled(false);

        ui->saveMapBtn->hide();
        ui->openMapBtn->show();
        ui->selectMapBtn->show();
        // ui->saveMapBtn->setEnabled(false);
        // ui->openMapBtn->setEnabled(true);
        // ui->selectMapBtn->setEnabled(true);
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
        ui->mappingBtn->setStyleSheet("background-color: white;");
        ui->navigationBtn->setStyleSheet("background-color: white;");
        //ui->remappingBtn->setStyleSheet("background-color: green;");
        ui->remappingBtn->setStyleSheet("background: qlineargradient(x1: 0, y1: 0, x2: 1, y2: 1, stop: 0 #1f406e,stop: 0.8 #87CEEB, stop: 1 #132742);");

        showBusyDialog();
        setButtonsEnabled(false);

        ui->saveMapBtn->show();
        ui->openMapBtn->show();
        ui->selectMapBtn->show();
        // ui->saveMapBtn->setEnabled(true);
        // ui->openMapBtn->setEnabled(true);
        // ui->selectMapBtn->setEnabled(true);
    }
}


void MainWindow::on_shutdownBtn_clicked()
{
    statusLabelPtr_->setText("\nShutdown 0 ...\n");

    rclcpp::shutdown(); // Stop the ROS 2 node and executor

    QApplication::quit(); // Exit the application
}


void MainWindow::on_btnEstop_clicked()
{
    statusLabelPtr_->setText("\nActivating E-Stop ...\n");
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
            saveMapBtnPtr_->setStyleSheet("background-color: white;"); 
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
    saveMapBtnPtr_->setStyleSheet("background-color: white;"); 
    //openMapBtnPtr_->setStyleSheet("background-color: none;");
    //selectMapBtnPtr_->setStyleSheet("background-color: none;");

    hideBusyDialog();
    setButtonsEnabled(true);
}



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
        //saveMapBtnPtr_->setStyleSheet("background-color: green;");
        saveMapBtnPtr_->setStyleSheet("background: qlineargradient(x1: 0, y1: 0, x2: 1, y2: 1, stop: 0 #1f406e, stop: 1 #132742);");
        
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
        // ui->selectMapBtn->hide();
        // ui->saveMapBtn->setEnabled(true);
        // ui->openMapBtn->setEnabled(false);
        // ui->selectMapBtn->setEnabled(false);
    

    
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
    ui->btnForward->setStyleSheet("color: white; background-color: none; border: 2px solid white;");
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
    ui->btnRight->setStyleSheet("color: white; background-color: none; border: 2px solid white;");
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
    ui->btnLeft->setStyleSheet("color: white; background-color: none; border: 2px solid white;");
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
    ui->btnStop->setStyleSheet("color: white; background-color: none; border: 2px solid white;");
}


/// for dragging
void MainWindow::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
        // When the left mouse button is pressed, store the offset relative to the top-left corner
        dragging = true;
        dragPosition = event->globalPos() - frameGeometry().topLeft();
        event->accept();
    }
}

void MainWindow::mouseMoveEvent(QMouseEvent *event)
{
    if (dragging && (event->buttons() & Qt::LeftButton)) {
        // Move the window based on the offset calculated in mousePressEvent
        move(event->globalPos() - dragPosition);
        event->accept();
    }
}

void MainWindow::mouseReleaseEvent(QMouseEvent *event)
{
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

        busyDialog_->move(350, 250); 
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
        statusLabelPtr_->setText("\n   x , y, theta တန်ဖိုးများထည့်သွင်းပါ။\n");
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
    
    // btnCancelGoal_->setEnabled(true);
    // btnReturnToHome_->setEnabled(true);
    // ui->btnEstop->setEnabled(true);
    
    auto pose = geometry_msgs::msg::Pose::SharedPtr(new geometry_msgs::msg::Pose());
    pose->position.x = x;
    pose->position.y = y;

    yaw_to_quaternion(theta, pose->orientation.z, pose->orientation.w);
    #ifdef ROM_Q_DEBUG 
        qDebug() << "[    on_goBtn_clicked        ] : sending pose";
    #endif

    //emit sendNavigationGoal(pose);
    
    //Emit the navigation goal
    QMetaObject::invokeMethod(this, [this, pose]() {
        emit sendNavigationGoal(pose);
    }, Qt::QueuedConnection);
}


void MainWindow::on_cancelBtn_clicked()
{
    if (is_goal_active_) 
    {
        statusLabelPtr_->setText("\n Cancel Goal လုပ်လိုက်ပါပြီ။ \n");
            emit sendCancelGoal(active_goal_uuid_);
        active_goal_uuid_.fill(0);
        is_goal_active_ = false;
    }
    else
    {
        statusLabelPtr_->setText("\n Cancel လုပ်ဖို့ Goal မရှိပါ။ \n");
        return;
    }
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
    
    // btnCancelGoal_->setEnabled(true);
    // ui->btnEstop->setEnabled(true);
    
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

    #ifdef ROM_Q_DEBUG 
        qDebug() << "[ onNavigationResult() slot  ] : get Navigation Result " << QString::fromStdString(result_status);
    #endif
    
    setButtonsEnabled(true);
    hideBusyDialog();
}


void MainWindow::onSendGoalId(const rclcpp_action::GoalUUID& goal_uuid)
{
    // need char array to string conversion

    QString currentText = statusLabelPtr_->text();
    //QString statusText = QString("\nGoal ID : %1\n").arg(QString::fromStdString(goal_id));
    QString statusText = QString("\nGoal ID \n");
    QString updateText = currentText + statusText;
    statusLabelPtr_->setText(updateText);

    #ifdef ROM_Q_DEBUG
        //qDebug() << "Goal ID from Mainwindow::sendGoalId " << QString::fromStdString(goal_id);
        qDebug() << "Goal ID from Mainwindow::sendGoalId ";
    #endif

    //goal_id_ = goal_id;
    active_goal_uuid_ = goal_uuid;
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

void MainWindow::onCmdServiceResponse(bool success){
    if(!success){
        QString statusText = QString("Service not found !\n");
        QString updateText = currentText_ + statusText;
        statusLabelPtr_->setText(updateText);

        #ifdef ROM_Q_DEBUG 
            qDebug() << "[    onCmdServiceResponse        ] : Service not found";
        #endif

        // ui->btnForward->setStyleSheet("color: #979ba1; background-color: none; border: 2px solid gray;");
        // ui->btnRight->setStyleSheet("color: #979ba1; background-color: none; border: 2px solid gray;");
        // ui->btnLeft->setStyleSheet("color: #979ba1; background-color: none; border: 2px solid gray;");
        // ui->btnStop->setStyleSheet("color: #979ba1; background-color: none; border: 2px solid gray;");

    }else {
        #ifdef ROM_Q_DEBUG 
            qDebug() << "[    onCmdServiceResponse        ] : Service OK !";
        #endif
    }

    

}


void MainWindow::onResponseDataReceived(std::shared_ptr<rom_interfaces::srv::WhichMaps::Response> response)
{
    if (!response || response->total_maps <= 0 || response->map_names.empty()) 
    {
        qDebug() << "Invalid or empty response received!";
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
            #ifdef ROM_Q_DEBUG 
                qDebug() << "Selected map:" << selectedItem->text();
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
    ui->btnEstop->setStyleSheet(
    "QPushButton:pressed {"
    "    background-color: rgb(255, 200, 200);"       
    "}"
    );
    ui->btnStop->setStyleSheet(
    "QPushButton {"
    "   border: 2px solid white;"
    "   background-color: none;"
    "   color: white;"
    "}"
    "QPushButton:pressed {"
    "    background-color: rgb(255, 200, 200);"       
    "}"
    );

    ui->btnForward->setStyleSheet(
    "QPushButton {"
    "   background-color: none;"
    "   color: #979ba1;"
    "}"
    "QPushButton:pressed {"
    "    background-color: rgb(200, 255, 200);"      
    "}"
    );
    ui->btnLeft->setStyleSheet(
    "QPushButton {"
    "   background-color: none;"
    "   color: #979ba1;"
    "}"
    "QPushButton:pressed {"
    "    background-color: rgb(200, 255, 200);"       
    "}"
    );
    ui->btnRight->setStyleSheet(
    "QPushButton {"
    "   background-color: none;"
    "   color: #979ba1;"
    "}"
    "QPushButton:pressed {"
    "    background-color: rgb(200, 255, 200);"        
    "}"
    );

    ui->saveMapBtn->setStyleSheet(
    "QPushButton {"
    "   background-color: white;"
    "}"
    "QPushButton:pressed {"
    "    background-color: rgb(200, 255, 200);" 
    "}"       
    );
    ui->openMapBtn->setStyleSheet(
    "QPushButton {"
    "   background-color: white;"
    "}"
    "QPushButton:pressed {"
    "    background-color: rgb(200, 255, 200);"       
    "}"
    );
    ui->selectMapBtn->setStyleSheet(
    "QPushButton {"
    "   background-color: white;"
    "}"
    "QPushButton:pressed {"
    "    background-color: rgb(200, 255, 200);"       
    "}"
    );


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
    ui->navigationBtn->setStyleSheet(
    "QPushButton {"
    "   background: qlineargradient("
    "       x1: 0, y1: 0, x2: 1, y2: 1, " // Diagonal gradient
    "       stop: 0 #1f406e, "            // Deep blue (core Neptune color)re
    "       stop: 0.8 #87CEEB, "         // Sky blue highlights
    "       stop: 1 #132742"             // White for clouds
    "   );"
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
    "   font-size: 35px;"
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
    "   font-size: 35px;"
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
    ui->cancelBtn->setStyleSheet(
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
    "   font-size: 36px;"
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
    ui->btnEstop->setStyleSheet(
    "QPushButton {"
    "   border: 1px solid gray;"
    "   background-color: white;"
    "   color: black;"
    "   font-weight: bold;"
    "}"
     "QPushButton:hover {"
     "   background-color: black;"
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
