#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <cmath> // For atan2 and M_PI
#include "rom_algorithm.h"

extern void shutdown_thread();

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), dragging(false)
{
    setWindowTitle("ROM Dynamics Company's Robot Suite");
    ui = std::make_shared<Ui::MainWindow>();
    ui->setupUi(this);

    setWindowFlags(Qt::Window | Qt::CustomizeWindowHint);
    ui->centralwidget->setStyleSheet("background-color: rgba(255, 255, 255, 230);"); // 50% opacity
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

    // Initialize the ROS Worker
    service_client_ = new ServiceClient();
    rosThread = new QThread();
    service_client_->moveToThread(rosThread);

    connect(sendMappingBtnPtr_, &QPushButton::clicked, this, &MainWindow::sendMappingMode);
    connect(sendNavigationBtnPtr_, &QPushButton::clicked, this, &MainWindow::sendNavigationMode);
    connect(sendRemappingBtnPtr_, &QPushButton::clicked, this, &MainWindow::sendRemappingMode);

    connect(saveMapBtnPtr_, &QPushButton::clicked, this, &MainWindow::saveMapClicked);
    connect(openMapBtnPtr_, &QPushButton::clicked, this, &MainWindow::openMapClicked);
    connect(selectMapBtnPtr_, &QPushButton::clicked, this, &MainWindow::selectMapClicked);
    

    // Open map အတွက် responseReceived, onResponseReceived အသစ်ပြန်ရေးရန်
    connect(service_client_, &ServiceClient::responseReceived, this, &MainWindow::onResponseReceived);

    rosThread->start();

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
    "QPushButton:pressed {"
    "    background-color: rgb(255, 200, 200);"       
    "}"
    );

    ui->btnForward->setStyleSheet(
    "QPushButton:pressed {"
    "    background-color: rgb(200, 255, 200);"      
    "}"
    );
    ui->btnLeft->setStyleSheet(
    "QPushButton:pressed {"
    "    background-color: rgb(200, 255, 200);"       
    "}"
    );
    ui->btnRight->setStyleSheet(
    "QPushButton:pressed {"
    "    background-color: rgb(200, 255, 200);"        
    "}"
    );

    ui->saveMapBtn->setStyleSheet(
    "QPushButton:pressed {"
    "    background-color: rgb(200, 255, 200);" 
    "}"       
    );
    ui->openMapBtn->setStyleSheet(
    "QPushButton:pressed {"
    "    background-color: rgb(200, 255, 200);"       
    "}"
    );
    ui->selectMapBtn->setStyleSheet(
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
     "   background-color: red;"
     "   color: white;"
     "}"
    );
    ui->goBtn->setStyleSheet(
    "QPushButton {"
    "   border-radius: 55px;"
    "   border: 1px solid gray;"
    "   background-color: white;"
    "   color: green;"
    "   font-size: 56px;"
    "}"
    "QPushButton:hover {"
    "   background-color: green;"
    "   color: white;"
    "}"
    );

    current_mode_ = "navi";

    /* signal ပို့လို့ရတဲ့ meta object ဖြစ်အောင်လို့ */
    qRegisterMetaType<nav_msgs::msg::Odometry::SharedPtr>("nav_msgs::msg::Odometry::SharedPtr");

    statusLabelPtr_->setText("App အား အသုံးပြုဖို့အတွက် အောက်ပါ ROS2 humble package နှစ်ခုကို install လုပ်ပါ။။\n      - rom_interfaces\n      - which_maps\n\n $ ros2 run which_maps which_maps_server\n # map save ရန် lifecycle လို/မလို စစ်ဆေးပါ။\n");

    ui->saveMapBtn->hide();//setEnabled(false);
    ui->openMapBtn->show();//setEnabled(true);
    ui->selectMapBtn->show();//setEnabled(true);
}


MainWindow::~MainWindow()
{
    rclcpp::shutdown();

    rosThread->quit();
    rosThread->wait();

    delete service_client_;
    delete rosThread;
    removeBusyDialog(); 
}


void MainWindow::displayCurrentPose(const nav_msgs::msg::Odometry::SharedPtr msg) 
{
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double theta = quaternion_to_euler(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    double theta_degree = theta * radian_to_degree_constant;

    ui->xValueLabel->setText(QString("%1").arg(x));
    ui->yValueLabel->setText(QString("%1").arg(y));
    ui->phiValueLabel->setText(QString("%1").arg(theta_degree));
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
        QMetaObject::invokeMethod(service_client_, [a, b, this]() { service_client_->sendRequest(a, b); });

        statusLabelPtr_->setText("Changing Mapping Mode...\nSending \"mapping\" mode...\n");
        ui->mappingBtn->setStyleSheet("background-color: green;");
        ui->navigationBtn->setStyleSheet("background-color: none;");
        ui->remappingBtn->setStyleSheet("background-color: none;");

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
        QMetaObject::invokeMethod(service_client_, [a, b, this]() { service_client_->sendRequest(a, b); });

        statusLabelPtr_->setText("Changing Mapping Mode...\nSending \"navi\" mode...\n");
        ui->mappingBtn->setStyleSheet("background-color: none;");
        ui->navigationBtn->setStyleSheet("background-color: green;");
        ui->remappingBtn->setStyleSheet("background-color: none;");

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
        QMetaObject::invokeMethod(service_client_, [a, b, this]() { service_client_->sendRequest(a, b); });
        
        statusLabelPtr_->setText("Changing Mapping Mode...\nSending \"remapping\" mode...\n");
        ui->mappingBtn->setStyleSheet("background-color: none;");
        ui->navigationBtn->setStyleSheet("background-color: none;");
        ui->remappingBtn->setStyleSheet("background-color: green;");

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
    QString currentText = statusLabelPtr_->text();

    if (service_status == -1) {
        statusLabelPtr_->setText(currentText + "\n" + "Error: Service not available or failed.\nReceiving not ok.\n");
    } else {
        //statusLabelPtr_->setText(QString("Result: %1").arg(sum));
        statusLabelPtr_->setText(currentText + "\n" + "Receiving response..\nIts ok.\n");
    }
    saveMapBtnPtr_->setStyleSheet("background-color: none;"); 
    //openMapBtnPtr_->setStyleSheet("background-color: none;");
    //selectMapBtnPtr_->setStyleSheet("background-color: none;");

    hideBusyDialog();
    setButtonsEnabled(true);
}



ServiceClient::ServiceClient() {
    //rclcpp::init(0, nullptr);
    node = rclcpp::Node::make_shared("qt_service_client");
    client = node->create_client<rom_interfaces::srv::WhichMaps>("/which_maps");

    // Start a separate thread for the ROS 2 spinning
    rosThread = std::thread(&ServiceClient::spin, this);
}

/// SERVICE CLIENT
ServiceClient::~ServiceClient() {
    rclcpp::shutdown();
    if (rosThread.joinable()) {
        rosThread.join();
    }
}

void ServiceClient::spin() {
    rclcpp::spin(node);
}


void ServiceClient::sendRequest(const std::string& request_string, const std::string& optional_param = "") {
    auto request = std::make_shared<rom_interfaces::srv::WhichMaps::Request>();

    request->request_string = request_string;

    if (request_string == "save_map") {
        request->map_name_to_save = optional_param;
    } 
    else if (request_string == "select_map") {
        request->map_name_to_select = optional_param;
    }
    

    if (!client->wait_for_service(std::chrono::seconds(5))) {
        emit responseReceived(-1); // Error: service not available
        return;
    }

    auto future = client->async_send_request(request);
    future.wait();

    try {
        auto response = future.get();
        emit responseReceived(response->status);
    } catch (const std::exception &e) {
        emit responseReceived(-1); // Error: response failure
    }
}


void MainWindow::saveMapClicked()
{
    showBusyDialog();
    setButtonsEnabled(false);
    /*
    statusLabelPtr_->setText("\nမြေပုံအား default_map အမည်ဖြင့်သိမ်းဆည်းခြင်းနေပါသည်။ ... \n");

    saveMapBtnPtr_->setStyleSheet("background-color: green;");

    std::string a = "save_map";
    std::string b = "office";
    QMetaObject::invokeMethod(service_client_, [a, b, this]() { service_client_->sendRequest(a, b); });

    statusLabelPtr_->setText("Sending save map request...\nString: \"save_map\" \nmap_name: \"default_map\"\n");
    */

    //-----------------------------------------------------------------------------------------------------------
    // Prompt the user to enter a map name
    bool ok;

    QRegExp regex("^[^_]+$"); // Regular expression: no underscores allowed

    QString mapName = QInputDialog::getText(this, tr("Save Map"),
                                            tr("Enter map name:"), QLineEdit::Normal,
                                            tr("default"), &ok);

    if (ok && !mapName.isEmpty()) {
        // Update the status label and send the save map request
        statusLabelPtr_->setText(tr("\nမြေပုံအား '%1' အမည်ဖြင့်သိမ်းဆည်းခြင်းနေပါသည်။ ... \n").arg(mapName));
        saveMapBtnPtr_->setStyleSheet("background-color: green;");

        std::string a = "save_map";
        std::string b = mapName.toStdString();
        QMetaObject::invokeMethod(service_client_, [a, b, this]() { service_client_->sendRequest(a, b); });

        statusLabelPtr_->setText(tr("Sending save map request...\nString: \"save_map\"\nmap_name: \"%1\"\n").arg(mapName));
    } 
    else 
    {
        statusLabelPtr_->setText("\nSave map canceled by user.\n");

        hideBusyDialog();
        setButtonsEnabled(true);
    }
    //-----------------------------------------------------------------------------------------------------------
}


void MainWindow::openMapClicked()
{
    statusLabelPtr_->setText("\nimplement မလုပ်ရသေးပါ။\n");
}


void MainWindow::selectMapClicked()
{
    statusLabelPtr_->setText("\nimplement မလုပ်ရသေးပါ။\n");
}


void MainWindow::labelEditForSetForward()
{
    statusLabelPtr_->setText("\nConstant Speed with 10Hz.\n\nForward:\n     Linear velocity    : 0.4   m/s\n     Angular velocity : 0.0 rad/s\n");
}

void MainWindow::labelEditForSetRight()
{
    statusLabelPtr_->setText("\nConstant Speed with 10Hz.\n\nRight:\n     Linear velocity    : 0.0   m/s\n     Angular velocity : 0.4   rad/s\n");
}

void MainWindow::labelEditForSetLeft()
{
    statusLabelPtr_->setText("\nConstant Speed with 10Hz.\n\nLeft:\n     Linear velocity    : 0.0   m/s\n     Angular velocity : -0.4 rad/s\n");
}

void MainWindow::labelEditForSetStop()
{
    statusLabelPtr_->setText("\nConstant Speed with 10Hz.\n\nStop:\n     Linear velocity    : 0.0   m/s\n     Angular velocity : 0.0   rad/s\n");
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
        busyDialog_->move(1300, 550); 
        //busyDialog_->hide();
    }

    // Show the dialog
    busyDialog_->show();
}

