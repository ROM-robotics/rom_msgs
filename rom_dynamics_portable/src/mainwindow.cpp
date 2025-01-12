#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <cmath> // For atan2 and M_PI

extern void shutdown_thread();

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent)
{
    setWindowTitle("ROM Dynamics Company's Robot Suite");
    ui = std::make_shared<Ui::MainWindow>();
    ui->setupUi(this);

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

    current_mode_ = "navi";

    statusLabelPtr_->setText("App အား အသုံးပြုဖို့အတွက် အောက်ပါ ROS2 humble package နှစ်ခုကို install လုပ်ပါ။။\n      - rom_interfaces\n      - which_maps\n\n $ ros2 run which_maps which_maps_server\n # map save ရန် lifecycle လို/မလို စစ်ဆေးပါ။\n");
}


MainWindow::~MainWindow()
{
    rclcpp::shutdown();

    rosThread->quit();
    rosThread->wait();

    delete service_client_;
    delete rosThread;
}



void MainWindow::DisplaySubscription(const QString &log)
{
    //statusLabelPtr_->setText("hello"); there is no publisher
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
}



ServiceClient::ServiceClient() {
    rclcpp::init(0, nullptr);
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
    } else if (request_string == "select_map") {
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
    statusLabelPtr_->setText("\nမြေပုံအား default_map အမည်ဖြင့်သိမ်းဆည်းခြင်းနေပါသည်။ ... \n");

    saveMapBtnPtr_->setStyleSheet("background-color: green;");

    std::string a = "save_map";
    std::string b = "default_map";
    QMetaObject::invokeMethod(service_client_, [a, b, this]() { service_client_->sendRequest(a, b); });

    statusLabelPtr_->setText("Sending save map request...\nString: \"save_map\" \nmap_name: \"default_map\"\n");
}


void MainWindow::openMapClicked()
{
    statusLabelPtr_->setText("\nimplement မလုပ်ရသေးပါ။\n");
}


void MainWindow::selectMapClicked()
{
    statusLabelPtr_->setText("\nimplement မလုပ်ရသေးပါ။\n");
}


// void ServiceClient::sendRequest(int a, int b) 
// {
//     auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
//     request->a = a;
//     request->b = b;

//     if (!client->wait_for_service(std::chrono::seconds(5))) {
//         emit responseReceived(-1); // Error: service not available
//         return;
//     }

//     auto future = client->async_send_request(request);
//     future.wait();

//     try {
//         auto response = future.get();
//         emit responseReceived(response->sum);
//     } catch (const std::exception &e) {
//         emit responseReceived(-1); // Error: response failure
//     }
// }
