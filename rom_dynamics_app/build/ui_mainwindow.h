/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGraphicsView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QPushButton *mappingBtn;
    QPushButton *navigationBtn;
    QPushButton *remappingBtn;
    QLabel *label;
    QPushButton *openMapBtn;
    QPushButton *selectMapBtn;
    QPushButton *saveMapBtn;
    QLabel *xLabel;
    QLabel *yLabel;
    QLabel *yValueLabel;
    QLabel *xValueLabel;
    QLabel *statusLabel;
    QPushButton *shutdownBtn;
    QPushButton *btnEstop;
    QPushButton *btnStop;
    QPushButton *btnForward;
    QPushButton *btnLeft;
    QPushButton *btnRight;
    QLabel *phiLabel;
    QLabel *phiValueLabel;
    QPushButton *goBtn;
    QLabel *xSendLabel;
    QLabel *ySendLabel;
    QLabel *zSendLabel;
    QLabel *sendGoalLabel;
    QLabel *companyLabel;
    QPushButton *cancelBtn;
    QPushButton *rthBtn;
    QSpinBox *xspinBox;
    QSpinBox *zspinBox;
    QSpinBox *yspinBox;
    QWidget *verticalLayoutWidget;
    QVBoxLayout *myVerticalLayout;
    QGraphicsView *graphicsView;
    QPushButton *eraserBtn;
    QPushButton *addWaypointBtn;
    QPushButton *normalBtn;
    QPushButton *zoomBtn;
    QPushButton *addWallBtn;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1600, 1200);
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(MainWindow->sizePolicy().hasHeightForWidth());
        MainWindow->setSizePolicy(sizePolicy);
        MainWindow->setMinimumSize(QSize(1440, 900));
        MainWindow->setMaximumSize(QSize(1600, 1200));
        QFont font;
        font.setPointSize(12);
        font.setBold(true);
        font.setWeight(75);
        MainWindow->setFont(font);
        MainWindow->setToolButtonStyle(Qt::ToolButtonIconOnly);
        MainWindow->setTabShape(QTabWidget::Rounded);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        mappingBtn = new QPushButton(centralwidget);
        mappingBtn->setObjectName(QString::fromUtf8("mappingBtn"));
        mappingBtn->setGeometry(QRect(30, 27, 171, 51));
        QFont font1;
        font1.setPointSize(10);
        font1.setBold(true);
        font1.setWeight(75);
        mappingBtn->setFont(font1);
        mappingBtn->setStyleSheet(QString::fromUtf8("background-color: white;"));
        navigationBtn = new QPushButton(centralwidget);
        navigationBtn->setObjectName(QString::fromUtf8("navigationBtn"));
        navigationBtn->setGeometry(QRect(220, 27, 191, 51));
        navigationBtn->setFont(font1);
        navigationBtn->setStyleSheet(QString::fromUtf8("background-color: green;"));
        remappingBtn = new QPushButton(centralwidget);
        remappingBtn->setObjectName(QString::fromUtf8("remappingBtn"));
        remappingBtn->setGeometry(QRect(430, 27, 181, 51));
        remappingBtn->setFont(font1);
        remappingBtn->setStyleSheet(QString::fromUtf8("background-color: white;"));
        label = new QLabel(centralwidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(768, 740, 271, 34));
        label->setFont(font1);
        label->setStyleSheet(QString::fromUtf8("background-color: none;\n"
"color: white;"));
        label->setText(QString::fromUtf8("Robot Position"));
        openMapBtn = new QPushButton(centralwidget);
        openMapBtn->setObjectName(QString::fromUtf8("openMapBtn"));
        openMapBtn->setGeometry(QRect(630, 27, 161, 51));
        openMapBtn->setFont(font1);
        openMapBtn->setStyleSheet(QString::fromUtf8("background-color: white;"));
        selectMapBtn = new QPushButton(centralwidget);
        selectMapBtn->setObjectName(QString::fromUtf8("selectMapBtn"));
        selectMapBtn->setGeometry(QRect(810, 27, 161, 51));
        selectMapBtn->setFont(font1);
        selectMapBtn->setStyleSheet(QString::fromUtf8("background-color: white;"));
        saveMapBtn = new QPushButton(centralwidget);
        saveMapBtn->setObjectName(QString::fromUtf8("saveMapBtn"));
        saveMapBtn->setGeometry(QRect(990, 27, 161, 51));
        saveMapBtn->setFont(font1);
        saveMapBtn->setStyleSheet(QString::fromUtf8("background-color: white;"));
        xLabel = new QLabel(centralwidget);
        xLabel->setObjectName(QString::fromUtf8("xLabel"));
        xLabel->setGeometry(QRect(780, 800, 129, 34));
        xLabel->setFont(font);
        xLabel->setStyleSheet(QString::fromUtf8("background-color: none;\n"
"color: white;"));
        yLabel = new QLabel(centralwidget);
        yLabel->setObjectName(QString::fromUtf8("yLabel"));
        yLabel->setGeometry(QRect(780, 880, 129, 34));
        yLabel->setFont(font);
        yLabel->setStyleSheet(QString::fromUtf8("background-color: none;\n"
"color: white;"));
        yValueLabel = new QLabel(centralwidget);
        yValueLabel->setObjectName(QString::fromUtf8("yValueLabel"));
        yValueLabel->setGeometry(QRect(828, 880, 91, 34));
        QFont font2;
        font2.setPointSize(9);
        font2.setBold(true);
        font2.setItalic(true);
        font2.setWeight(75);
        yValueLabel->setFont(font2);
        yValueLabel->setStyleSheet(QString::fromUtf8("background-color: none;\n"
"color: orange;"));
        xValueLabel = new QLabel(centralwidget);
        xValueLabel->setObjectName(QString::fromUtf8("xValueLabel"));
        xValueLabel->setGeometry(QRect(828, 800, 91, 34));
        QFont font3;
        font3.setPointSize(9);
        font3.setBold(true);
        font3.setItalic(true);
        font3.setUnderline(false);
        font3.setWeight(75);
        xValueLabel->setFont(font3);
        xValueLabel->setStyleSheet(QString::fromUtf8("background-color: none;\n"
"color: orange;"));
        statusLabel = new QLabel(centralwidget);
        statusLabel->setObjectName(QString::fromUtf8("statusLabel"));
        statusLabel->setGeometry(QRect(30, 860, 541, 291));
        QFont font4;
        font4.setPointSize(9);
        font4.setBold(false);
        font4.setWeight(50);
        statusLabel->setFont(font4);
        statusLabel->setStyleSheet(QString::fromUtf8("background-color: #2E3436 ;\n"
"color: rgb(246, 245, 244);\n"
"border: 1px solid gray;\n"
"padding-top: 10px;\n"
"padding-left: 20px;"));
        statusLabel->setMargin(10);
        shutdownBtn = new QPushButton(centralwidget);
        shutdownBtn->setObjectName(QString::fromUtf8("shutdownBtn"));
        shutdownBtn->setGeometry(QRect(1550, 0, 51, 51));
        QFont font5;
        font5.setBold(true);
        font5.setWeight(75);
        shutdownBtn->setFont(font5);
        shutdownBtn->setStyleSheet(QString::fromUtf8("border-radius: 25px;\n"
"border: 1px solid gray;\n"
"background-color: white;\n"
"color: red;\n"
"font-size: 36px;"));
        shutdownBtn->setIconSize(QSize(16, 16));
        btnEstop = new QPushButton(centralwidget);
        btnEstop->setObjectName(QString::fromUtf8("btnEstop"));
        btnEstop->setGeometry(QRect(590, 860, 101, 101));
        btnEstop->setFont(font1);
        btnStop = new QPushButton(centralwidget);
        btnStop->setObjectName(QString::fromUtf8("btnStop"));
        btnStop->setGeometry(QRect(1281, 860, 101, 101));
        QFont font6;
        font6.setPointSize(20);
        font6.setBold(true);
        font6.setWeight(75);
        btnStop->setFont(font6);
        btnStop->setStyleSheet(QString::fromUtf8("color: white;"));
        btnForward = new QPushButton(centralwidget);
        btnForward->setObjectName(QString::fromUtf8("btnForward"));
        btnForward->setGeometry(QRect(1280, 734, 101, 101));
        btnForward->setFont(font6);
        btnForward->setStyleSheet(QString::fromUtf8("color: white;"));
        btnLeft = new QPushButton(centralwidget);
        btnLeft->setObjectName(QString::fromUtf8("btnLeft"));
        btnLeft->setGeometry(QRect(1141, 860, 101, 101));
        btnLeft->setFont(font6);
        btnLeft->setStyleSheet(QString::fromUtf8("color: white;"));
        btnRight = new QPushButton(centralwidget);
        btnRight->setObjectName(QString::fromUtf8("btnRight"));
        btnRight->setGeometry(QRect(1421, 860, 101, 101));
        btnRight->setFont(font6);
        btnRight->setStyleSheet(QString::fromUtf8("color: white;"));
        phiLabel = new QLabel(centralwidget);
        phiLabel->setObjectName(QString::fromUtf8("phiLabel"));
        phiLabel->setGeometry(QRect(773, 959, 129, 34));
        phiLabel->setFont(font);
        phiLabel->setStyleSheet(QString::fromUtf8("background-color: none;\n"
"color: white;"));
        phiValueLabel = new QLabel(centralwidget);
        phiValueLabel->setObjectName(QString::fromUtf8("phiValueLabel"));
        phiValueLabel->setGeometry(QRect(821, 959, 101, 34));
        phiValueLabel->setFont(font2);
        phiValueLabel->setStyleSheet(QString::fromUtf8("background-color: none;\n"
"color: orange;"));
        goBtn = new QPushButton(centralwidget);
        goBtn->setObjectName(QString::fromUtf8("goBtn"));
        goBtn->setGeometry(QRect(1060, 1060, 71, 71));
        goBtn->setFont(font5);
        goBtn->setStyleSheet(QString::fromUtf8("border-radius: 35px;\n"
"border: 1px solid gray;\n"
"border: 2px solid #1B1F4F;\n"
"color: green;\n"
"font-size: 35px;"));
        goBtn->setIconSize(QSize(16, 16));
        xSendLabel = new QLabel(centralwidget);
        xSendLabel->setObjectName(QString::fromUtf8("xSendLabel"));
        xSendLabel->setGeometry(QRect(590, 1050, 129, 34));
        QFont font7;
        font7.setPointSize(7);
        font7.setBold(false);
        font7.setWeight(50);
        xSendLabel->setFont(font7);
        xSendLabel->setStyleSheet(QString::fromUtf8("background-color: none;\n"
"color: #e8e7e6;"));
        ySendLabel = new QLabel(centralwidget);
        ySendLabel->setObjectName(QString::fromUtf8("ySendLabel"));
        ySendLabel->setGeometry(QRect(750, 1050, 129, 34));
        ySendLabel->setFont(font7);
        ySendLabel->setStyleSheet(QString::fromUtf8("background-color: none;\n"
"color: #e8e7e6;"));
        zSendLabel = new QLabel(centralwidget);
        zSendLabel->setObjectName(QString::fromUtf8("zSendLabel"));
        zSendLabel->setGeometry(QRect(910, 1050, 129, 34));
        zSendLabel->setFont(font7);
        zSendLabel->setStyleSheet(QString::fromUtf8("background-color: none;\n"
"color: #e8e7e6;"));
        sendGoalLabel = new QLabel(centralwidget);
        sendGoalLabel->setObjectName(QString::fromUtf8("sendGoalLabel"));
        sendGoalLabel->setGeometry(QRect(600, 1020, 271, 34));
        QFont font8;
        font8.setPointSize(11);
        font8.setBold(false);
        font8.setWeight(50);
        sendGoalLabel->setFont(font8);
        sendGoalLabel->setStyleSheet(QString::fromUtf8("background-color: none;\n"
"color: #e8e7e6;"));
        sendGoalLabel->setText(QString::fromUtf8("Goal"));
        companyLabel = new QLabel(centralwidget);
        companyLabel->setObjectName(QString::fromUtf8("companyLabel"));
        companyLabel->setGeometry(QRect(9, 1166, 901, 34));
        companyLabel->setFont(font7);
        companyLabel->setStyleSheet(QString::fromUtf8("background-color: none;\n"
"color: white;"));
        companyLabel->setText(QString::fromUtf8("\302\251 2015-2025 ROM Dynamics Robotics Company Limited. All rights reserved."));
        cancelBtn = new QPushButton(centralwidget);
        cancelBtn->setObjectName(QString::fromUtf8("cancelBtn"));
        cancelBtn->setGeometry(QRect(1420, 1020, 141, 141));
        cancelBtn->setFont(font5);
        cancelBtn->setStyleSheet(QString::fromUtf8("border-radius: 70px;\n"
"border: 2px solid #FF8C00;\n"
"background-color: white;\n"
"color: red;\n"
"font-size: 26px;\n"
"font-weight: bold;"));
        cancelBtn->setIconSize(QSize(16, 16));
        rthBtn = new QPushButton(centralwidget);
        rthBtn->setObjectName(QString::fromUtf8("rthBtn"));
        rthBtn->setGeometry(QRect(1250, 1040, 101, 101));
        rthBtn->setFont(font5);
        rthBtn->setStyleSheet(QString::fromUtf8("border-radius: 50px;\n"
"border: 2px solid #3E8E41;\n"
"background-color: white;\n"
"color: #0f9ff2;\n"
"font-size: 35px;"));
        rthBtn->setIconSize(QSize(16, 16));
        xspinBox = new QSpinBox(centralwidget);
        xspinBox->setObjectName(QString::fromUtf8("xspinBox"));
        xspinBox->setGeometry(QRect(590, 1090, 101, 49));
        xspinBox->setStyleSheet(QString::fromUtf8("color: white;"));
        zspinBox = new QSpinBox(centralwidget);
        zspinBox->setObjectName(QString::fromUtf8("zspinBox"));
        zspinBox->setGeometry(QRect(920, 1090, 101, 49));
        zspinBox->setStyleSheet(QString::fromUtf8("color: white;"));
        yspinBox = new QSpinBox(centralwidget);
        yspinBox->setObjectName(QString::fromUtf8("yspinBox"));
        yspinBox->setGeometry(QRect(750, 1090, 101, 49));
        yspinBox->setStyleSheet(QString::fromUtf8("color: white;"));
        verticalLayoutWidget = new QWidget(centralwidget);
        verticalLayoutWidget->setObjectName(QString::fromUtf8("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(30, 100, 731, 601));
        myVerticalLayout = new QVBoxLayout(verticalLayoutWidget);
        myVerticalLayout->setObjectName(QString::fromUtf8("myVerticalLayout"));
        myVerticalLayout->setContentsMargins(0, 0, 0, 0);
        graphicsView = new QGraphicsView(verticalLayoutWidget);
        graphicsView->setObjectName(QString::fromUtf8("graphicsView"));
        QBrush brush(QColor(143, 240, 164, 255));
        brush.setStyle(Qt::NoBrush);
        graphicsView->setBackgroundBrush(brush);

        myVerticalLayout->addWidget(graphicsView);

        eraserBtn = new QPushButton(centralwidget);
        eraserBtn->setObjectName(QString::fromUtf8("eraserBtn"));
        eraserBtn->setGeometry(QRect(310, 734, 101, 100));
        eraserBtn->setFont(font6);
        eraserBtn->setStyleSheet(QString::fromUtf8("color: white;\n"
"background-image: url(/home/mr_robot/Desktop/Git/rom_msgs/rom_dynamics_app/ico/eraser.png);\n"
"background-repeat: no-repeat; \n"
"background-position: center; \n"
"border: 2px solid #979ba1;"));
        addWaypointBtn = new QPushButton(centralwidget);
        addWaypointBtn->setObjectName(QString::fromUtf8("addWaypointBtn"));
        addWaypointBtn->setGeometry(QRect(30, 734, 101, 100));
        addWaypointBtn->setFont(font6);
        addWaypointBtn->setStyleSheet(QString::fromUtf8("color: white;\n"
"background-image: url(/home/mr_robot/Desktop/Git/rom_msgs/rom_dynamics_app/ico/waypoint.png);\n"
"background-repeat: no-repeat; \n"
"background-position: center; \n"
"border: 2px solid #979ba1;"));
        normalBtn = new QPushButton(centralwidget);
        normalBtn->setObjectName(QString::fromUtf8("normalBtn"));
        normalBtn->setGeometry(QRect(590, 734, 101, 100));
        normalBtn->setFont(font6);
        normalBtn->setStyleSheet(QString::fromUtf8("color: white;\n"
"background-image: url(/home/mr_robot/Desktop/Git/rom_msgs/rom_dynamics_app/ico/normal.png);\n"
"background-repeat: no-repeat; \n"
"background-position: center; \n"
"border: 3px solid white;"));
        zoomBtn = new QPushButton(centralwidget);
        zoomBtn->setObjectName(QString::fromUtf8("zoomBtn"));
        zoomBtn->setGeometry(QRect(450, 734, 101, 100));
        zoomBtn->setFont(font6);
        zoomBtn->setStyleSheet(QString::fromUtf8("color: white;\n"
"background-image: url(/home/mr_robot/Desktop/Git/rom_msgs/rom_dynamics_app/ico/zoom.png);\n"
"background-repeat: no-repeat; \n"
"background-position: center; \n"
"border: 2px solid #979ba1;"));
        addWallBtn = new QPushButton(centralwidget);
        addWallBtn->setObjectName(QString::fromUtf8("addWallBtn"));
        addWallBtn->setGeometry(QRect(170, 734, 101, 100));
        addWallBtn->setFont(font6);
        addWallBtn->setStyleSheet(QString::fromUtf8("color: white;\n"
"background-image: url(/home/mr_robot/Desktop/Git/rom_msgs/rom_dynamics_app/ico/wall.png);\n"
"background-repeat: no-repeat; \n"
"background-position: center; \n"
"border: 2px solid #979ba1;"));
        MainWindow->setCentralWidget(centralwidget);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "ROM Dynamics's Robot Suite", nullptr));
        mappingBtn->setText(QCoreApplication::translate("MainWindow", "Mapping", nullptr));
        navigationBtn->setText(QCoreApplication::translate("MainWindow", "Navigation", nullptr));
        remappingBtn->setText(QCoreApplication::translate("MainWindow", "Remapping", nullptr));
        openMapBtn->setText(QCoreApplication::translate("MainWindow", "Open Maps", nullptr));
        selectMapBtn->setText(QCoreApplication::translate("MainWindow", "Select Map", nullptr));
        saveMapBtn->setText(QCoreApplication::translate("MainWindow", "Save Map", nullptr));
        xLabel->setText(QCoreApplication::translate("MainWindow", "x :", nullptr));
        yLabel->setText(QCoreApplication::translate("MainWindow", "y :", nullptr));
        yValueLabel->setText(QCoreApplication::translate("MainWindow", "sync ...", nullptr));
        xValueLabel->setText(QCoreApplication::translate("MainWindow", "sync ...", nullptr));
        statusLabel->setText(QString());
        shutdownBtn->setText(QCoreApplication::translate("MainWindow", "X", nullptr));
        btnEstop->setText(QCoreApplication::translate("MainWindow", "E-Stop", nullptr));
        btnStop->setText(QCoreApplication::translate("MainWindow", "\342\230\267", nullptr));
        btnForward->setText(QCoreApplication::translate("MainWindow", "\342\206\221", nullptr));
        btnLeft->setText(QCoreApplication::translate("MainWindow", "\342\237\262", nullptr));
        btnRight->setText(QCoreApplication::translate("MainWindow", "\342\237\263", nullptr));
        phiLabel->setText(QCoreApplication::translate("MainWindow", "\317\225 :", nullptr));
        phiValueLabel->setText(QCoreApplication::translate("MainWindow", "sync ...", nullptr));
        goBtn->setText(QCoreApplication::translate("MainWindow", "Go", nullptr));
        xSendLabel->setText(QCoreApplication::translate("MainWindow", "x ( feet )", nullptr));
        ySendLabel->setText(QCoreApplication::translate("MainWindow", "y ( feet )", nullptr));
        zSendLabel->setText(QCoreApplication::translate("MainWindow", "z ( degree )", nullptr));
        cancelBtn->setText(QCoreApplication::translate("MainWindow", "Waypoints", nullptr));
        rthBtn->setText(QCoreApplication::translate("MainWindow", "RTH", nullptr));
        eraserBtn->setText(QString());
        addWaypointBtn->setText(QString());
        normalBtn->setText(QString());
        zoomBtn->setText(QString());
        addWallBtn->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
