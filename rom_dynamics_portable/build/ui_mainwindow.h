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
#include <QtWidgets/QComboBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
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
    QComboBox *xCombo;
    QComboBox *yCombo;
    QComboBox *zCombo;
    QLabel *xSendLabel;
    QLabel *ySendLabel;
    QLabel *zSendLabel;
    QLabel *sendGoalLabel;
    QLabel *companyLabel;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1700, 700);
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(MainWindow->sizePolicy().hasHeightForWidth());
        MainWindow->setSizePolicy(sizePolicy);
        MainWindow->setMinimumSize(QSize(1700, 700));
        MainWindow->setMaximumSize(QSize(1700, 700));
        QFont font;
        font.setPointSize(12);
        font.setBold(true);
        MainWindow->setFont(font);
        MainWindow->setToolButtonStyle(Qt::ToolButtonIconOnly);
        MainWindow->setTabShape(QTabWidget::Rounded);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        mappingBtn = new QPushButton(centralwidget);
        mappingBtn->setObjectName(QString::fromUtf8("mappingBtn"));
        mappingBtn->setGeometry(QRect(70, 80, 261, 71));
        navigationBtn = new QPushButton(centralwidget);
        navigationBtn->setObjectName(QString::fromUtf8("navigationBtn"));
        navigationBtn->setGeometry(QRect(390, 80, 261, 71));
        navigationBtn->setStyleSheet(QString::fromUtf8("background-color: green;"));
        remappingBtn = new QPushButton(centralwidget);
        remappingBtn->setObjectName(QString::fromUtf8("remappingBtn"));
        remappingBtn->setGeometry(QRect(710, 80, 261, 71));
        label = new QLabel(centralwidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(78, 180, 271, 34));
        label->setFont(font);
        label->setStyleSheet(QString::fromUtf8("background-color: none;"));
        label->setText(QString::fromUtf8("Robot Position"));
        openMapBtn = new QPushButton(centralwidget);
        openMapBtn->setObjectName(QString::fromUtf8("openMapBtn"));
        openMapBtn->setGeometry(QRect(1030, 80, 201, 71));
        selectMapBtn = new QPushButton(centralwidget);
        selectMapBtn->setObjectName(QString::fromUtf8("selectMapBtn"));
        selectMapBtn->setGeometry(QRect(1250, 80, 191, 71));
        saveMapBtn = new QPushButton(centralwidget);
        saveMapBtn->setObjectName(QString::fromUtf8("saveMapBtn"));
        saveMapBtn->setGeometry(QRect(1460, 80, 191, 71));
        xLabel = new QLabel(centralwidget);
        xLabel->setObjectName(QString::fromUtf8("xLabel"));
        xLabel->setGeometry(QRect(100, 240, 129, 34));
        xLabel->setFont(font);
        xLabel->setStyleSheet(QString::fromUtf8("background-color: none;"));
        yLabel = new QLabel(centralwidget);
        yLabel->setObjectName(QString::fromUtf8("yLabel"));
        yLabel->setGeometry(QRect(100, 320, 129, 34));
        yLabel->setFont(font);
        yLabel->setStyleSheet(QString::fromUtf8("background-color: none;"));
        yValueLabel = new QLabel(centralwidget);
        yValueLabel->setObjectName(QString::fromUtf8("yValueLabel"));
        yValueLabel->setGeometry(QRect(158, 320, 271, 34));
        QFont font1;
        font1.setPointSize(9);
        font1.setBold(true);
        font1.setItalic(true);
        yValueLabel->setFont(font1);
        yValueLabel->setStyleSheet(QString::fromUtf8("background-color: none;"));
        xValueLabel = new QLabel(centralwidget);
        xValueLabel->setObjectName(QString::fromUtf8("xValueLabel"));
        xValueLabel->setGeometry(QRect(158, 240, 271, 34));
        QFont font2;
        font2.setPointSize(9);
        font2.setBold(true);
        font2.setItalic(true);
        font2.setUnderline(false);
        xValueLabel->setFont(font2);
        xValueLabel->setStyleSheet(QString::fromUtf8("background-color: none;"));
        statusLabel = new QLabel(centralwidget);
        statusLabel->setObjectName(QString::fromUtf8("statusLabel"));
        statusLabel->setGeometry(QRect(390, 200, 691, 291));
        QFont font3;
        font3.setPointSize(10);
        font3.setBold(false);
        statusLabel->setFont(font3);
        statusLabel->setStyleSheet(QString::fromUtf8("background-color: #000000\n"
";\n"
"color: rgb(246, 245, 244);"));
        statusLabel->setMargin(10);
        shutdownBtn = new QPushButton(centralwidget);
        shutdownBtn->setObjectName(QString::fromUtf8("shutdownBtn"));
        shutdownBtn->setGeometry(QRect(1646, 4, 51, 51));
        QFont font4;
        font4.setBold(true);
        shutdownBtn->setFont(font4);
        shutdownBtn->setStyleSheet(QString::fromUtf8("border-radius: 25px;\n"
"border: 1px solid gray;\n"
"background-color: white;\n"
"color: red;\n"
"font-size: 36px;"));
        shutdownBtn->setIconSize(QSize(16, 16));
        btnEstop = new QPushButton(centralwidget);
        btnEstop->setObjectName(QString::fromUtf8("btnEstop"));
        btnEstop->setGeometry(QRect(78, 530, 191, 91));
        btnStop = new QPushButton(centralwidget);
        btnStop->setObjectName(QString::fromUtf8("btnStop"));
        btnStop->setGeometry(QRect(1360, 520, 101, 101));
        QFont font5;
        font5.setPointSize(20);
        font5.setBold(true);
        btnStop->setFont(font5);
        btnForward = new QPushButton(centralwidget);
        btnForward->setObjectName(QString::fromUtf8("btnForward"));
        btnForward->setGeometry(QRect(1360, 210, 101, 101));
        btnForward->setFont(font5);
        btnLeft = new QPushButton(centralwidget);
        btnLeft->setObjectName(QString::fromUtf8("btnLeft"));
        btnLeft->setGeometry(QRect(1170, 360, 101, 101));
        btnLeft->setFont(font5);
        btnRight = new QPushButton(centralwidget);
        btnRight->setObjectName(QString::fromUtf8("btnRight"));
        btnRight->setGeometry(QRect(1540, 360, 101, 101));
        btnRight->setFont(font5);
        phiLabel = new QLabel(centralwidget);
        phiLabel->setObjectName(QString::fromUtf8("phiLabel"));
        phiLabel->setGeometry(QRect(98, 400, 129, 34));
        phiLabel->setFont(font);
        phiLabel->setStyleSheet(QString::fromUtf8("background-color: none;"));
        phiValueLabel = new QLabel(centralwidget);
        phiValueLabel->setObjectName(QString::fromUtf8("phiValueLabel"));
        phiValueLabel->setGeometry(QRect(156, 400, 271, 34));
        phiValueLabel->setFont(font1);
        phiValueLabel->setStyleSheet(QString::fromUtf8("background-color: none;"));
        goBtn = new QPushButton(centralwidget);
        goBtn->setObjectName(QString::fromUtf8("goBtn"));
        goBtn->setGeometry(QRect(920, 540, 111, 111));
        goBtn->setFont(font4);
        goBtn->setStyleSheet(QString::fromUtf8("border-radius: 55px;\n"
"border: 1px solid gray;\n"
"background-color: white;\n"
"color: green;\n"
"font-size: 56px;"));
        goBtn->setIconSize(QSize(16, 16));
        xCombo = new QComboBox(centralwidget);
        xCombo->setObjectName(QString::fromUtf8("xCombo"));
        xCombo->setGeometry(QRect(390, 580, 111, 41));
        xCombo->setEditable(false);
        yCombo = new QComboBox(centralwidget);
        yCombo->setObjectName(QString::fromUtf8("yCombo"));
        yCombo->setGeometry(QRect(550, 580, 111, 41));
        zCombo = new QComboBox(centralwidget);
        zCombo->setObjectName(QString::fromUtf8("zCombo"));
        zCombo->setGeometry(QRect(720, 580, 111, 41));
        xSendLabel = new QLabel(centralwidget);
        xSendLabel->setObjectName(QString::fromUtf8("xSendLabel"));
        xSendLabel->setGeometry(QRect(390, 550, 129, 34));
        QFont font6;
        font6.setPointSize(7);
        font6.setBold(false);
        xSendLabel->setFont(font6);
        xSendLabel->setStyleSheet(QString::fromUtf8("background-color: none;"));
        ySendLabel = new QLabel(centralwidget);
        ySendLabel->setObjectName(QString::fromUtf8("ySendLabel"));
        ySendLabel->setGeometry(QRect(550, 550, 129, 34));
        ySendLabel->setFont(font6);
        ySendLabel->setStyleSheet(QString::fromUtf8("background-color: none;"));
        zSendLabel = new QLabel(centralwidget);
        zSendLabel->setObjectName(QString::fromUtf8("zSendLabel"));
        zSendLabel->setGeometry(QRect(720, 550, 129, 34));
        zSendLabel->setFont(font6);
        zSendLabel->setStyleSheet(QString::fromUtf8("background-color: none;"));
        sendGoalLabel = new QLabel(centralwidget);
        sendGoalLabel->setObjectName(QString::fromUtf8("sendGoalLabel"));
        sendGoalLabel->setGeometry(QRect(390, 510, 271, 34));
        QFont font7;
        font7.setPointSize(11);
        font7.setBold(false);
        sendGoalLabel->setFont(font7);
        sendGoalLabel->setStyleSheet(QString::fromUtf8("background-color: none;"));
        sendGoalLabel->setText(QString::fromUtf8("Goal"));
        companyLabel = new QLabel(centralwidget);
        companyLabel->setObjectName(QString::fromUtf8("companyLabel"));
        companyLabel->setGeometry(QRect(10, 660, 901, 34));
        companyLabel->setFont(font6);
        companyLabel->setStyleSheet(QString::fromUtf8("background-color: none;"));
        companyLabel->setText(QString::fromUtf8("\302\251 2015-2025 ROM Dynamics Robotics Company Limited. All rights reserved."));
        MainWindow->setCentralWidget(centralwidget);

        retranslateUi(MainWindow);

        xCombo->setCurrentIndex(-1);


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
        btnStop->setText(QCoreApplication::translate("MainWindow", "\342\233\224", nullptr));
        btnForward->setText(QCoreApplication::translate("MainWindow", "\342\206\221", nullptr));
        btnLeft->setText(QCoreApplication::translate("MainWindow", "\342\237\262", nullptr));
        btnRight->setText(QCoreApplication::translate("MainWindow", "\342\237\263", nullptr));
        phiLabel->setText(QCoreApplication::translate("MainWindow", "\317\225 :", nullptr));
        phiValueLabel->setText(QCoreApplication::translate("MainWindow", "sync ...", nullptr));
        goBtn->setText(QCoreApplication::translate("MainWindow", "Go", nullptr));
        xSendLabel->setText(QCoreApplication::translate("MainWindow", "x ( meter )", nullptr));
        ySendLabel->setText(QCoreApplication::translate("MainWindow", "y ( meter )", nullptr));
        zSendLabel->setText(QCoreApplication::translate("MainWindow", "z ( degree )", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
