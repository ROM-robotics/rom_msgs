#include "mainwindow.h"
#include <QtWidgets/QApplication>
#include <QObject>

int main(int argc, char *argv[])
{
    // QT APPLICATION //
    QApplication a(argc, argv);
    a.setStyleSheet(
        "QPushButton {"
        "    border: 3px solid #8f8f91;"
        "    border-radius: 25px;" // Make buttons round
        "     font-weight: bold;"
        "}"
        "QLabel {"
        "    border-radius: 25px;" // Make buttons round
        "}"
    );

    MainWindow mainWindow;

    mainWindow.show();
   
    return a.exec();
}

