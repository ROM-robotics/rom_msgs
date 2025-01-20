/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../src/mainwindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_ServiceClient_t {
    QByteArrayData data[4];
    char stringdata0[39];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_ServiceClient_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_ServiceClient_t qt_meta_stringdata_ServiceClient = {
    {
QT_MOC_LITERAL(0, 0, 13), // "ServiceClient"
QT_MOC_LITERAL(1, 14, 16), // "responseReceived"
QT_MOC_LITERAL(2, 31, 0), // ""
QT_MOC_LITERAL(3, 32, 6) // "status"

    },
    "ServiceClient\0responseReceived\0\0status"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ServiceClient[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       1,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   19,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::Int,    3,

       0        // eod
};

void ServiceClient::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<ServiceClient *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->responseReceived((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (ServiceClient::*)(int );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ServiceClient::responseReceived)) {
                *result = 0;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject ServiceClient::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_ServiceClient.data,
    qt_meta_data_ServiceClient,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *ServiceClient::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ServiceClient::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_ServiceClient.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int ServiceClient::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 1)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 1;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 1)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 1;
    }
    return _id;
}

// SIGNAL 0
void ServiceClient::responseReceived(int _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
struct qt_meta_stringdata_MainWindow_t {
    QByteArrayData data[29];
    char stringdata0[517];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MainWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MainWindow_t qt_meta_stringdata_MainWindow = {
    {
QT_MOC_LITERAL(0, 0, 10), // "MainWindow"
QT_MOC_LITERAL(1, 11, 18), // "sendNavigationGoal"
QT_MOC_LITERAL(2, 30, 0), // ""
QT_MOC_LITERAL(3, 31, 35), // "geometry_msgs::msg::Pose::Sha..."
QT_MOC_LITERAL(4, 67, 3), // "msg"
QT_MOC_LITERAL(5, 71, 18), // "displayCurrentPose"
QT_MOC_LITERAL(6, 90, 34), // "nav_msgs::msg::Odometry::Shar..."
QT_MOC_LITERAL(7, 125, 17), // "changeCurrentMode"
QT_MOC_LITERAL(8, 143, 32), // "std_msgs::msg::String::SharedPtr"
QT_MOC_LITERAL(9, 176, 15), // "sendMappingMode"
QT_MOC_LITERAL(10, 192, 18), // "sendNavigationMode"
QT_MOC_LITERAL(11, 211, 17), // "sendRemappingMode"
QT_MOC_LITERAL(12, 229, 14), // "saveMapClicked"
QT_MOC_LITERAL(13, 244, 14), // "openMapClicked"
QT_MOC_LITERAL(14, 259, 16), // "selectMapClicked"
QT_MOC_LITERAL(15, 276, 22), // "labelEditForSetForward"
QT_MOC_LITERAL(16, 299, 20), // "labelEditForSetRight"
QT_MOC_LITERAL(17, 320, 19), // "labelEditForSetLeft"
QT_MOC_LITERAL(18, 340, 19), // "labelEditForSetStop"
QT_MOC_LITERAL(19, 360, 18), // "onNavigationResult"
QT_MOC_LITERAL(20, 379, 11), // "std::string"
QT_MOC_LITERAL(21, 391, 3), // "_t1"
QT_MOC_LITERAL(22, 395, 22), // "on_shutdownBtn_clicked"
QT_MOC_LITERAL(23, 418, 19), // "on_btnEstop_clicked"
QT_MOC_LITERAL(24, 438, 18), // "onResponseReceived"
QT_MOC_LITERAL(25, 457, 3), // "sum"
QT_MOC_LITERAL(26, 461, 16), // "on_goBtn_clicked"
QT_MOC_LITERAL(27, 478, 20), // "on_cancelBtn_clicked"
QT_MOC_LITERAL(28, 499, 17) // "on_rthBtn_clicked"

    },
    "MainWindow\0sendNavigationGoal\0\0"
    "geometry_msgs::msg::Pose::SharedPtr\0"
    "msg\0displayCurrentPose\0"
    "nav_msgs::msg::Odometry::SharedPtr\0"
    "changeCurrentMode\0std_msgs::msg::String::SharedPtr\0"
    "sendMappingMode\0sendNavigationMode\0"
    "sendRemappingMode\0saveMapClicked\0"
    "openMapClicked\0selectMapClicked\0"
    "labelEditForSetForward\0labelEditForSetRight\0"
    "labelEditForSetLeft\0labelEditForSetStop\0"
    "onNavigationResult\0std::string\0_t1\0"
    "on_shutdownBtn_clicked\0on_btnEstop_clicked\0"
    "onResponseReceived\0sum\0on_goBtn_clicked\0"
    "on_cancelBtn_clicked\0on_rthBtn_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MainWindow[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      20,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,  114,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       5,    1,  117,    2, 0x0a /* Public */,
       7,    1,  120,    2, 0x0a /* Public */,
       9,    0,  123,    2, 0x0a /* Public */,
      10,    0,  124,    2, 0x0a /* Public */,
      11,    0,  125,    2, 0x0a /* Public */,
      12,    0,  126,    2, 0x0a /* Public */,
      13,    0,  127,    2, 0x0a /* Public */,
      14,    0,  128,    2, 0x0a /* Public */,
      15,    0,  129,    2, 0x0a /* Public */,
      16,    0,  130,    2, 0x0a /* Public */,
      17,    0,  131,    2, 0x0a /* Public */,
      18,    0,  132,    2, 0x0a /* Public */,
      19,    1,  133,    2, 0x0a /* Public */,
      22,    0,  136,    2, 0x08 /* Private */,
      23,    0,  137,    2, 0x08 /* Private */,
      24,    1,  138,    2, 0x08 /* Private */,
      26,    0,  141,    2, 0x08 /* Private */,
      27,    0,  142,    2, 0x08 /* Private */,
      28,    0,  143,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 6,    4,
    QMetaType::Void, 0x80000000 | 8,    4,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 20,   21,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,   25,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<MainWindow *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->sendNavigationGoal((*reinterpret_cast< const geometry_msgs::msg::Pose::SharedPtr(*)>(_a[1]))); break;
        case 1: _t->displayCurrentPose((*reinterpret_cast< const nav_msgs::msg::Odometry::SharedPtr(*)>(_a[1]))); break;
        case 2: _t->changeCurrentMode((*reinterpret_cast< const std_msgs::msg::String::SharedPtr(*)>(_a[1]))); break;
        case 3: _t->sendMappingMode(); break;
        case 4: _t->sendNavigationMode(); break;
        case 5: _t->sendRemappingMode(); break;
        case 6: _t->saveMapClicked(); break;
        case 7: _t->openMapClicked(); break;
        case 8: _t->selectMapClicked(); break;
        case 9: _t->labelEditForSetForward(); break;
        case 10: _t->labelEditForSetRight(); break;
        case 11: _t->labelEditForSetLeft(); break;
        case 12: _t->labelEditForSetStop(); break;
        case 13: _t->onNavigationResult((*reinterpret_cast< const std::string(*)>(_a[1]))); break;
        case 14: _t->on_shutdownBtn_clicked(); break;
        case 15: _t->on_btnEstop_clicked(); break;
        case 16: _t->onResponseReceived((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 17: _t->on_goBtn_clicked(); break;
        case 18: _t->on_cancelBtn_clicked(); break;
        case 19: _t->on_rthBtn_clicked(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (MainWindow::*)(const geometry_msgs::msg::Pose::SharedPtr );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MainWindow::sendNavigationGoal)) {
                *result = 0;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject MainWindow::staticMetaObject = { {
    QMetaObject::SuperData::link<QMainWindow::staticMetaObject>(),
    qt_meta_stringdata_MainWindow.data,
    qt_meta_data_MainWindow,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow.stringdata0))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 20)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 20;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 20)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 20;
    }
    return _id;
}

// SIGNAL 0
void MainWindow::sendNavigationGoal(const geometry_msgs::msg::Pose::SharedPtr _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
