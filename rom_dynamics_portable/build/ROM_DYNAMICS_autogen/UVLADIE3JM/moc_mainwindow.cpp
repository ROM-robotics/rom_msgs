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
    QByteArrayData data[7];
    char stringdata0[127];
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
QT_MOC_LITERAL(3, 32, 6), // "status"
QT_MOC_LITERAL(4, 39, 20), // "responseDataReceived"
QT_MOC_LITERAL(5, 60, 57), // "std::shared_ptr<rom_interface..."
QT_MOC_LITERAL(6, 118, 8) // "response"

    },
    "ServiceClient\0responseReceived\0\0status\0"
    "responseDataReceived\0"
    "std::shared_ptr<rom_interfaces::srv::WhichMaps::Response>\0"
    "response"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ServiceClient[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   24,    2, 0x06 /* Public */,
       4,    1,   27,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, 0x80000000 | 5,    6,

       0        // eod
};

void ServiceClient::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<ServiceClient *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->responseReceived((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->responseDataReceived((*reinterpret_cast< std::shared_ptr<rom_interfaces::srv::WhichMaps::Response>(*)>(_a[1]))); break;
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
        {
            using _t = void (ServiceClient::*)(std::shared_ptr<rom_interfaces::srv::WhichMaps::Response> );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ServiceClient::responseDataReceived)) {
                *result = 1;
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
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 2)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 2;
    }
    return _id;
}

// SIGNAL 0
void ServiceClient::responseReceived(int _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void ServiceClient::responseDataReceived(std::shared_ptr<rom_interfaces::srv::WhichMaps::Response> _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
struct qt_meta_stringdata_MainWindow_t {
    QByteArrayData data[40];
    char stringdata0[727];
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
QT_MOC_LITERAL(5, 71, 14), // "sendCancelGoal"
QT_MOC_LITERAL(6, 86, 23), // "rclcpp_action::GoalUUID"
QT_MOC_LITERAL(7, 110, 9), // "goal_uuid"
QT_MOC_LITERAL(8, 120, 9), // "selectMap"
QT_MOC_LITERAL(9, 130, 11), // "std::string"
QT_MOC_LITERAL(10, 142, 8), // "map_name"
QT_MOC_LITERAL(11, 151, 18), // "displayCurrentPose"
QT_MOC_LITERAL(12, 170, 34), // "nav_msgs::msg::Odometry::Shar..."
QT_MOC_LITERAL(13, 205, 17), // "changeCurrentMode"
QT_MOC_LITERAL(14, 223, 32), // "std_msgs::msg::String::SharedPtr"
QT_MOC_LITERAL(15, 256, 15), // "sendMappingMode"
QT_MOC_LITERAL(16, 272, 18), // "sendNavigationMode"
QT_MOC_LITERAL(17, 291, 17), // "sendRemappingMode"
QT_MOC_LITERAL(18, 309, 14), // "saveMapClicked"
QT_MOC_LITERAL(19, 324, 14), // "openMapClicked"
QT_MOC_LITERAL(20, 339, 16), // "selectMapClicked"
QT_MOC_LITERAL(21, 356, 22), // "labelEditForSetForward"
QT_MOC_LITERAL(22, 379, 20), // "labelEditForSetRight"
QT_MOC_LITERAL(23, 400, 19), // "labelEditForSetLeft"
QT_MOC_LITERAL(24, 420, 19), // "labelEditForSetStop"
QT_MOC_LITERAL(25, 440, 18), // "onNavigationResult"
QT_MOC_LITERAL(26, 459, 13), // "result_status"
QT_MOC_LITERAL(27, 473, 12), // "onSendGoalId"
QT_MOC_LITERAL(28, 486, 20), // "onCmdServiceResponse"
QT_MOC_LITERAL(29, 507, 7), // "success"
QT_MOC_LITERAL(30, 515, 22), // "on_shutdownBtn_clicked"
QT_MOC_LITERAL(31, 538, 19), // "on_btnEstop_clicked"
QT_MOC_LITERAL(32, 558, 18), // "onResponseReceived"
QT_MOC_LITERAL(33, 577, 3), // "sum"
QT_MOC_LITERAL(34, 581, 22), // "onResponseDataReceived"
QT_MOC_LITERAL(35, 604, 57), // "std::shared_ptr<rom_interface..."
QT_MOC_LITERAL(36, 662, 8), // "response"
QT_MOC_LITERAL(37, 671, 16), // "on_goBtn_clicked"
QT_MOC_LITERAL(38, 688, 20), // "on_cancelBtn_clicked"
QT_MOC_LITERAL(39, 709, 17) // "on_rthBtn_clicked"

    },
    "MainWindow\0sendNavigationGoal\0\0"
    "geometry_msgs::msg::Pose::SharedPtr\0"
    "msg\0sendCancelGoal\0rclcpp_action::GoalUUID\0"
    "goal_uuid\0selectMap\0std::string\0"
    "map_name\0displayCurrentPose\0"
    "nav_msgs::msg::Odometry::SharedPtr\0"
    "changeCurrentMode\0std_msgs::msg::String::SharedPtr\0"
    "sendMappingMode\0sendNavigationMode\0"
    "sendRemappingMode\0saveMapClicked\0"
    "openMapClicked\0selectMapClicked\0"
    "labelEditForSetForward\0labelEditForSetRight\0"
    "labelEditForSetLeft\0labelEditForSetStop\0"
    "onNavigationResult\0result_status\0"
    "onSendGoalId\0onCmdServiceResponse\0"
    "success\0on_shutdownBtn_clicked\0"
    "on_btnEstop_clicked\0onResponseReceived\0"
    "sum\0onResponseDataReceived\0"
    "std::shared_ptr<rom_interfaces::srv::WhichMaps::Response>\0"
    "response\0on_goBtn_clicked\0"
    "on_cancelBtn_clicked\0on_rthBtn_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MainWindow[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      25,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,  139,    2, 0x06 /* Public */,
       5,    1,  142,    2, 0x06 /* Public */,
       8,    1,  145,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      11,    1,  148,    2, 0x0a /* Public */,
      13,    1,  151,    2, 0x0a /* Public */,
      15,    0,  154,    2, 0x0a /* Public */,
      16,    0,  155,    2, 0x0a /* Public */,
      17,    0,  156,    2, 0x0a /* Public */,
      18,    0,  157,    2, 0x0a /* Public */,
      19,    0,  158,    2, 0x0a /* Public */,
      20,    1,  159,    2, 0x0a /* Public */,
      21,    0,  162,    2, 0x0a /* Public */,
      22,    0,  163,    2, 0x0a /* Public */,
      23,    0,  164,    2, 0x0a /* Public */,
      24,    0,  165,    2, 0x0a /* Public */,
      25,    1,  166,    2, 0x0a /* Public */,
      27,    1,  169,    2, 0x0a /* Public */,
      28,    1,  172,    2, 0x0a /* Public */,
      30,    0,  175,    2, 0x08 /* Private */,
      31,    0,  176,    2, 0x08 /* Private */,
      32,    1,  177,    2, 0x08 /* Private */,
      34,    1,  180,    2, 0x08 /* Private */,
      37,    0,  183,    2, 0x08 /* Private */,
      38,    0,  184,    2, 0x08 /* Private */,
      39,    0,  185,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 6,    7,
    QMetaType::Void, 0x80000000 | 9,   10,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 12,    4,
    QMetaType::Void, 0x80000000 | 14,    4,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 9,   10,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 9,   26,
    QMetaType::Void, 0x80000000 | 6,    7,
    QMetaType::Void, QMetaType::Bool,   29,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,   33,
    QMetaType::Void, 0x80000000 | 35,   36,
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
        case 1: _t->sendCancelGoal((*reinterpret_cast< const rclcpp_action::GoalUUID(*)>(_a[1]))); break;
        case 2: _t->selectMap((*reinterpret_cast< std::string(*)>(_a[1]))); break;
        case 3: _t->displayCurrentPose((*reinterpret_cast< const nav_msgs::msg::Odometry::SharedPtr(*)>(_a[1]))); break;
        case 4: _t->changeCurrentMode((*reinterpret_cast< const std_msgs::msg::String::SharedPtr(*)>(_a[1]))); break;
        case 5: _t->sendMappingMode(); break;
        case 6: _t->sendNavigationMode(); break;
        case 7: _t->sendRemappingMode(); break;
        case 8: _t->saveMapClicked(); break;
        case 9: _t->openMapClicked(); break;
        case 10: _t->selectMapClicked((*reinterpret_cast< std::string(*)>(_a[1]))); break;
        case 11: _t->labelEditForSetForward(); break;
        case 12: _t->labelEditForSetRight(); break;
        case 13: _t->labelEditForSetLeft(); break;
        case 14: _t->labelEditForSetStop(); break;
        case 15: _t->onNavigationResult((*reinterpret_cast< const std::string(*)>(_a[1]))); break;
        case 16: _t->onSendGoalId((*reinterpret_cast< const rclcpp_action::GoalUUID(*)>(_a[1]))); break;
        case 17: _t->onCmdServiceResponse((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 18: _t->on_shutdownBtn_clicked(); break;
        case 19: _t->on_btnEstop_clicked(); break;
        case 20: _t->onResponseReceived((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 21: _t->onResponseDataReceived((*reinterpret_cast< std::shared_ptr<rom_interfaces::srv::WhichMaps::Response>(*)>(_a[1]))); break;
        case 22: _t->on_goBtn_clicked(); break;
        case 23: _t->on_cancelBtn_clicked(); break;
        case 24: _t->on_rthBtn_clicked(); break;
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
        {
            using _t = void (MainWindow::*)(const rclcpp_action::GoalUUID & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MainWindow::sendCancelGoal)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (MainWindow::*)(std::string );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MainWindow::selectMap)) {
                *result = 2;
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
        if (_id < 25)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 25;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 25)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 25;
    }
    return _id;
}

// SIGNAL 0
void MainWindow::sendNavigationGoal(const geometry_msgs::msg::Pose::SharedPtr _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void MainWindow::sendCancelGoal(const rclcpp_action::GoalUUID & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void MainWindow::selectMap(std::string _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
