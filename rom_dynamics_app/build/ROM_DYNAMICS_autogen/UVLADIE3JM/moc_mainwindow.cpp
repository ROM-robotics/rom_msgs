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
    QByteArrayData data[50];
    char stringdata0[990];
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
QT_MOC_LITERAL(5, 71, 9), // "selectMap"
QT_MOC_LITERAL(6, 81, 11), // "std::string"
QT_MOC_LITERAL(7, 93, 8), // "map_name"
QT_MOC_LITERAL(8, 102, 13), // "sendWaypoints"
QT_MOC_LITERAL(9, 116, 74), // "std::shared_ptr<std::unordere..."
QT_MOC_LITERAL(10, 191, 7), // "wp_list"
QT_MOC_LITERAL(11, 199, 17), // "sendWaypointsGoal"
QT_MOC_LITERAL(12, 217, 24), // "std::vector<std::string>"
QT_MOC_LITERAL(13, 242, 8), // "wp_names"
QT_MOC_LITERAL(14, 251, 18), // "displayCurrentPose"
QT_MOC_LITERAL(15, 270, 34), // "nav_msgs::msg::Odometry::Shar..."
QT_MOC_LITERAL(16, 305, 17), // "changeCurrentMode"
QT_MOC_LITERAL(17, 323, 32), // "std_msgs::msg::String::SharedPtr"
QT_MOC_LITERAL(18, 356, 15), // "sendMappingMode"
QT_MOC_LITERAL(19, 372, 18), // "sendNavigationMode"
QT_MOC_LITERAL(20, 391, 17), // "sendRemappingMode"
QT_MOC_LITERAL(21, 409, 14), // "saveMapClicked"
QT_MOC_LITERAL(22, 424, 14), // "openMapClicked"
QT_MOC_LITERAL(23, 439, 16), // "selectMapClicked"
QT_MOC_LITERAL(24, 456, 22), // "labelEditForSetForward"
QT_MOC_LITERAL(25, 479, 20), // "labelEditForSetRight"
QT_MOC_LITERAL(26, 500, 19), // "labelEditForSetLeft"
QT_MOC_LITERAL(27, 520, 19), // "labelEditForSetStop"
QT_MOC_LITERAL(28, 540, 18), // "onNavigationResult"
QT_MOC_LITERAL(29, 559, 13), // "result_status"
QT_MOC_LITERAL(30, 573, 20), // "onCmdServiceResponse"
QT_MOC_LITERAL(31, 594, 7), // "success"
QT_MOC_LITERAL(32, 602, 12), // "onUpdateWpUI"
QT_MOC_LITERAL(33, 615, 11), // "onUpdateMap"
QT_MOC_LITERAL(34, 627, 39), // "nav_msgs::msg::OccupancyGrid:..."
QT_MOC_LITERAL(35, 667, 19), // "onZoomButtonClicked"
QT_MOC_LITERAL(36, 687, 24), // "onWayPointsButtonClicked"
QT_MOC_LITERAL(37, 712, 19), // "onWallButtonClicked"
QT_MOC_LITERAL(38, 732, 21), // "onEraserButtonClicked"
QT_MOC_LITERAL(39, 754, 21), // "onNormalButtonClicked"
QT_MOC_LITERAL(40, 776, 22), // "on_shutdownBtn_clicked"
QT_MOC_LITERAL(41, 799, 19), // "on_btnEstop_clicked"
QT_MOC_LITERAL(42, 819, 18), // "onResponseReceived"
QT_MOC_LITERAL(43, 838, 3), // "sum"
QT_MOC_LITERAL(44, 842, 22), // "onResponseDataReceived"
QT_MOC_LITERAL(45, 865, 57), // "std::shared_ptr<rom_interface..."
QT_MOC_LITERAL(46, 923, 8), // "response"
QT_MOC_LITERAL(47, 932, 16), // "on_goBtn_clicked"
QT_MOC_LITERAL(48, 949, 22), // "on_waypointBtn_clicked"
QT_MOC_LITERAL(49, 972, 17) // "on_rthBtn_clicked"

    },
    "MainWindow\0sendNavigationGoal\0\0"
    "geometry_msgs::msg::Pose::SharedPtr\0"
    "msg\0selectMap\0std::string\0map_name\0"
    "sendWaypoints\0"
    "std::shared_ptr<std::unordered_map<std::string,geometry_msgs::msg::Pos"
    "e> >\0"
    "wp_list\0sendWaypointsGoal\0"
    "std::vector<std::string>\0wp_names\0"
    "displayCurrentPose\0"
    "nav_msgs::msg::Odometry::SharedPtr\0"
    "changeCurrentMode\0std_msgs::msg::String::SharedPtr\0"
    "sendMappingMode\0sendNavigationMode\0"
    "sendRemappingMode\0saveMapClicked\0"
    "openMapClicked\0selectMapClicked\0"
    "labelEditForSetForward\0labelEditForSetRight\0"
    "labelEditForSetLeft\0labelEditForSetStop\0"
    "onNavigationResult\0result_status\0"
    "onCmdServiceResponse\0success\0onUpdateWpUI\0"
    "onUpdateMap\0nav_msgs::msg::OccupancyGrid::SharedPtr\0"
    "onZoomButtonClicked\0onWayPointsButtonClicked\0"
    "onWallButtonClicked\0onEraserButtonClicked\0"
    "onNormalButtonClicked\0on_shutdownBtn_clicked\0"
    "on_btnEstop_clicked\0onResponseReceived\0"
    "sum\0onResponseDataReceived\0"
    "std::shared_ptr<rom_interfaces::srv::WhichMaps::Response>\0"
    "response\0on_goBtn_clicked\0"
    "on_waypointBtn_clicked\0on_rthBtn_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MainWindow[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      32,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,  174,    2, 0x06 /* Public */,
       5,    1,  177,    2, 0x06 /* Public */,
       8,    1,  180,    2, 0x06 /* Public */,
      11,    1,  183,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      14,    1,  186,    2, 0x0a /* Public */,
      16,    1,  189,    2, 0x0a /* Public */,
      18,    0,  192,    2, 0x0a /* Public */,
      19,    0,  193,    2, 0x0a /* Public */,
      20,    0,  194,    2, 0x0a /* Public */,
      21,    0,  195,    2, 0x0a /* Public */,
      22,    0,  196,    2, 0x0a /* Public */,
      23,    1,  197,    2, 0x0a /* Public */,
      24,    0,  200,    2, 0x0a /* Public */,
      25,    0,  201,    2, 0x0a /* Public */,
      26,    0,  202,    2, 0x0a /* Public */,
      27,    0,  203,    2, 0x0a /* Public */,
      28,    1,  204,    2, 0x0a /* Public */,
      30,    1,  207,    2, 0x0a /* Public */,
      32,    1,  210,    2, 0x0a /* Public */,
      33,    1,  213,    2, 0x0a /* Public */,
      35,    0,  216,    2, 0x0a /* Public */,
      36,    0,  217,    2, 0x0a /* Public */,
      37,    0,  218,    2, 0x0a /* Public */,
      38,    0,  219,    2, 0x0a /* Public */,
      39,    0,  220,    2, 0x0a /* Public */,
      40,    0,  221,    2, 0x08 /* Private */,
      41,    0,  222,    2, 0x08 /* Private */,
      42,    1,  223,    2, 0x08 /* Private */,
      44,    1,  226,    2, 0x08 /* Private */,
      47,    0,  229,    2, 0x08 /* Private */,
      48,    0,  230,    2, 0x08 /* Private */,
      49,    0,  231,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 6,    7,
    QMetaType::Void, 0x80000000 | 9,   10,
    QMetaType::Void, 0x80000000 | 12,   13,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 15,    4,
    QMetaType::Void, 0x80000000 | 17,    4,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 6,    7,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 6,   29,
    QMetaType::Void, QMetaType::Bool,   31,
    QMetaType::Void, 0x80000000 | 12,   13,
    QMetaType::Void, 0x80000000 | 34,    4,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,   43,
    QMetaType::Void, 0x80000000 | 45,   46,
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
        case 1: _t->selectMap((*reinterpret_cast< std::string(*)>(_a[1]))); break;
        case 2: _t->sendWaypoints((*reinterpret_cast< std::shared_ptr<std::unordered_map<std::string,geometry_msgs::msg::Pose> >(*)>(_a[1]))); break;
        case 3: _t->sendWaypointsGoal((*reinterpret_cast< std::vector<std::string>(*)>(_a[1]))); break;
        case 4: _t->displayCurrentPose((*reinterpret_cast< const nav_msgs::msg::Odometry::SharedPtr(*)>(_a[1]))); break;
        case 5: _t->changeCurrentMode((*reinterpret_cast< const std_msgs::msg::String::SharedPtr(*)>(_a[1]))); break;
        case 6: _t->sendMappingMode(); break;
        case 7: _t->sendNavigationMode(); break;
        case 8: _t->sendRemappingMode(); break;
        case 9: _t->saveMapClicked(); break;
        case 10: _t->openMapClicked(); break;
        case 11: _t->selectMapClicked((*reinterpret_cast< std::string(*)>(_a[1]))); break;
        case 12: _t->labelEditForSetForward(); break;
        case 13: _t->labelEditForSetRight(); break;
        case 14: _t->labelEditForSetLeft(); break;
        case 15: _t->labelEditForSetStop(); break;
        case 16: _t->onNavigationResult((*reinterpret_cast< const std::string(*)>(_a[1]))); break;
        case 17: _t->onCmdServiceResponse((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 18: _t->onUpdateWpUI((*reinterpret_cast< std::vector<std::string>(*)>(_a[1]))); break;
        case 19: _t->onUpdateMap((*reinterpret_cast< const nav_msgs::msg::OccupancyGrid::SharedPtr(*)>(_a[1]))); break;
        case 20: _t->onZoomButtonClicked(); break;
        case 21: _t->onWayPointsButtonClicked(); break;
        case 22: _t->onWallButtonClicked(); break;
        case 23: _t->onEraserButtonClicked(); break;
        case 24: _t->onNormalButtonClicked(); break;
        case 25: _t->on_shutdownBtn_clicked(); break;
        case 26: _t->on_btnEstop_clicked(); break;
        case 27: _t->onResponseReceived((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 28: _t->onResponseDataReceived((*reinterpret_cast< std::shared_ptr<rom_interfaces::srv::WhichMaps::Response>(*)>(_a[1]))); break;
        case 29: _t->on_goBtn_clicked(); break;
        case 30: _t->on_waypointBtn_clicked(); break;
        case 31: _t->on_rthBtn_clicked(); break;
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
            using _t = void (MainWindow::*)(std::string );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MainWindow::selectMap)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (MainWindow::*)(std::shared_ptr<std::unordered_map<std::string,geometry_msgs::msg::Pose>> );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MainWindow::sendWaypoints)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (MainWindow::*)(std::vector<std::string> );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MainWindow::sendWaypointsGoal)) {
                *result = 3;
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
        if (_id < 32)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 32;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 32)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 32;
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
void MainWindow::selectMap(std::string _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void MainWindow::sendWaypoints(std::shared_ptr<std::unordered_map<std::string,geometry_msgs::msg::Pose>> _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void MainWindow::sendWaypointsGoal(std::vector<std::string> _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
