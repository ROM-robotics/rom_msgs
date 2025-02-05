/****************************************************************************
** Meta object code from reading C++ file 'cmd_service_client.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../ros2/header/cmd_service_client.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'cmd_service_client.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_CmdServiceClient_t {
    QByteArrayData data[9];
    char stringdata0[94];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_CmdServiceClient_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_CmdServiceClient_t qt_meta_stringdata_CmdServiceClient = {
    {
QT_MOC_LITERAL(0, 0, 16), // "CmdServiceClient"
QT_MOC_LITERAL(1, 17, 15), // "commandResponse"
QT_MOC_LITERAL(2, 33, 0), // ""
QT_MOC_LITERAL(3, 34, 7), // "success"
QT_MOC_LITERAL(4, 42, 15), // "serviceResponse"
QT_MOC_LITERAL(5, 58, 10), // "setForward"
QT_MOC_LITERAL(6, 69, 7), // "setLeft"
QT_MOC_LITERAL(7, 77, 8), // "setRight"
QT_MOC_LITERAL(8, 86, 7) // "setStop"

    },
    "CmdServiceClient\0commandResponse\0\0"
    "success\0serviceResponse\0setForward\0"
    "setLeft\0setRight\0setStop"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_CmdServiceClient[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   44,    2, 0x06 /* Public */,
       4,    1,   47,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       5,    0,   50,    2, 0x0a /* Public */,
       6,    0,   51,    2, 0x0a /* Public */,
       7,    0,   52,    2, 0x0a /* Public */,
       8,    0,   53,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void, QMetaType::Bool,    3,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void CmdServiceClient::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<CmdServiceClient *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->commandResponse((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 1: _t->serviceResponse((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 2: _t->setForward(); break;
        case 3: _t->setLeft(); break;
        case 4: _t->setRight(); break;
        case 5: _t->setStop(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (CmdServiceClient::*)(bool );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&CmdServiceClient::commandResponse)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (CmdServiceClient::*)(bool );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&CmdServiceClient::serviceResponse)) {
                *result = 1;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject CmdServiceClient::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_CmdServiceClient.data,
    qt_meta_data_CmdServiceClient,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *CmdServiceClient::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *CmdServiceClient::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_CmdServiceClient.stringdata0))
        return static_cast<void*>(this);
    if (!strcmp(_clname, "rclcpp::Node"))
        return static_cast< rclcpp::Node*>(this);
    return QObject::qt_metacast(_clname);
}

int CmdServiceClient::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 6)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 6;
    }
    return _id;
}

// SIGNAL 0
void CmdServiceClient::commandResponse(bool _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void CmdServiceClient::serviceResponse(bool _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
struct qt_meta_stringdata_ConstructYamlServiceClient_t {
    QByteArrayData data[5];
    char stringdata0[127];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_ConstructYamlServiceClient_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_ConstructYamlServiceClient_t qt_meta_stringdata_ConstructYamlServiceClient = {
    {
QT_MOC_LITERAL(0, 0, 26), // "ConstructYamlServiceClient"
QT_MOC_LITERAL(1, 27, 15), // "onSendWaypoints"
QT_MOC_LITERAL(2, 43, 0), // ""
QT_MOC_LITERAL(3, 44, 74), // "std::shared_ptr<std::unordere..."
QT_MOC_LITERAL(4, 119, 7) // "wp_list"

    },
    "ConstructYamlServiceClient\0onSendWaypoints\0"
    "\0"
    "std::shared_ptr<std::unordered_map<std::string,geometry_msgs::msg::Pos"
    "e> >\0"
    "wp_list"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ConstructYamlServiceClient[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       1,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   19,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 3,    4,

       0        // eod
};

void ConstructYamlServiceClient::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<ConstructYamlServiceClient *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->onSendWaypoints((*reinterpret_cast< std::shared_ptr<std::unordered_map<std::string,geometry_msgs::msg::Pose> >(*)>(_a[1]))); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject ConstructYamlServiceClient::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_ConstructYamlServiceClient.data,
    qt_meta_data_ConstructYamlServiceClient,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *ConstructYamlServiceClient::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ConstructYamlServiceClient::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_ConstructYamlServiceClient.stringdata0))
        return static_cast<void*>(this);
    if (!strcmp(_clname, "rclcpp::Node"))
        return static_cast< rclcpp::Node*>(this);
    return QObject::qt_metacast(_clname);
}

int ConstructYamlServiceClient::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
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
struct qt_meta_stringdata_WaypointListSubscriber_t {
    QByteArrayData data[5];
    char stringdata0[69];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_WaypointListSubscriber_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_WaypointListSubscriber_t qt_meta_stringdata_WaypointListSubscriber = {
    {
QT_MOC_LITERAL(0, 0, 22), // "WaypointListSubscriber"
QT_MOC_LITERAL(1, 23, 10), // "updateWpUI"
QT_MOC_LITERAL(2, 34, 0), // ""
QT_MOC_LITERAL(3, 35, 24), // "std::vector<std::string>"
QT_MOC_LITERAL(4, 60, 8) // "wp_names"

    },
    "WaypointListSubscriber\0updateWpUI\0\0"
    "std::vector<std::string>\0wp_names"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_WaypointListSubscriber[] = {

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
    QMetaType::Void, 0x80000000 | 3,    4,

       0        // eod
};

void WaypointListSubscriber::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<WaypointListSubscriber *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->updateWpUI((*reinterpret_cast< std::vector<std::string>(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (WaypointListSubscriber::*)(std::vector<std::string> );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&WaypointListSubscriber::updateWpUI)) {
                *result = 0;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject WaypointListSubscriber::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_WaypointListSubscriber.data,
    qt_meta_data_WaypointListSubscriber,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *WaypointListSubscriber::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *WaypointListSubscriber::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_WaypointListSubscriber.stringdata0))
        return static_cast<void*>(this);
    if (!strcmp(_clname, "rclcpp::Node"))
        return static_cast< rclcpp::Node*>(this);
    return QObject::qt_metacast(_clname);
}

int WaypointListSubscriber::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
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
void WaypointListSubscriber::updateWpUI(std::vector<std::string> _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
struct qt_meta_stringdata_SendWaypointsClient_t {
    QByteArrayData data[5];
    char stringdata0[75];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_SendWaypointsClient_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_SendWaypointsClient_t qt_meta_stringdata_SendWaypointsClient = {
    {
QT_MOC_LITERAL(0, 0, 19), // "SendWaypointsClient"
QT_MOC_LITERAL(1, 20, 19), // "onSendWaypointsGoal"
QT_MOC_LITERAL(2, 40, 0), // ""
QT_MOC_LITERAL(3, 41, 24), // "std::vector<std::string>"
QT_MOC_LITERAL(4, 66, 8) // "wp_names"

    },
    "SendWaypointsClient\0onSendWaypointsGoal\0"
    "\0std::vector<std::string>\0wp_names"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_SendWaypointsClient[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       1,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   19,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 3,    4,

       0        // eod
};

void SendWaypointsClient::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<SendWaypointsClient *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->onSendWaypointsGoal((*reinterpret_cast< std::vector<std::string>(*)>(_a[1]))); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject SendWaypointsClient::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_SendWaypointsClient.data,
    qt_meta_data_SendWaypointsClient,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *SendWaypointsClient::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *SendWaypointsClient::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_SendWaypointsClient.stringdata0))
        return static_cast<void*>(this);
    if (!strcmp(_clname, "rclcpp::Node"))
        return static_cast< rclcpp::Node*>(this);
    return QObject::qt_metacast(_clname);
}

int SendWaypointsClient::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
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
QT_WARNING_POP
QT_END_MOC_NAMESPACE
