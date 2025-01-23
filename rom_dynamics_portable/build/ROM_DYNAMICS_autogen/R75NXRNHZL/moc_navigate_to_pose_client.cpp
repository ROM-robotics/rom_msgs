/****************************************************************************
** Meta object code from reading C++ file 'navigate_to_pose_client.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../ros2/header/navigate_to_pose_client.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'navigate_to_pose_client.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_NavigateToPoseClient_t {
    QByteArrayData data[12];
    char stringdata0[187];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_NavigateToPoseClient_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_NavigateToPoseClient_t qt_meta_stringdata_NavigateToPoseClient = {
    {
QT_MOC_LITERAL(0, 0, 20), // "NavigateToPoseClient"
QT_MOC_LITERAL(1, 21, 16), // "navigationResult"
QT_MOC_LITERAL(2, 38, 0), // ""
QT_MOC_LITERAL(3, 39, 11), // "std::string"
QT_MOC_LITERAL(4, 51, 6), // "result"
QT_MOC_LITERAL(5, 58, 10), // "sendGoalId"
QT_MOC_LITERAL(6, 69, 23), // "rclcpp_action::GoalUUID"
QT_MOC_LITERAL(7, 93, 9), // "goal_uuid"
QT_MOC_LITERAL(8, 103, 20), // "onSendNavigationGoal"
QT_MOC_LITERAL(9, 124, 35), // "geometry_msgs::msg::Pose::Sha..."
QT_MOC_LITERAL(10, 160, 9), // "goal_pose"
QT_MOC_LITERAL(11, 170, 16) // "onSendCancelGoal"

    },
    "NavigateToPoseClient\0navigationResult\0"
    "\0std::string\0result\0sendGoalId\0"
    "rclcpp_action::GoalUUID\0goal_uuid\0"
    "onSendNavigationGoal\0"
    "geometry_msgs::msg::Pose::SharedPtr\0"
    "goal_pose\0onSendCancelGoal"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_NavigateToPoseClient[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   34,    2, 0x06 /* Public */,
       5,    1,   37,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       8,    1,   40,    2, 0x0a /* Public */,
      11,    1,   43,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 6,    7,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 9,   10,
    QMetaType::Void, 0x80000000 | 6,    7,

       0        // eod
};

void NavigateToPoseClient::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<NavigateToPoseClient *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->navigationResult((*reinterpret_cast< const std::string(*)>(_a[1]))); break;
        case 1: _t->sendGoalId((*reinterpret_cast< const rclcpp_action::GoalUUID(*)>(_a[1]))); break;
        case 2: _t->onSendNavigationGoal((*reinterpret_cast< const geometry_msgs::msg::Pose::SharedPtr(*)>(_a[1]))); break;
        case 3: _t->onSendCancelGoal((*reinterpret_cast< const rclcpp_action::GoalUUID(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (NavigateToPoseClient::*)(const std::string & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&NavigateToPoseClient::navigationResult)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (NavigateToPoseClient::*)(const rclcpp_action::GoalUUID & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&NavigateToPoseClient::sendGoalId)) {
                *result = 1;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject NavigateToPoseClient::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_NavigateToPoseClient.data,
    qt_meta_data_NavigateToPoseClient,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *NavigateToPoseClient::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *NavigateToPoseClient::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_NavigateToPoseClient.stringdata0))
        return static_cast<void*>(this);
    if (!strcmp(_clname, "rclcpp::Node"))
        return static_cast< rclcpp::Node*>(this);
    return QObject::qt_metacast(_clname);
}

int NavigateToPoseClient::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 4;
    }
    return _id;
}

// SIGNAL 0
void NavigateToPoseClient::navigationResult(const std::string & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void NavigateToPoseClient::sendGoalId(const rclcpp_action::GoalUUID & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
