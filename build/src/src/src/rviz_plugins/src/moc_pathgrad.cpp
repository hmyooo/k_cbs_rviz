/****************************************************************************
** Meta object code from reading C++ file 'pathgrad.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.8)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../../src/src/src/src/rviz_plugins/src/pathgrad.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'pathgrad.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.8. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_rviz__PathGradDisplay_t {
    QByteArrayData data[5];
    char stringdata0[70];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz__PathGradDisplay_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz__PathGradDisplay_t qt_meta_stringdata_rviz__PathGradDisplay = {
    {
QT_MOC_LITERAL(0, 0, 21), // "rviz::PathGradDisplay"
QT_MOC_LITERAL(1, 22, 18), // "updateBufferLength"
QT_MOC_LITERAL(2, 41, 0), // ""
QT_MOC_LITERAL(3, 42, 11), // "updateStyle"
QT_MOC_LITERAL(4, 54, 15) // "updateLineWidth"

    },
    "rviz::PathGradDisplay\0updateBufferLength\0"
    "\0updateStyle\0updateLineWidth"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz__PathGradDisplay[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   29,    2, 0x09 /* Protected */,
       3,    0,   30,    2, 0x09 /* Protected */,
       4,    0,   31,    2, 0x09 /* Protected */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void rviz::PathGradDisplay::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<PathGradDisplay *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->updateBufferLength(); break;
        case 1: _t->updateStyle(); break;
        case 2: _t->updateLineWidth(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

QT_INIT_METAOBJECT const QMetaObject rviz::PathGradDisplay::staticMetaObject = { {
    &MessageFilterDisplay<nav_msgs::Path>::staticMetaObject,
    qt_meta_stringdata_rviz__PathGradDisplay.data,
    qt_meta_data_rviz__PathGradDisplay,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *rviz::PathGradDisplay::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz::PathGradDisplay::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_rviz__PathGradDisplay.stringdata0))
        return static_cast<void*>(this);
    return MessageFilterDisplay<nav_msgs::Path>::qt_metacast(_clname);
}

int rviz::PathGradDisplay::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = MessageFilterDisplay<nav_msgs::Path>::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 3)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 3;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
