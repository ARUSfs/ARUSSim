/****************************************************************************
** Meta object code from reading C++ file 'main_interface.hpp'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../src/arussim/include/arussim/main_interface.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'main_interface.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_main_interface__MainInterface_t {
    QByteArrayData data[10];
    char stringdata0[166];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_main_interface__MainInterface_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_main_interface__MainInterface_t qt_meta_stringdata_main_interface__MainInterface = {
    {
QT_MOC_LITERAL(0, 0, 29), // "main_interface::MainInterface"
QT_MOC_LITERAL(1, 30, 21), // "launch_button_clicked"
QT_MOC_LITERAL(2, 52, 0), // ""
QT_MOC_LITERAL(3, 53, 19), // "stop_button_clicked"
QT_MOC_LITERAL(4, 73, 20), // "reset_button_clicked"
QT_MOC_LITERAL(5, 94, 22), // "update_lap_time_labels"
QT_MOC_LITERAL(6, 117, 9), // "lap_time_"
QT_MOC_LITERAL(7, 127, 16), // "circuit_selector"
QT_MOC_LITERAL(8, 144, 6), // "option"
QT_MOC_LITERAL(9, 151, 14) // "process_output"

    },
    "main_interface::MainInterface\0"
    "launch_button_clicked\0\0stop_button_clicked\0"
    "reset_button_clicked\0update_lap_time_labels\0"
    "lap_time_\0circuit_selector\0option\0"
    "process_output"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_main_interface__MainInterface[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   44,    2, 0x08 /* Private */,
       3,    0,   45,    2, 0x08 /* Private */,
       4,    0,   46,    2, 0x08 /* Private */,
       5,    1,   47,    2, 0x08 /* Private */,
       7,    1,   50,    2, 0x08 /* Private */,
       9,    0,   53,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Double,    6,
    QMetaType::Void, QMetaType::QString,    8,
    QMetaType::Void,

       0        // eod
};

void main_interface::MainInterface::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<MainInterface *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->launch_button_clicked(); break;
        case 1: _t->stop_button_clicked(); break;
        case 2: _t->reset_button_clicked(); break;
        case 3: _t->update_lap_time_labels((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 4: _t->circuit_selector((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 5: _t->process_output(); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject main_interface::MainInterface::staticMetaObject = { {
    QMetaObject::SuperData::link<rviz_common::Panel::staticMetaObject>(),
    qt_meta_stringdata_main_interface__MainInterface.data,
    qt_meta_data_main_interface__MainInterface,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *main_interface::MainInterface::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *main_interface::MainInterface::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_main_interface__MainInterface.stringdata0))
        return static_cast<void*>(this);
    return rviz_common::Panel::qt_metacast(_clname);
}

int main_interface::MainInterface::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = rviz_common::Panel::qt_metacall(_c, _id, _a);
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
QT_WARNING_POP
QT_END_MOC_NAMESPACE
