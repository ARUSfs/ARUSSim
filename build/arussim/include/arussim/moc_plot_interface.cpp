/****************************************************************************
** Meta object code from reading C++ file 'plot_interface.hpp'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../src/arussim/include/arussim/plot_interface.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'plot_interface.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_plot_interface__PlotInterface_t {
    QByteArrayData data[23];
    char stringdata0[309];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_plot_interface__PlotInterface_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_plot_interface__PlotInterface_t qt_meta_stringdata_plot_interface__PlotInterface = {
    {
QT_MOC_LITERAL(0, 0, 29), // "plot_interface::PlotInterface"
QT_MOC_LITERAL(1, 30, 21), // "launch_button_clicked"
QT_MOC_LITERAL(2, 52, 0), // ""
QT_MOC_LITERAL(3, 53, 14), // "reset_callback"
QT_MOC_LITERAL(4, 68, 20), // "update_telemetry_bar"
QT_MOC_LITERAL(5, 89, 9), // "fr_param_"
QT_MOC_LITERAL(6, 99, 9), // "fl_param_"
QT_MOC_LITERAL(7, 109, 9), // "rr_param_"
QT_MOC_LITERAL(8, 119, 9), // "rl_param_"
QT_MOC_LITERAL(9, 129, 22), // "update_vx_target_graph"
QT_MOC_LITERAL(10, 152, 15), // "update_gg_graph"
QT_MOC_LITERAL(11, 168, 23), // "update_telemetry_labels"
QT_MOC_LITERAL(12, 192, 14), // "state_callback"
QT_MOC_LITERAL(13, 207, 3), // "vx_"
QT_MOC_LITERAL(14, 211, 3), // "vy_"
QT_MOC_LITERAL(15, 215, 2), // "r_"
QT_MOC_LITERAL(16, 218, 3), // "ax_"
QT_MOC_LITERAL(17, 222, 3), // "ay_"
QT_MOC_LITERAL(18, 226, 6), // "delta_"
QT_MOC_LITERAL(19, 233, 19), // "zoom_in_speed_graph"
QT_MOC_LITERAL(20, 253, 20), // "zoom_out_speed_graph"
QT_MOC_LITERAL(21, 274, 16), // "zoom_in_gg_graph"
QT_MOC_LITERAL(22, 291, 17) // "zoom_out_gg_graph"

    },
    "plot_interface::PlotInterface\0"
    "launch_button_clicked\0\0reset_callback\0"
    "update_telemetry_bar\0fr_param_\0fl_param_\0"
    "rr_param_\0rl_param_\0update_vx_target_graph\0"
    "update_gg_graph\0update_telemetry_labels\0"
    "state_callback\0vx_\0vy_\0r_\0ax_\0ay_\0"
    "delta_\0zoom_in_speed_graph\0"
    "zoom_out_speed_graph\0zoom_in_gg_graph\0"
    "zoom_out_gg_graph"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_plot_interface__PlotInterface[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      11,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   69,    2, 0x08 /* Private */,
       3,    0,   70,    2, 0x08 /* Private */,
       4,    4,   71,    2, 0x08 /* Private */,
       9,    0,   80,    2, 0x08 /* Private */,
      10,    0,   81,    2, 0x08 /* Private */,
      11,    0,   82,    2, 0x08 /* Private */,
      12,    6,   83,    2, 0x08 /* Private */,
      19,    0,   96,    2, 0x08 /* Private */,
      20,    0,   97,    2, 0x08 /* Private */,
      21,    0,   98,    2, 0x08 /* Private */,
      22,    0,   99,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double,    5,    6,    7,    8,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double,   13,   14,   15,   16,   17,   18,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void plot_interface::PlotInterface::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<PlotInterface *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->launch_button_clicked(); break;
        case 1: _t->reset_callback(); break;
        case 2: _t->update_telemetry_bar((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< double(*)>(_a[4]))); break;
        case 3: _t->update_vx_target_graph(); break;
        case 4: _t->update_gg_graph(); break;
        case 5: _t->update_telemetry_labels(); break;
        case 6: _t->state_callback((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< double(*)>(_a[4])),(*reinterpret_cast< double(*)>(_a[5])),(*reinterpret_cast< double(*)>(_a[6]))); break;
        case 7: _t->zoom_in_speed_graph(); break;
        case 8: _t->zoom_out_speed_graph(); break;
        case 9: _t->zoom_in_gg_graph(); break;
        case 10: _t->zoom_out_gg_graph(); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject plot_interface::PlotInterface::staticMetaObject = { {
    QMetaObject::SuperData::link<rviz_common::Panel::staticMetaObject>(),
    qt_meta_stringdata_plot_interface__PlotInterface.data,
    qt_meta_data_plot_interface__PlotInterface,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *plot_interface::PlotInterface::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *plot_interface::PlotInterface::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_plot_interface__PlotInterface.stringdata0))
        return static_cast<void*>(this);
    return rviz_common::Panel::qt_metacast(_clname);
}

int plot_interface::PlotInterface::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = rviz_common::Panel::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 11)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 11;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 11)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 11;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
