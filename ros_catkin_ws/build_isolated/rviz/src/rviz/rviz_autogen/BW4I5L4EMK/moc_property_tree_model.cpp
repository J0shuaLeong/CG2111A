/****************************************************************************
** Meta object code from reading C++ file 'property_tree_model.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.11.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "rviz/properties/property_tree_model.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'property_tree_model.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.11.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_rviz__PropertyTreeModel_t {
    QByteArrayData data[10];
    char stringdata0[120];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz__PropertyTreeModel_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz__PropertyTreeModel_t qt_meta_stringdata_rviz__PropertyTreeModel = {
    {
QT_MOC_LITERAL(0, 0, 23), // "rviz::PropertyTreeModel"
QT_MOC_LITERAL(1, 24, 21), // "propertyHiddenChanged"
QT_MOC_LITERAL(2, 46, 0), // ""
QT_MOC_LITERAL(3, 47, 15), // "const Property*"
QT_MOC_LITERAL(4, 63, 8), // "property"
QT_MOC_LITERAL(5, 72, 13), // "configChanged"
QT_MOC_LITERAL(6, 86, 6), // "expand"
QT_MOC_LITERAL(7, 93, 11), // "QModelIndex"
QT_MOC_LITERAL(8, 105, 5), // "index"
QT_MOC_LITERAL(9, 111, 8) // "collapse"

    },
    "rviz::PropertyTreeModel\0propertyHiddenChanged\0"
    "\0const Property*\0property\0configChanged\0"
    "expand\0QModelIndex\0index\0collapse"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz__PropertyTreeModel[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   34,    2, 0x06 /* Public */,
       5,    0,   37,    2, 0x06 /* Public */,
       6,    1,   38,    2, 0x06 /* Public */,
       9,    1,   41,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 7,    8,
    QMetaType::Void, 0x80000000 | 7,    8,

       0        // eod
};

void rviz::PropertyTreeModel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        PropertyTreeModel *_t = static_cast<PropertyTreeModel *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->propertyHiddenChanged((*reinterpret_cast< const Property*(*)>(_a[1]))); break;
        case 1: _t->configChanged(); break;
        case 2: _t->expand((*reinterpret_cast< const QModelIndex(*)>(_a[1]))); break;
        case 3: _t->collapse((*reinterpret_cast< const QModelIndex(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (PropertyTreeModel::*)(const Property * );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&PropertyTreeModel::propertyHiddenChanged)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (PropertyTreeModel::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&PropertyTreeModel::configChanged)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (PropertyTreeModel::*)(const QModelIndex & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&PropertyTreeModel::expand)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (PropertyTreeModel::*)(const QModelIndex & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&PropertyTreeModel::collapse)) {
                *result = 3;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject rviz::PropertyTreeModel::staticMetaObject = {
    { &QAbstractItemModel::staticMetaObject, qt_meta_stringdata_rviz__PropertyTreeModel.data,
      qt_meta_data_rviz__PropertyTreeModel,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *rviz::PropertyTreeModel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz::PropertyTreeModel::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_rviz__PropertyTreeModel.stringdata0))
        return static_cast<void*>(this);
    return QAbstractItemModel::qt_metacast(_clname);
}

int rviz::PropertyTreeModel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QAbstractItemModel::qt_metacall(_c, _id, _a);
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
void rviz::PropertyTreeModel::propertyHiddenChanged(const Property * _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void rviz::PropertyTreeModel::configChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}

// SIGNAL 2
void rviz::PropertyTreeModel::expand(const QModelIndex & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void rviz::PropertyTreeModel::collapse(const QModelIndex & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
