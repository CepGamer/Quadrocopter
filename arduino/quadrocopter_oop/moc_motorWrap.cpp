/****************************************************************************
** Meta object code from reading C++ file 'motorWrap.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.5)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "wrappers/motorWrap.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'motorWrap.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.5. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_MotorWrap[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      17,   11,   10,   10, 0x0a,
      35,   10,   31,   10, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_MotorWrap[] = {
    "MotorWrap\0\0power\0setPower(int)\0int\0"
    "power()\0"
};

void MotorWrap::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        MotorWrap *_t = static_cast<MotorWrap *>(_o);
        switch (_id) {
        case 0: _t->setPower((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: { int _r = _t->power();
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = _r; }  break;
        default: ;
        }
    }
}

const QMetaObjectExtraData MotorWrap::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject MotorWrap::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_MotorWrap,
      qt_meta_data_MotorWrap, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &MotorWrap::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *MotorWrap::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *MotorWrap::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_MotorWrap))
        return static_cast<void*>(const_cast< MotorWrap*>(this));
    return QObject::qt_metacast(_clname);
}

int MotorWrap::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
