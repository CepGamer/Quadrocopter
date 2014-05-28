/****************************************************************************
** Meta object code from reading C++ file 'robotWrap.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.5)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "wrappers/robotWrap.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'robotWrap.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.5. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_RobotWrapper[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      26,   13,   14,   13, 0x0a,
      44,   13,   14,   13, 0x0a,
      75,   70,   59,   13, 0x0a,
     103,   70,   90,   13, 0x0a,
     135,   13,  120,   13, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_RobotWrapper[] = {
    "RobotWrapper\0\0QStringList\0powerMotorPorts()\0"
    "encoderPorts()\0MotorWrap*\0port\0"
    "motor(QString)\0EncoderWrap*\0"
    "encoder(QString)\0GyroscopeWrap*\0"
    "gyroscope()\0"
};

void RobotWrapper::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        RobotWrapper *_t = static_cast<RobotWrapper *>(_o);
        switch (_id) {
        case 0: { QStringList _r = _t->powerMotorPorts();
            if (_a[0]) *reinterpret_cast< QStringList*>(_a[0]) = _r; }  break;
        case 1: { QStringList _r = _t->encoderPorts();
            if (_a[0]) *reinterpret_cast< QStringList*>(_a[0]) = _r; }  break;
        case 2: { MotorWrap* _r = _t->motor((*reinterpret_cast< const QString(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< MotorWrap**>(_a[0]) = _r; }  break;
        case 3: { EncoderWrap* _r = _t->encoder((*reinterpret_cast< const QString(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< EncoderWrap**>(_a[0]) = _r; }  break;
        case 4: { GyroscopeWrap* _r = _t->gyroscope();
            if (_a[0]) *reinterpret_cast< GyroscopeWrap**>(_a[0]) = _r; }  break;
        default: ;
        }
    }
}

const QMetaObjectExtraData RobotWrapper::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject RobotWrapper::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_RobotWrapper,
      qt_meta_data_RobotWrapper, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &RobotWrapper::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *RobotWrapper::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *RobotWrapper::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_RobotWrapper))
        return static_cast<void*>(const_cast< RobotWrapper*>(this));
    return QObject::qt_metacast(_clname);
}

int RobotWrapper::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
