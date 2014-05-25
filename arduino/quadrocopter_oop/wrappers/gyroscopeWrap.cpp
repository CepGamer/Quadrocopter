#include "gyroscopeWrap.h"

GyroscopeWrap::GyroscopeWrap(trikControl::Sensor3d *gyroscope) :
    QObject(gyroscope)
  , isReal(true)
  , realGyro(gyroscope)
  , emulGyro(nullptr)
{
}

GyroscopeWrap::GyroscopeWrap(emulators::GyroscopeEmulator *gyroscope):
    QObject(gyroscope)
    , isReal(false)
    , realGyro(nullptr)
    , emulGyro(gyroscope)
{
}

GyroscopeWrap::~GyroscopeWrap()
{

}

QVector<int> const &GyroscopeWrap::read() const
{
    if(isReal)
        return realGyro->read();
    else
        return emulGyro->read();
}

const QVector<int> &GyroscopeWrap::readRenewed(int * speeds) const
{
    if(!isReal)
        return emulGyro->readRenewed(speeds);
}
