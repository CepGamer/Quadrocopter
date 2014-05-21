#pragma once

#include <QObject>
#include "../../../routeBuilder-master/emulatorTest/gyroscopeEmulator.h"
#include "../../trikRuntime/trikControl/include/trikControl/sensor3d.h"

class GyroscopeWrap : public QObject
{
    Q_OBJECT
public:
    explicit GyroscopeWrap(trikControl::Sensor3d *gyroscope);
    explicit GyroscopeWrap(emulators::GyroscopeEmulator * gyroscope);
    ~GyroscopeWrap();

private:
    bool isReal;
    trikControl::Sensor3d * realGyro;
    emulators::GyroscopeEmulator * emulGyro;

signals:

public slots:
    const QVector<int> &read() const;

};
