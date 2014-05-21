#pragma once

#include <QtCore/QThread>

#include "trikControl/brick.h"
#include "../../../routeBuilder-master/emulatorTest/brickEmulator.h"

#include "motorWrap.h"
#include "encoderWrap.h"
#include "gyroscopeWrap.h"

class RobotWrapper : public QObject
{
	Q_OBJECT
public:
    explicit RobotWrapper(QThread *guiThread, bool isRobo);
	~RobotWrapper();

public slots:
	/// Retruns list of ports for motors of a given type.
	QStringList powerMotorPorts() const;

	/// Retruns list of ports for encoders
	QStringList encoderPorts() const;

	/// Returns reference to motor of a given type on a given port
	MotorWrap *motor(QString const &port);

	/// Returns reference to encoder on given port.
	EncoderWrap *encoder(QString const &port);

    /// Returns reference to gyroscope
    GyroscopeWrap *gyroscope();

protected:
	bool mHasRealRobot;
	trikControl::Brick *mBrick;
	emulators::BrickEmulator *mBrickEmulator;

	QHash<QString, MotorWrap *> mMotorWrappers;
	QHash<QString, EncoderWrap *> mEncoderWrappers;

    GyroscopeWrap * mGyro;

	void wrapRealDevices();
	void wrapEmulators();
};

