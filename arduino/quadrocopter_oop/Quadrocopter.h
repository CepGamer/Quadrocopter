#ifndef QUADROCOPTER_H
#define QUADROCOPTER_H

#include "Definitions.h"
#include "MPU6050DMP.h"
#include "RVector3D.h"
#include "TimerCount.h"
//#include "Motor.h"
#include "MotorController.h"
//#include "MySerial.h"
#include "PID.h"
//#include "InfoLED.h"
//#include "VoltageSensor.h"
#include "PWMJoystick.h"
#include "LowPassFilter.h"

//#include "../../trikRuntime/trikControl/include/trikControl/brick.h"
#include "wrappers/robotWrap.h"

#include <QString>
#include <QObject>

#ifdef USE_COMPASS
    // i2cdevlib
    #include <HMC5883L.h>
#endif

#define BN 60

//  reaction type (different types of processing sensors' data)
enum reactionType_ {ReactionNone, ReactionAngularVelocity, ReactionAcceleration, ReactionAngle};

const int msec = 1000;
const int frames = 25;

class Quadrocopter : public QObject
{
    Q_OBJECT
private:
    MotorController * MController;
    TimerCount DeltaT;
//    MySerial* MSerial;
//    VoltageSensor* VSensor;
    MPU6050DMP* MyMPU;

    RobotWrapper * controller;

#ifdef USE_COMPASS
    HMC5883L* MyCompass;
#endif
    PWMJoystick* Joystick;

    //  pins configuration
    //  Номера портов, по которым подключены моторы и сенсор
    //  Меняем пины моторов на силовые/серво порты
    QStringList DefaultMotorPins;
    QTimer * mTimer;
    QTimer * logTimer;
    int DefaultVSensorPin;

    QFile * logFile;
    QString logMessage;

    reactionType_ reactionType;

    //  torque corrections
    RVector3D torqueAutomaticCorrection;
    RVector3D angleManualCorrection;

    static constexpr double DefaultVSensorMaxVoltage = 17.95;   //  maximal voltage (before voltage divider)
    //  15.16? 11.95->2.6

    static constexpr double g = 9.80665;    // gravitational acceleration

    //physical quantities
    RVector3D angle;            // angle between Earth's coordinate and ours (filtered)
    RVector3D angularVelocity;  // angular velocity from gyroscope
    double voltage;             // accumulators voltage (не использована)

#ifdef USE_COMPASS
    double copterHeading;
    double joystickHeading;
    RVector3D BMag;

    //temp mag variables
    int16_t magX, magY, magZ;
#endif

    //corrections
    static constexpr double angleMaxCorrection = MPI / 4;
    static constexpr double angularVelocityMaxCorrection = MPI / 4;

    PID pidAngleX, pidAngleY;

    bool flying;                //  Летит ли в данный момент квадрокоптер
    double flyingTime;          //  Сколько времени летит квадрокоптер

#ifdef PID_USE_YAW
    PID pidAngularVelocityZ;
#endif

#ifdef PID_USE_YAW_ANGLE
    PID pidAngleZ;
#endif

    RVector3D getAngleCorrection(RVector3D angle, double dt);

    double dt, dtMax, sensorsTime, calculationsTime;
    TimerCount tCount;

//    InfoLED myLed;

    double forceOverrideValue;
    bool forceOverride;

    RVector3D angleOffsetPC;

    LowPassFilter<double> angleZLPF;

#ifdef DEBUG_FREQ_PIN
//    InfoLED freqLed;
#endif

#ifdef DEBUG_MPUBYTES_PIN
//    InfoLED mpuBytesLed;
#endif

    // bytes to read
    // Defines in Quadrocopter.cpp
    unsigned int serialReadN;
    bool needPCTx;

public:
    Quadrocopter(QThread * thread, bool isRobot);
    ~Quadrocopter();

    void reset();

    void processSensorsData();
    void processCorrection();
    void processMotors();

    void processSerialGetCommand();
    void processSerialDoCommand();

    void processSerialPCRx();
    void processSerialPCTx();

    void processSerialTextTx();

    void processJoystickRx();

    void MPUInterrupt();

    RVector3D getTorques();

public slots:
    void iteration();

private slots:
    void log();

};

#endif // QUADROCOPTER_H
