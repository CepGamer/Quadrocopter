#include "Quadrocopter.h"
//#include "Arduino.h"
#include "PID.h"
#ifdef _arch_avr_
    #include <avr/delay.h>
#endif

Quadrocopter::Quadrocopter(QThread *thread, bool isRobot)
    : QObject(NULL)
    , needPCTx(false)
    , flying(false)
#ifdef PID_USE_YAW_ANGLE
    , serialReadN(27)   // 3 + 12 + 6 + 6
#else
    , serialReadN(21)   // 3 + 12 + 6
#endif
    , reactionType(ReactionAngle)
    , forceOverrideValue(0)
    , forceOverride(true)
{
//    DefaultVSensorPin = A4;

    DefaultMotorPins.append(QString ("JM1"));
    DefaultMotorPins.append(QString ("JM2"));
    DefaultMotorPins.append(QString ("JM3"));
    DefaultMotorPins.append(QString ("JM4"));

#ifdef DEBUG_SERIAL_SECOND
    DEBUG_SERIAL_SECOND.begin(115200);
#endif

//    MSerial = new MySerial;
    controller = new RobotWrapper(thread, isRobot);
    MController = new MotorController(controller, DefaultMotorPins);
    //  Зачем сенсор вольтажа?
//    VSensor = new VoltageSensor(DefaultVSensorPin, DefaultVSensorMaxVoltage);
    MyMPU = new MPU6050DMP(controller);

    pidAngleX = PID(PID::DIFFERENCE_ANGLE);
    pidAngleY = PID(PID::DIFFERENCE_ANGLE);

#ifdef PID_USE_YAW_ANGLE
    pidAngleZ = PID(PID::DIFFERENCE_ANGLE);
#endif

#ifdef DEBUG_FREQ_PIN
//    freqLed = InfoLED(DEBUG_FREQ_PIN, InfoLED::DIGITAL);
#endif

#ifdef DEBUG_MPUBYTES_PIN
//    mpuBytesLed = InfoLED(DEBUG_MPUBYTES_PIN, InfoLED::DIGITAL);
#endif

//    MController->calibrate();

    reset();

    MyMPU->initialize();

#ifdef USE_COMPASS
    MyCompass = new HMC5883L;
    MyCompass->initialize();
#endif

    Joystick = new PWMJoystick();

    mTimer = new QTimer(this);
    mTimer->setInterval(msec / frames);
    QObject::connect(mTimer, SIGNAL(timeout()), this, SLOT(iteration()));
    mTimer->start();

    logTimer = new QTimer(this);
    logTimer->setInterval(msec);
    QObject::connect(logTimer, SIGNAL(timeout()), this, SLOT(log()));
    logTimer->start();

    logMessage.clear();

    logFile = new QFile("log.txt");
    logFile->open(QFile::WriteOnly);

#ifdef DEBUG_DAC
//    myLed = MyMPU->myLed;
//    myLed.setState(0);
#endif

#ifdef _arch_avr_
    interrupts();
#endif
}

Quadrocopter::~Quadrocopter()
{
//    delete Joystick;
//    delete mTimer;
}

void Quadrocopter::reset()
{
    angleZLPF.setPeriod(0.05);
    flyingTime = 0;
    flying = false;

    angleOffsetPC = RVector3D();
    angle = RVector3D();
    torqueAutomaticCorrection = RVector3D();
    angleManualCorrection = RVector3D();

    MController->setForce(0.58);
    MController->setTorque(RVector3D());

    pidAngleX.reset();
    pidAngleX.setYMinYMax(angleMaxCorrection);
    pidAngleX.setKpKiKd(1, 0.5, 0);
    pidAngleX.setPMinMax(100.0);
    pidAngleX.setIMinMax(100.0);
    pidAngleX.setDMinMax(100.0);

    pidAngleY.reset();
    pidAngleY.setYMinYMax(angleMaxCorrection);
    pidAngleY.setKpKiKd(1, 0.5, 0);
    pidAngleY.setPMinMax(100.0);
    pidAngleY.setIMinMax(100.0);
    pidAngleY.setDMinMax(100.0);


#ifdef PID_USE_YAW
    pidAngularVelocityZ.reset();
    pidAngularVelocityZ.setYMinYMax(angularVelocityMaxCorrection);
#endif

#ifdef PID_USE_YAW_ANGLE
    pidAngleZ.reset();
    pidAngleZ.setYMinYMax(angleMaxCorrection);
#endif

    voltage = 0;
    dtMax = 0;

    MyMPU->resetFIFO();
    logMessage.clear();
}

void Quadrocopter::processCorrection()
{
    torqueAutomaticCorrection = RVector3D();

    switch(reactionType)
    {
    //    case ReactionAngularVelocity:
    //        torqueAutomaticCorrection = getAngularVelocityCorrection(angularVelocity, DeltaT.getTimeDifferenceSeconds());
    //        break;

    //    case ReactionAcceleration:
    //        torqueAutomaticCorrection = getAccelerationCorrection(angle, accelDataRaw);
    //        break;

    case ReactionAngle:
        torqueAutomaticCorrection = getAngleCorrection(angle, DeltaT.getTimeDifferenceSeconds());
        break;
    }
}

void Quadrocopter::processMotors()
{
    if(MController->getForce() >= MINIMUM_FLYING_THROTTLE)
        flyingTime += dt;
    if(flyingTime >= MINIMUM_FLYING_TIME)
    {
        flying = true;
        MController->setTorque(getTorques());
    }
}

void Quadrocopter::iteration()
{
    if(dtMax < dt)
        dtMax = dt;

    if(MyMPU->getNewData()) // on each MPU data packet
    {

#ifdef DEBUG_FREQ_PIN
//    freqLed.changeDigitalState();
#endif

#ifdef DEBUG_MPUBYTES_PIN
//    mpuBytesLed.setState(MyMPU->bytesAvailableFIFO() > MyMPU->getPacketSize());
#endif

#ifdef DEBUG_DAC
//    myLed.setState(0);
#endif

        { // Serial
//            processSerialGetCommand();
//            myLed.setState(5);
//            processSerialDoCommand();
        }

#ifdef DEBUG_DAC
//        myLed.setState(20);
#endif

        tCount.setTime();
        // Sensors

        MyMPU->iteration(dt);
        processSensorsData();
#ifdef LOG
        double * temp = MyMPU->getAngleXYZ();
        QVector<int> tmp = controller->gyroscope()->read();
        for(int i = 0; i < 4; i++)
        {
            logMessage.append(QString::number(this->MController->motors_[i]->power()));
            logMessage.append("\t");
        }
        logMessage.append("\n");
        for(int i = 0; i < DIM; i++)
        {
            logMessage.append(QString::number(temp[i], 'f', 6));
            logMessage.append("\t");
        }
        logMessage.append("\n");
        for(int i = 0; i < DIM; i++)
        {
            logMessage.append(QString::number(tmp.at(i)));
            logMessage.append("\t");
        }

        logMessage.append("\n");
        logMessage.append(QString::number(dt, 'f', 6));
        logMessage.append("\n\n");
#endif
        sensorsTime = tCount.getTimeDifferenceSeconds();

#ifdef DEBUG_DAC
//        myLed.setState(80);
#endif


        { // Joystick

//            processJoystickRx();
        }

        tCount.setTime();
        // Corrections, Motors
        dt = DeltaT.getTimeDifferenceSeconds();

        processCorrection();
        DeltaT.setTime();
        processMotors();

        calculationsTime = tCount.getTimeDifferenceSeconds();


#ifdef DEBUG_DAC
//        myLed.setState(100);
#endif

        MyMPU->resetNewData();
    }
}

void Quadrocopter::log()
{
    logFile->write(logMessage.toUtf8().constData());
    logMessage.clear();
    logFile->flush();
}

void Quadrocopter::MPUInterrupt()
{
    MyMPU->processInterrupt();
}

RVector3D Quadrocopter::getTorques()
{
    return(torqueAutomaticCorrection);
}
