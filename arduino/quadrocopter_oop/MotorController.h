#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include "RVector3D.h"
//  Заменил их мотор на наш.
#include "wrappers/robotWrap.h"
#include "wrappers/motorWrap.h"

#include "Definitions.h"

class MotorController
{
private:

#ifdef DEBUG_NO_MOTORS
    static constexpr int INIT_TIMEOUT = 0; // ms
#else
    static constexpr int INIT_TIMEOUT = 1000; // ms
#endif

    double force;

    bool initialized;
    
    static constexpr double MIN_SPEED = 0.1;

    enum MOTORS
    {
        A, B, C, D, N_MOTORS
    };

//#ifdef PID_USE_YAW
    //1 if AV ^
    int direction[N_MOTORS];
//#endif


    bool useMotors[N_MOTORS];

    RVector3D coordinatesOfMotors[N_MOTORS];

public:
    MotorController(RobotWrapper * brick, QStringList ports);
    ~MotorController();

    MotorWrap * motors_[N_MOTORS];

    double getSpeed(RVector3D torqueVec, int motor);

    //set raw speed
    void setMotors(double power[N_MOTORS]); // values in [0...1]

    //torques and force method
    void setTorque(RVector3D torqueVec); //torques
    void setForce(double a); //force

    double getForce();

    void initialize();

    void calibrate();
};

#endif
