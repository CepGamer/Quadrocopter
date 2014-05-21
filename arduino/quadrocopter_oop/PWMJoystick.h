#include "Definitions.h"

#ifndef PWMJOYSTICK_H
#define PWMJOYSTICK_H

class PWMJoystick
{
    static const int AVIndex = 3, AngleXIndex = 0, AngleYIndex = 1, PowerIndex = 2;
    static constexpr double AVCoeff = 2.5, AngleCoeff = 0.7;
    static constexpr double AVCoeffMin = 0.25;
public:
    PWMJoystick();
    double getAV();
    double getAngleX();
    double getAngleY();
    double getPower();
};

#endif // PWMJOYSTICK_H
