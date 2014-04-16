#include "PWMJoystick.h"
//#include "PWMInput.h"

PWMJoystick::PWMJoystick()
{
#ifndef STABILIZATION
    PWMInit();
#endif
}

double PWMJoystick::getAV()
{
    double v = 0;
#ifndef STABILIZATION
    v = PWMGetValue(AVIndex) * AVCoeff;
    if(fabs(v) < AVCoeffMin) v = 0;
#endif
    return v;
}

double PWMJoystick::getAngleX()
{
#ifndef STABILIZATION
    return(PWMGetValue(AngleXIndex) * AngleCoeff);
#else
    return 0.0;
#endif
}

double PWMJoystick::getAngleY()
{
#ifndef STABILIZATION
    return(PWMGetValue(AngleYIndex) * AngleCoeff);
#else
    return 0.0;
#endif
}

double PWMJoystick::getPower()
{
#ifndef STABILIZATION
    return(PWMGetValue(PowerIndex));
#else
    return 0.0;
#endif
}
