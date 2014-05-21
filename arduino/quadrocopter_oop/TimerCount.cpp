//#include <Arduino.h>
#include "TimerCount.h"

TimerCount::TimerCount()
{
    timeIsset = false;
}

void TimerCount::setTime()
{
    time.start();
    timeIsset = true;
}

unsigned long TimerCount::getTimeDifference()
{
    unsigned long toRet = time.nsecsElapsed();
    return(toRet);
}

double TimerCount::getTimeDifferenceSeconds()
{
    double toRet = (double)getTimeDifference();
    toRet /= (double)1.E9;
    return toRet;
}

bool TimerCount::getTimeIsset()
{
    return(timeIsset);
}
