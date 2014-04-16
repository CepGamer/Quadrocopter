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
    return(time.elapsed());
}

double TimerCount::getTimeDifferenceSeconds()
{
    return(getTimeDifference() / 1.E3);
}

bool TimerCount::getTimeIsset()
{
    return(timeIsset);
}
