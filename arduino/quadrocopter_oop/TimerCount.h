#include "Definitions.h"
#include <QTime>

#ifndef TIMERCOUNT_H
#define TIMERCOUNT_H

class TimerCount
{
    private:
        QTime time;
        bool timeIsset;
    public:
        TimerCount();

        void setTime();
        
        unsigned long getTimeDifference();

        double getTimeDifferenceSeconds();
        
        bool getTimeIsset();
};

#endif
