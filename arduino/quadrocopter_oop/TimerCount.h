#include "Definitions.h"
#include <QElapsedTimer>

#ifndef TIMERCOUNT_H
#define TIMERCOUNT_H

class TimerCount
{
    private:
        QElapsedTimer time;
        bool timeIsset;
    public:
        TimerCount();

        void setTime();
        
        unsigned long getTimeDifference();

        double getTimeDifferenceSeconds();
        
        bool getTimeIsset();
};

#endif
