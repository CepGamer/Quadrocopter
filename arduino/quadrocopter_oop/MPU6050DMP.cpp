// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
/*
// Changelog:
//     2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//     2012-06-20 - improved FIFO overflow handling and simplified read process
//     2012-06-19 - completely rearranged DMP initialization code and simplification
//     2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//     2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//     2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                - add 3D math helper file to DMP6 example sketch
//                - add Euler output and Yaw/Pitch/Roll output formats
//     2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//     2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//     2012-05-30 - basic DMP initialization working
*/

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

//#include "I2Cdev.h"
//#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050DMP.h"
#include "TimerCount.h"
#include "Quadrocopter.h"
#include <math.h>

#include <QDebug>

bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

extern Quadrocopter *quadro;

MPU6050DMP::MPU6050DMP(RobotWrapper *brck)
    : QObject(NULL)
{
#ifdef DEBUG_NO_MPU
    return;
#endif
    brick = brck;
    dmpReady = false;
    dcm.from_euler(0, 0, 0);
    av.x = av.y = av.z = 0;
}

MPU6050DMP::~MPU6050DMP()
{

}

void dmpDataReady()
{
#ifdef DEBUG_NO_MPU
    return;
#endif
#ifdef _arch_avr_
    interrupts();
#endif
    mpuInterrupt = true;
//    quadro->MPUInterrupt();
}

double* MPU6050DMP::getAngleXYZ()
{
    tdouble[0] = +ypr.z;
    tdouble[1] = +ypr.y;
    tdouble[2] = +ypr.x;
    return(tdouble);
}

double *MPU6050DMP::getAngularVelocityXYZ()
{
    tdouble[0] = av[0] * gyroMulConstRad;
    tdouble[1] = av[1] * gyroMulConstRad;
    tdouble[2] = av[2] * gyroMulConstRad;
    return(tdouble);
}

//double *MPU6050DMP::getAccelXYZ()
//{
//    tdouble[0] = acc[0];
//    tdouble[1] = acc[1];
//    tdouble[2] = acc[2];
//    return(tdouble);
//}

void MPU6050DMP::attachFIFOInterrupt()
{
#ifdef DEBUG_NO_MPU
    return;
#endif
    // enable Arduino interrupt detection
#ifdef _arch_avr_
    attachInterrupt(0, dmpDataReady, RISING);
#endif
#ifdef _arch_arm_
    attachInterrupt(2, dmpDataReady, RISING);
#endif
//    mpuIntStatus = mpu.getIntStatus();

}

int MPU6050DMP::bytesAvailableFIFO()
{
#ifdef DEBUG_NO_MPU
    return 0;
#endif
//    return(mpu.getFIFOCount());
}

void MPU6050DMP::resetNewData()
{
#ifdef DEBUG_NO_MPU
    return;
#endif
    newData = true;
}

bool MPU6050DMP::getNewData()
{
    return(newData);
}

void MPU6050DMP::resetFIFO()
{
#ifdef DEBUG_NO_MPU
    return;
#endif
    if(dmpReady);
        //mpu.flushFIFOBytes(mpu.getFIFOCount());
//        mpu.resetFIFO();
}

int MPU6050DMP::getPacketSize()
{
    return(packetSize);
}

void MPU6050DMP::initialize()
{
#ifdef DEBUG_NO_MPU
    return;
#endif
#ifdef DEBUG_DAC
    #ifdef _arch_avr_
        myLed = InfoLED(A0, InfoLED::DAC_8512);
    #endif
    #ifdef _arch_arm_
        myLed = InfoLED(0, InfoLED::DAC_ONBOARD);
    #endif
#endif

    // reset YPR data
    ypr.zero();
    dmpReady = true;

/*
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize device
    mpu.initialize();

    // load and configure the DMP
    devStatus = mpu.dmpInitialize();
    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        attachFIFOInterrupt();

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;

        //Serial.print("MPU init ok\n");

#ifdef USE_MPU_BYPASS
        mpu.setI2CBypassEnabled(true);
#endif
    }
    //else Serial.print("MPU init failed\n");
    */
    newData = true;
}

bool MPU6050DMP::notBusy()
{
#ifdef DEBUG_NO_MPU
    return false;
#endif
    return(!mpuInterrupt && fifoCount < packetSize);
}

void MPU6050DMP::processInterrupt()
{
#ifdef DEBUG_NO_MPU
    return;
#endif
    newData = true;
}

void MPU6050DMP::iteration(double dt)
{
#ifdef DEBUG_NO_MPU
    return;
#endif
    if(!dmpReady || dt > 0.2) return;

#ifdef DEBUG_DAC
    myLed.setState(20);
#endif
/*
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if(fifoCount % packetSize != 0) {
        // reset so we can continue cleanly
#ifdef MPUDEBUG
        Serial.println(F(" #OVERFLOW!# \n"));
#endif
#ifdef DEBUG_DAC
        myLed.setState(70);
#endif
        mpu.resetFIFO();

        // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else
    {
        // wait for correct available data length, should be a VERY short wait

        while(fifoCount > 0)
        {
            // read a packet from FIFO
            mpu.getFIFOBytes(fifoBuffer, packetSize);
            fifoCount -= packetSize;
        }
#ifdef DEBUG_DAC
        myLed.setState(70);
#endif
*/
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
//        mpu.dmpGetQuaternion(&q, fifoBuffer);
//        mpu.dmpGetGyro(av, fifoBuffer);
//        mpu.dmpGetGravity(&gravity, &q);
//        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
#ifdef DEBUG
    int speeds[4];
    for (int i = 0; i < 4; ++i) {
        speeds[i] = brick->motor(brick->powerMotorPorts()[i])->power();
    }
    QVector<int> x = brick->gyroscope()->readRenewed(speeds);
#else
    QVector<int> x = brick->gyroscope()->read();
#endif

    av.x = x[0] / gyroMulConstRad * dt;
    av.y = x[1] / gyroMulConstRad * dt;
    av.z = x[2] / gyroMulConstRad * dt;

    dcm.rotate(av);

    from_rotation_matrix();
    dmpGetGravity();
    dmpGetYawPitchRoll();

#ifdef MPUDEBUG
        getAngleXYZ();
        for(int i = 0; i < 3; i++)
        {
            if(tdouble[i] > 0) Serial.print("+");
            Serial.print(tdouble[i]);
            Serial.print("\t");
        }
        getAngularVelocityXYZ();
        for(int i = 0; i < 3; i++)
        {
            if(tdouble[i] > 0) Serial.print("+");
            Serial.print(tdouble[i]);
            Serial.print("\t");
        }
        Serial.print("\n");
#endif
        newData = true;

//    }
#ifdef DEBUG_DAC
    myLed.setState(70);
#endif
}

//  Code from: http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm
void MPU6050DMP::from_rotation_matrix()
{
    double tr = dcm.a.x + dcm.b.y + dcm.c.z;

    if (tr > 0) {
        double S = sqrtf(tr+1) * 2;
        q.setScalar(0.25f * S);
        q.setX((dcm.c.y - dcm.b.z) / S);
        q.setY((dcm.a.z - dcm.c.x) / S);
        q.setZ((dcm.b.x - dcm.a.y) / S);
    } else if ((dcm.a.x > dcm.b.y) && (dcm.a.x > dcm.c.z)) {
        double S = sqrtf(1.0 + dcm.a.x - dcm.b.y - dcm.c.z) * 2;
        q.setScalar((dcm.c.y - dcm.b.z) / S);
        q.setX(0.25f * S);
        q.setY((dcm.a.y + dcm.b.x) / S);
        q.setZ((dcm.a.z + dcm.c.x) / S);
    } else if (dcm.b.y > dcm.c.z) {
        double S = sqrtf(1.0 + dcm.b.y - dcm.a.x - dcm.c.z) * 2;
        q.setScalar((dcm.a.z - dcm.c.x) / S);
        q.setX((dcm.a.y + dcm.b.x) / S);
        q.setY(0.25f * S);
        q.setZ((dcm.b.z + dcm.c.y) / S);
    } else {
        double S = sqrtf(1.0 + dcm.c.z - dcm.a.x - dcm.b.y) * 2;
        q.setScalar((dcm.b.x - dcm.a.y) / S);
        q.setX((dcm.a.z + dcm.c.x) / S);
        q.setY((dcm.b.z + dcm.c.y) / S);
        q.setZ(0.25f * S);
    }
}

void MPU6050DMP::dmpGetGravity() {
//    gravity.x = 2 * (q.x()*q.z() - q.scalar()*q.y());
//    gravity.y = 2 * (q.scalar()*q.x() + q.y()*q.z());
//    gravity.z = q.scalar()*q.scalar() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z();
}

void MPU6050DMP::dmpGetYawPitchRoll() {
    dcm.to_euler(&ypr.x, &ypr.y, &ypr.z);
//    // yaw: (about Z axis)
//    ypr.x = atan2(2 * q.x() * q.y() - 2 * q.scalar() * q.z(), 2 * q.scalar() * q.scalar() + 2 * q.x() * q.x() - 1);
//    // pitch: (nose up/down, about Y axis)
//    ypr.y = atan(gravity.x / sqrt(gravity.y * gravity.y + gravity.z * gravity.z));
//    // roll: (tilt left/right, about X axis)
//    ypr.z = atan(gravity.y / sqrt(gravity.x * gravity.x + gravity.z * gravity.z));
}
