######################################################################
# Automatically generated by qmake (2.01a) Thu Jan 2 00:10:43 2014
######################################################################

#TEMPLATE = app
#TARGET =
#DEPENDPATH += .
#INCLUDEPATH += .

QT       += core
QT       += gui

QMAKE_CXXFLAGS += -std=c++11

TRIKCONTROL_DIR = $$PWD/../../trikRuntime/trikControl
EMUL_DIR = $$PWD/../../routeBuilder-master/emulatorTest

TRIKCONTROL_BINDIR = $$PWD/../../trikRuntime/bin/x86-debug
EMUL_BINDIR = $$PWD/../../routeBuilder-master/bin

INCLUDEPATH = \
        $$TRIKCONTROL_DIR/include \
        $$EMUL_DIR

LIBS += -L$$EMUL_BINDIR -lemtest -L$$TRIKCONTROL_BINDIR -ltrikControl-x86-d

# Input
HEADERS += \
    ComplementaryFilter.h \
    DAC8512.h \
    Definitions.h \
    LowPassFilter.h \
    MotorController.h \
    MPU6050DMP.h \
    PID.h \
    Quadrocopter.h \
    RVector3D.h \
    TimerCount.h \
    PWMJoystick.h \
    matrix3.h \
    vector3.h \
    wrappers/encoderWrap.h \
    wrappers/gyroscopeWrap.h \
    wrappers/motorWrap.h \
    wrappers/robotWrap.h

SOURCES += \
    DAC8512.cpp \
    MotorController.cpp \
    MPU6050DMP.cpp \
    PID.cpp \
    Quadrocopter.cpp \
    QuadrocopterCorrections.cpp \
    QuadrocopterSensors.cpp \
    QuadrocopterSerial.cpp \
    RVector3D.cpp \
    TimerCount.cpp \
    PWMJoystick.cpp \
    main.cpp \
    LowPassFilter.cpp \
    ComplementaryFilter.cpp \
    matrix3.cpp \
    vector3.cpp \
    wrappers/encoderWrap.cpp \
    wrappers/gyroscopeWrap.cpp \
    wrappers/motorWrap.cpp \
    wrappers/robotWrap.cpp
