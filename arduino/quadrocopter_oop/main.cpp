#include <QCoreApplication>
#include "Quadrocopter.h"

int main (int argc, char ** argv)
{
    QCoreApplication * core = new QCoreApplication(argc, argv);
    Quadrocopter * quadro = new Quadrocopter();
    while(1)
    {
        quadro->iteration();
    }
    return core->exec();
}
