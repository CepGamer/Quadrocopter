#include <QCoreApplication>

#include "Quadrocopter.h"
#include "../../trikRuntime/trikControl/include/trikControl/brick.h"

int main (int argc, char ** argv)
{
    QCoreApplication *core = new QCoreApplication(argc, argv);

    bool isReal = false;
    Quadrocopter *quadro = new Quadrocopter(core->thread(), isReal);

    return core->exec();
}
