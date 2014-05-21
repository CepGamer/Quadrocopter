#include <QCoreApplication>

#include "Quadrocopter.h"
#include "../../trikRuntime/trikControl/include/trikControl/brick.h"

Quadrocopter * quadro;
trikControl::Brick * brick;

int main (int argc, char ** argv)
{
    bool isReal = false;
    QCoreApplication * core = new QCoreApplication(argc, argv);
    quadro = new Quadrocopter(core->thread(), isReal);

    return core->exec();
}
