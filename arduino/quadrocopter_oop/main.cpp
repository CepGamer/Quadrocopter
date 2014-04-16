#include <QCoreApplication>

#include "Quadrocopter.h"
#include "../../trikRuntime/trikControl/include/trikControl/brick.h"

Quadrocopter * quadro;
trikControl::Brick * brick;

int main (int argc, char ** argv)
{
    QCoreApplication * core = new QCoreApplication(argc, argv);
    quadro = new Quadrocopter();
    brick = new trikControl::Brick(core->thread(), "");
    while(1)
    {
        quadro->iteration();
    }
    return core->exec();
}
