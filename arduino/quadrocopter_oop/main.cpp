#include <QCoreApplication>

#include "Quadrocopter.h"
#include "../../trikRuntime/trikControl/include/trikControl/brick.h"

Quadrocopter * quadro;
trikControl::Brick * brick;

int main (int argc, char ** argv)
{
    const QString &a = QString("");
    QCoreApplication * core = new QCoreApplication(argc, argv);
    brick = new trikControl::Brick(core->thread(), a);
    quadro = new Quadrocopter(brick);

    return core->exec();
}
