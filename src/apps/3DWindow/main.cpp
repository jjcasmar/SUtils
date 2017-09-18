#include <QApplication>

#include <Qt3DExtras>
#include <Qt3DRender>
#include <QDebug>

#include "hwindow.h"

int main(int argc, char **argv) {

    QApplication app(argc, argv);

    Qt3DCore::QEntity *cubeEntity = new Qt3DCore::QEntity;
    cubeEntity->addComponent(new Qt3DExtras::QCuboidMesh);
    auto material = new Qt3DExtras::QPhongMaterial;
    material->setAmbient(QColor("red"));
    cubeEntity->addComponent(material);

    HWindow *window = new HWindow;
    HWindow *window2 = new HWindow;

    window->setScene(cubeEntity);
    window2->setScene(cubeEntity);

    auto frameAction1 = window->frameAction();
    auto frameAction2 = window2->frameAction();
    QObject::connect(frameAction1, &Qt3DLogic::QFrameAction::triggered, window, &HWindow::shutdown);
    QObject::connect(frameAction1, &Qt3DLogic::QFrameAction::triggered, window2, &HWindow::turnOn);
    QObject::connect(frameAction2, &Qt3DLogic::QFrameAction::triggered, window2, &HWindow::shutdown);
    QObject::connect(frameAction2, &Qt3DLogic::QFrameAction::triggered, window, &HWindow::turnOn);

    window->turnOn();
//    window2->turnOn();

    window->show();
    window2->show();
    return app.exec();
}
