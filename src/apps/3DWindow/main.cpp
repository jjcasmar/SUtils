#include "../../core/mainwindow.h"

#include <QApplication>

#include <Qt3DExtras>
#include <Qt3DRender>

int main(int argc, char **argv) {

    QApplication app(argc, argv);


    MainWindow window;


    Qt3DCore::QEntity *rootEntity = window.sceneRootEntity();

    //Create a simple cube
    Qt3DCore::QEntity *cubeEntity = new Qt3DCore::QEntity(rootEntity);
    Qt3DExtras::QCuboidMesh *cubeMesh = new Qt3DExtras::QCuboidMesh;
    Qt3DExtras::QPhongMaterial *material = new Qt3DExtras::QPhongMaterial;

    material->setAmbient(QColor(0,0,0));
    material->setDiffuse(QColor(255,0,0));

    cubeEntity->addComponent(cubeMesh);
    cubeEntity->addComponent(material);

    //Create a light
    Qt3DCore::QEntity *lightEntity = new Qt3DCore::QEntity(rootEntity);
    Qt3DRender::QPointLight *pointLight = new Qt3DRender::QPointLight;
    Qt3DCore::QTransform *lightTransform = new Qt3DCore::QTransform;
    lightTransform->setTranslation({0,1.5,5});
    lightEntity->addComponent(pointLight);
    lightEntity->addComponent(lightTransform);

    Qt3DRender::QCamera *camera = window.sceneCamera();
    camera->lens()->setPerspectiveProjection(45.0f, 16.0f/9.0f, 0.1f, 1000.0f);
    camera->setPosition(QVector3D(0, 1, 4.0f));
    camera->setViewCenter(QVector3D(0, 0, 0));


    window.show();
    return app.exec();
}
