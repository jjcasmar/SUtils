#include "../../core/mainwindow.h"

#include <QApplication>

#include <Qt3DExtras>
#include <Qt3DRender>

#include "../../core/wireframematerial.h"

int main(int argc, char **argv) {

    QApplication app(argc, argv);

    QSurfaceFormat format;
    format.setProfile(QSurfaceFormat::CoreProfile);
    format.setMajorVersion(3);
    format.setMinorVersion(1);
    QSurfaceFormat::setDefaultFormat(format);
    MainWindow window;


    Qt3DCore::QEntity *rootEntity = window.sceneRootEntity();

    //Create a simple cube
    Qt3DCore::QEntity *cubeEntity = new Qt3DCore::QEntity(rootEntity);
    Qt3DExtras::QCuboidMesh *cubeMesh = new Qt3DExtras::QCuboidMesh;
//    Qt3DExtras::QPhongMaterial *material = new Qt3DExtras::QPhongMaterial;
    Qt3DCore::QTransform *transform = new Qt3DCore::QTransform;

    WireframeMaterial *material = new WireframeMaterial;

    transform->setTranslation({1,0,0});

    cubeEntity->addComponent(cubeMesh);
    cubeEntity->addComponent(material);
    cubeEntity->addComponent(transform);

    Qt3DCore::QEntity *blankEntity = new Qt3DCore::QEntity(cubeEntity);

    Qt3DCore::QEntity *cubeEntity2 = new Qt3DCore::QEntity(blankEntity);
    Qt3DExtras::QCuboidMesh *cubeMesh2 = new Qt3DExtras::QCuboidMesh;
    Qt3DExtras::QPhongMaterial *material2 = new Qt3DExtras::QPhongMaterial;
    Qt3DCore::QTransform *transform2 = new Qt3DCore::QTransform;

    transform2->setTranslation({0,1,0});
    material2->setAmbient(QColor(0,0,0));
    material2->setDiffuse(QColor(0,255,0));

    cubeEntity2->addComponent(cubeMesh2);
    cubeEntity2->addComponent(material2);
    cubeEntity2->addComponent(transform2);

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
