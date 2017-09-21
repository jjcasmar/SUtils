#include <QApplication>

#include <Qt3DExtras>
#include <Qt3DRender>
#include <QDebug>

#include "../../core/cadmaterial.h"
#include "../../core/trackballcameracontroller.h"

int main(int argc, char **argv) {

    QApplication app(argc, argv);

    Qt3DExtras::Qt3DWindow w;

    Qt3DCore::QEntity *cubeEntity = new Qt3DCore::QEntity;
    Qt3DExtras::QCuboidMesh *cubeMesh = new Qt3DExtras::QCuboidMesh;
    CADMaterial *material = new CADMaterial;
    material->setMaterialEffects(CADMaterial::Wireframe | CADMaterial::Phong);

    cubeEntity->addComponent(cubeMesh);
    cubeEntity->addComponent(material);

    Qt3DExtras::QTrackballCameraController *controller = new Qt3DExtras::QTrackballCameraController(cubeEntity);
    controller->setCamera(w.camera());
    w.camera()->setPosition(QVector3D(5,0,0));
    w.camera()->setViewCenter(QVector3D(0,0,0));
    w.camera()->setUpVector(QVector3D(0,1,0));

    Qt3DRender::QDirectionalLight *light = new Qt3DRender::QDirectionalLight;
    light->setColor(QColor("white"));
    w.camera()->addComponent(light);
    light->setWorldDirection(-w.camera()->position() + w.camera()->viewCenter());

    w.setRootEntity(cubeEntity);

    w.show();

    return app.exec();
}
