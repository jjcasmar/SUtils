#include "openglwindow.h"

#include <Qt3DCore/QTransform>
#include <Qt3DRender/QRenderCapture>
#include <Qt3DRender/QRenderCaptureReply>
#include <Qt3DExtras/QPlaneMesh>
#include <Qt3DExtras/QPhongMaterial>
#include <Qt3DExtras/QForwardRenderer>
#include <Qt3DLogic/QFrameAction>
#include <Qt3DRender/QCameraSelector>
#include <Qt3DRender/QLayerFilter>
#include <Qt3DExtras>
#include <Qt3DRender>


OpenGLWindow::OpenGLWindow() :
    Qt3DWindow(),
    m_rootEntity(new Qt3DCore::QEntity),
    m_internalRootEntity(new Qt3DCore::QEntity),
    m_frameAction(new Qt3DLogic::QFrameAction),
    m_renderCaptureFrameGraph(new Qt3DRender::QRenderCapture)
{
    m_rootEntity->setParent(m_internalRootEntity);
    this->setRootEntity(m_internalRootEntity);
    m_rootEntity->addComponent(m_frameAction);

    //Frame action tick
    connect(m_frameAction, &Qt3DLogic::QFrameAction::triggered,
            this, &OpenGLWindow::frameTick);

    //Set the background gradient typical of cad apps
    //I will add a new renderview to perform this
    Qt3DRender::QFrameGraphNode *framegraph = new Qt3DRender::QFrameGraphNode();

    Qt3DRender::QClearBuffers *backgroundClearBuffers = new Qt3DRender::QClearBuffers(framegraph);
    backgroundClearBuffers->setBuffers(Qt3DRender::QClearBuffers::AllBuffers);

    //Background renderView
    Qt3DRender::QCameraSelector *backgroundCameraSelector = new Qt3DRender::QCameraSelector(backgroundClearBuffers);
    Qt3DRender::QLayerFilter *layerFilter = new Qt3DRender::QLayerFilter(backgroundCameraSelector);
    Qt3DRender::QViewport *viewport = new Qt3DRender::QViewport(layerFilter);
    //Qt3DRender::QRenderTargetSelector *renderTargetSelector = new Qt3DRender::QRenderTargetSelector(viewport);
    Qt3DRender::QRenderSurfaceSelector *renderSurfaceSelector = new Qt3DRender::QRenderSurfaceSelector(viewport);

    //Forward renderer renderView
    Qt3DExtras::QForwardRenderer *forwardRenderer = defaultFrameGraph();;
    //Find the forward renderer clearbuffers and adapt it
    Qt3DRender::QClearBuffers *forwardDefaultClearBuffers = forwardRenderer->findChild<Qt3DRender::QClearBuffers*>();
    forwardDefaultClearBuffers->setBuffers(Qt3DRender::QClearBuffers::DepthStencilBuffer);
    forwardRenderer->setParent(framegraph);

    renderSurfaceSelector->setSurface(this);
    viewport->setNormalizedRect(QRectF(0,0,1,1));

    Qt3DRender::QCamera *backgroundCamera = new Qt3DRender::QCamera(backgroundCameraSelector);
    backgroundCamera->lens()->setOrthographicProjection(0,1,0,1,0.1,10);
    backgroundCamera->setPosition({5000,5000,-9001});
    backgroundCamera->setViewCenter({5000,5000,-9000});
    backgroundCamera->setUpVector({0,1,0});
    backgroundCameraSelector->setCamera(backgroundCamera);

    Qt3DRender::QLayer *layer = new Qt3DRender::QLayer;
    layerFilter->addLayer(layer);

    Qt3DCore::QEntity *backgroundEntity = new Qt3DCore::QEntity(m_internalRootEntity);
    Qt3DExtras::QPlaneMesh *backgroundMesh = new Qt3DExtras::QPlaneMesh(backgroundEntity);
    Qt3DExtras::QPerVertexColorMaterial *backgroundMaterial = new Qt3DExtras::QPerVertexColorMaterial(backgroundEntity);
    Qt3DCore::QTransform *backgroundTransform = new Qt3DCore::QTransform(backgroundEntity);
    backgroundTransform->setRotation(QQuaternion::fromAxisAndAngle({1,0,0},-90));
    backgroundTransform->setTranslation({4999.5,5000.5,-9000});
    backgroundMesh->setWidth(1);
    backgroundMesh->setHeight(1);
    backgroundMesh->setMeshResolution(QSize(2,2));
    backgroundEntity->addComponent(backgroundMesh);
    backgroundEntity->addComponent(backgroundMaterial);
    backgroundEntity->addComponent(backgroundTransform);
    backgroundEntity->addComponent(layer);

    //Add a color attribute to the background
    Qt3DRender::QBuffer *colorBuffer = new Qt3DRender::QBuffer;
    QByteArray colorData(4*3*sizeof(float), 0);
    QVector3D *colorDataStream = reinterpret_cast<QVector3D*>(colorData.data());
    colorDataStream[0] = QVector3D(166/255.0,166/255.0,189/255.0);
    colorDataStream[1] = QVector3D(166/255.0,166/255.0,189/255.0);
    colorDataStream[2] = QVector3D(51/255.0,51/255.0,101/255.0);
    colorDataStream[3] = QVector3D(51/255.0,51/255.0,101/255.0);
    colorBuffer->setData(colorData);

    Qt3DRender::QAttribute *colorAttribute = new Qt3DRender::QAttribute;
    colorAttribute->setName(Qt3DRender::QAttribute::defaultColorAttributeName());
    colorAttribute->setDataType(Qt3DRender::QAttribute::Float);
    colorAttribute->setBuffer(colorBuffer);
    colorAttribute->setVertexSize(3);
    colorAttribute->setByteStride(3*sizeof(float));
    colorAttribute->setCount(4);
    backgroundMesh->geometry()->addAttribute(colorAttribute);


    //Add support for taking images
    //framegraph->setParent(m_renderCaptureFrameGraph);
    //m_renderCaptureFrameGraph->setParent(forwardRenderer);
    forwardRenderer->setParent(m_renderCaptureFrameGraph);
    m_renderCaptureFrameGraph->setParent(framegraph);
    setActiveFrameGraph(framegraph);

    //Set the camera control
    m_trackballController = new TrackballCameraController(camera());

    //Set a grid floor
    /*
    Qt3DCore::QEntity *floorEntity = new Qt3DCore::QEntity(m_internalRootEntity);
    Qt3DExtras::QPlaneMesh *floorRenderer = new Qt3DExtras::QPlaneMesh(floorEntity);
    Qt3DExtras::QPhongMaterial *floorMaterial = new Qt3DExtras::QPhongMaterial(floorEntity);
    Qt3DCore::QTransform *floorTransform = new Qt3DCore::QTransform(floorEntity);
    floorRenderer->setWidth(100);
    floorRenderer->setHeight(100);
    floorRenderer->setMeshResolution(QSize(2,2));
    floorMaterial->setDiffuse({50,50,50});
    floorMaterial->setShininess(0);
    floorEntity->addComponent(floorRenderer);
    floorEntity->addComponent(floorMaterial);
    */
}

OpenGLWindow::~OpenGLWindow()
{

}

void OpenGLWindow::takeImage(const QString &filename)
{
    Qt3DRender::QRenderCaptureReply *captureReply = m_renderCaptureFrameGraph->requestCapture(m_imageCounter++);
    QObject::connect(captureReply, &Qt3DRender::QRenderCaptureReply::completeChanged,
                                                 [this, captureReply, filename](bool isComplete) {
        if (isComplete)
            captureReply->saveToFile(filename);
            captureReply->deleteLater();
            this->imageTaken();
    });
}

Qt3DCore::QEntity *OpenGLWindow::rootEntity() const
{
    return m_rootEntity;
}

void OpenGLWindow::wheelEvent(QWheelEvent *ev)
{
    m_trackballController->wheelEvent(ev);
}

void OpenGLWindow::mousePressEvent(QMouseEvent *ev)
{
    m_trackballController->mousePressEvent(ev);
}

void OpenGLWindow::mouseReleaseEvent(QMouseEvent *ev)
{
    m_trackballController->mouseReleaseEvent(ev);
}

void OpenGLWindow::mouseMoveEvent(QMouseEvent *ev)
{
    m_trackballController->mouseMoveEvent(size(), ev);
}
