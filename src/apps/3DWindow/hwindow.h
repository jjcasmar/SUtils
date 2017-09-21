#ifndef HWINDOW_H
#define HWINDOW_H

#include <Qt3DCore>
#include <Qt3DRender>
#include <Qt3DLogic>
#include <Qt3DExtras>

class HWindow : public QWindow {
    Q_OBJECT

public:
    HWindow (QWindow *parent = nullptr) :
        m_engine(new Qt3DCore::QAspectEngine()),
        m_renderAspect(new Qt3DRender::QRenderAspect()),
        m_framegraph(new Qt3DExtras::QForwardRenderer),
        m_rootEntity(new Qt3DCore::QEntity()),
        m_renderSettings(new Qt3DRender::QRenderSettings()),
        m_logicAspect(new Qt3DLogic::QLogicAspect()),
        m_frameAction(new Qt3DLogic::QFrameAction())
    {
        setSurfaceType(QWindow::OpenGLSurface);

        m_framegraph->setSurface(this);
        m_engine->registerAspect(m_renderAspect);
        m_engine->registerAspect(m_logicAspect);
        m_renderSettings->setActiveFrameGraph(m_framegraph);
        m_rootEntity->addComponent(m_renderSettings);
        m_frameAction->setParent(m_framegraph);
    }


    Qt3DRender::QRenderSettings *renderSettings() {return m_renderSettings;}
    Qt3DRender::QFrameGraphNode *frameGraph() {return m_framegraph;}
    void setScene(Qt3DCore::QEntity *scene) {m_scene = scene;}
    Qt3DCore::QEntity *rootEntity() {return m_rootEntity;}
    Qt3DLogic::QFrameAction *frameAction() {return m_frameAction;}

public Q_SLOTS:
    void shutdown() {
        m_engine->setRootEntity(Qt3DCore::QEntityPtr());
        m_scene->setParent((Qt3DCore::QNode*)nullptr);
    }

    void turnOn() {
        m_scene->setParent(m_rootEntity);
        m_engine->setRootEntity(Qt3DCore::QEntityPtr(m_rootEntity));
    }

public:
    Qt3DCore::QAspectEngine *m_engine;
    Qt3DRender::QRenderAspect *m_renderAspect;
    Qt3DExtras::QForwardRenderer *m_framegraph;
    Qt3DCore::QEntity *m_rootEntity;
    Qt3DLogic::QLogicAspect *m_logicAspect;
    Qt3DLogic::QFrameAction *m_frameAction;

    Qt3DCore::QEntity *m_scene;
    Qt3DRender::QRenderSettings *m_renderSettings;
};

#endif // HWINDOW_H
