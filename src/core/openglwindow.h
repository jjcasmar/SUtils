#ifndef OPENGLWINDOW_H
#define OPENGLWINDOW_H

#include <QWindow>
#include <Qt3DExtras/Qt3DWindow>
#include <Qt3DLogic/QFrameAction>

namespace Qt3DRender {
class QRenderCapture;
class QRenderCaptureReply;
}

namespace Qt3DExtras {
class QOrbitCameraController;
class QTrackballCameraController;
}

class OpenGLWindow : public Qt3DExtras::Qt3DWindow
{
    Q_OBJECT
public:
    OpenGLWindow(bool offScreenRendering = false);
    ~OpenGLWindow();

    void takeImage(const QString &filename);
    void saveImage();

    Qt3DCore::QEntity *rootEntity() const;
    Qt3DExtras::QTrackballCameraController *trackballCameraController() const;

Q_SIGNALS:
    void imageTaken();
    void frameTick(float dt);

protected:
    Qt3DCore::QEntity *m_rootEntity;
    Qt3DCore::QEntity *m_internalRootEntity;
    Qt3DLogic::QFrameAction *m_frameAction;
    Qt3DRender::QRenderCapture *m_renderCaptureFrameGraph;
    Qt3DRender::QRenderCaptureReply *m_renderCaptureReply;
    Qt3DExtras::QOrbitCameraController *m_orbitCameraController;
    Qt3DExtras::QTrackballCameraController *m_trackballCameraController;
    QMetaObject::Connection *m_renderCaptureReplyConnection;

    uint m_imageCounter;
    bool m_offScreenRendering;
};

#endif // OPENGLWINDOW_H
