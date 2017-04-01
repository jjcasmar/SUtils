#ifndef OPENGLWINDOW_H
#define OPENGLWINDOW_H

#include <QWindow>
#include <Qt3DExtras/Qt3DWindow>
#include <Qt3DLogic/QFrameAction>

#include "trackballcameracontroller.h"

namespace Qt3DRender {
class QRenderCapture;
class QRenderCaptureReply;
}

class OpenGLWindow : public Qt3DExtras::Qt3DWindow
{
    Q_OBJECT
public:
    OpenGLWindow();
    ~OpenGLWindow();

    void takeImage(const QString &filename);
    void saveImage();

    Qt3DCore::QEntity *rootEntity() const;

Q_SIGNALS:
    void imageTaken();
    void frameTick(float dt);

protected:
    Qt3DCore::QEntity *m_rootEntity;
    Qt3DCore::QEntity *m_internalRootEntity;
    Qt3DLogic::QFrameAction *m_frameAction;
    Qt3DRender::QRenderCapture *m_renderCaptureFrameGraph;
    Qt3DRender::QRenderCaptureReply *m_renderCaptureReply;
    QMetaObject::Connection *m_renderCaptureReplyConnection;

    TrackballCameraController *m_trackballController;
    uint m_imageCounter;

    // QWindow interface
protected:
    void wheelEvent(QWheelEvent *ev);
    void mousePressEvent(QMouseEvent *ev);
    void mouseReleaseEvent(QMouseEvent *ev);
    void mouseMoveEvent(QMouseEvent *ev);


};

#endif // OPENGLWINDOW_H
