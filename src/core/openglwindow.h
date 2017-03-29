#ifndef OPENGLWINDOW_H
#define OPENGLWINDOW_H

#include <QWindow>
#include <Qt3DExtras/Qt3DWindow>

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
    void takeVideo(const QString &directory);

protected:
    Qt3DRender::QRenderCapture *m_renderCaptureFrameGraph;
    Qt3DRender::QRenderCaptureReply *m_renderCaptureReply;

    TrackballCameraController *m_trackballController;

    // QWindow interface
protected:
    void wheelEvent(QWheelEvent *ev);
    void mousePressEvent(QMouseEvent *ev);
    void mouseReleaseEvent(QMouseEvent *ev);
    void mouseMoveEvent(QMouseEvent *ev);
};

#endif // OPENGLWINDOW_H
