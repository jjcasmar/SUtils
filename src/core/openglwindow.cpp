#include "openglwindow.h"

#include <Qt3DRender/QRenderCapture>
#include <Qt3DRender/QRenderCaptureReply>

OpenGLWindow::OpenGLWindow()
{
    //Add support for taking images
    Qt3DRender::QFrameGraphNode *framegraph = activeFrameGraph();
    m_renderCaptureFrameGraph = new Qt3DRender::QRenderCapture;

    framegraph->setParent(m_renderCaptureFrameGraph);
    setActiveFrameGraph(m_renderCaptureFrameGraph);

    m_trackballController = new TrackballCameraController(camera());
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
            //captureReply->deleteLater();
    });
}

void OpenGLWindow::takeVideo(const QString &directory)
{

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
