#ifndef TRACKBALLCAMERACONTROLLER_H
#define TRACKBALLCAMERACONTROLLER_H

#include <QWheelEvent>
#include <Qt3DRender/QCamera>
#include <QQuaternion>

class TrackballCameraController
{
public:
    TrackballCameraController(Qt3DRender::QCamera *camera);
    ~TrackballCameraController();

    void wheelEvent(QWheelEvent *ev);
    void mousePressEvent(QMouseEvent *ev);
    void mouseReleaseEvent(QMouseEvent *ev);
    void mouseMoveEvent(QSize screenSize, QMouseEvent *ev);

protected:
    Qt3DRender::QCamera *m_controlledCamera;
    Qt::MouseButtons m_mouseButtonsPressed;
    QPoint m_mouseLastPosition;

    QQuaternion m_lastRotation;
};

#endif // TRACKBALLCAMERACONTROLLER_H
