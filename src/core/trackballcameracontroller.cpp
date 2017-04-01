#include "trackballcameracontroller.h"
#include "trackballhelper.h"

#include <Qt3DCore/QTransform>

TrackballCameraController::TrackballCameraController(Qt3DRender::QCamera *camera)
{
    m_controlledCamera = camera;
}

TrackballCameraController::~TrackballCameraController()
{

}

void TrackballCameraController::wheelEvent(QWheelEvent *ev)
{
    QVector3D eyePosition = m_controlledCamera->position();
    QVector3D targetView = m_controlledCamera->viewCenter();
    float targetDistance = (eyePosition - targetView).length();
    m_controlledCamera->translate(QVector3D(0.0,0.0,-0.5*targetDistance*ev->angleDelta().y()/120.0),
                                  Qt3DRender::QCamera::DontTranslateViewCenter);
}

void TrackballCameraController::mousePressEvent(QMouseEvent *ev)
{
    m_mouseButtonsPressed = m_mouseButtonsPressed | ev->button();
    m_mouseLastPosition = ev->pos();
}

void TrackballCameraController::mouseReleaseEvent(QMouseEvent *ev)
{
    m_mouseButtonsPressed = m_mouseButtonsPressed ^ ev->button();
}

void TrackballCameraController::mouseMoveEvent(QSize screenSize, QMouseEvent *ev)
{
    QPoint mouseCurrentPos = ev->pos();
    QPoint deltaPos = m_mouseLastPosition - mouseCurrentPos;
    if (m_mouseButtonsPressed & Qt::MidButton) {
        m_controlledCamera->translate({0.005*deltaPos.x(),
                                       -0.005*deltaPos.y(),
                                       0
                                      },
                                      Qt3DRender::QCamera::TranslateViewCenter);
    }

    if (m_mouseButtonsPressed & Qt::LeftButton) {
        QQuaternion rotation = TrackBallHelper::rotation(m_mouseLastPosition,
                                                         mouseCurrentPos,
                                                         screenSize,
                                                         //QPoint(0,0),
                                                         QPoint(screenSize.width()/2,screenSize.height()/2),
                                                         1.0);

        QQuaternion currentRotation = m_controlledCamera->transform()->rotation();
        QQuaternion currentRotationInversed = currentRotation.conjugated();

        QVector3D rotatedAxis = currentRotationInversed.rotatedVector(rotation.vector());
        float angle = 1.3*rotation.scalar();

        m_controlledCamera->rotateAboutViewCenter(QQuaternion::fromAxisAndAngle(rotatedAxis,
                                                                                angle));
    }

    m_mouseLastPosition = mouseCurrentPos;
}
