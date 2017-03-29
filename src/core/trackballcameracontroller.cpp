#include "trackballcameracontroller.h"
#include "trackballhelper.h"

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

    qDebug() << "Target view: " <<targetView;
    qDebug() << "Eye position: " << eyePosition;
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
                                      Qt3DRender::QCamera::DontTranslateViewCenter);

        qDebug() << "Position: " << m_controlledCamera->position();
        qDebug() << "View center: " << m_controlledCamera->viewCenter();
    }

    if (m_mouseButtonsPressed & Qt::LeftButton) {

        QVector3D eyePosition = m_controlledCamera->position();
        QVector3D targetView = m_controlledCamera->viewCenter();
        float targetDistance = (eyePosition - targetView).length();

        QQuaternion rotation = TrackBallHelper::rotation(m_mouseLastPosition,
                                                         mouseCurrentPos,
                                                         screenSize,
                                                         //QPoint(0,0),
                                                         QPoint(screenSize.width()/2,screenSize.height()/2),
                                                         targetDistance);

        m_controlledCamera->rotateAboutViewCenter(rotation);
    }

    m_mouseLastPosition = mouseCurrentPos;
}
