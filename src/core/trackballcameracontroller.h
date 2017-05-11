/****************************************************************************
**
** Copyright (C) 2017 Klaralvdalens Datakonsult AB (KDAB).
** Contact: http://www.qt-project.org/legal
**
** This file is part of the Qt3D module of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:LGPL3$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see http://www.qt.io/terms-conditions. For further
** information use the contact form at http://www.qt.io/contact-us.
**
** GNU Lesser General Public License Usage
** Alternatively, this file may be used under the terms of the GNU Lesser
** General Public License version 3 as published by the Free Software
** Foundation and appearing in the file LICENSE.LGPLv3 included in the
** packaging of this file. Please review the following information to
** ensure the GNU Lesser General Public License version 3 requirements
** will be met: https://www.gnu.org/licenses/lgpl.html.
**
** GNU General Public License Usage
** Alternatively, this file may be used under the terms of the GNU
** General Public License version 2.0 or later as published by the Free
** Software Foundation and appearing in the file LICENSE.GPL included in
** the packaging of this file. Please review the following information to
** ensure the GNU General Public License version 2.0 requirements will be
** met: http://www.gnu.org/licenses/gpl-2.0.html.
**
** $QT_END_LICENSE$
**
****************************************************************************/

#ifndef QTRACKBALLCAMERACONTROLLER_H
#define QTRACKBALLCAMERACONTROLLER_H

#include <Qt3DCore/QEntity>
#include <Qt3DExtras/qt3dextras_global.h>
#include <QPoint>
#include <QSize>

QT_BEGIN_NAMESPACE

namespace Qt3DRender {
class QCamera;
}

namespace Qt3DLogic {
class QFrameAction;
}

namespace Qt3DInput {

class QKeyboardDevice;
class QMouseDevice;
class QMouseHandler;
class QLogicalDevice;
class QAction;
class QActionInput;
class QAxis;
class QAnalogAxisInput;
class QButtonAxisInput;
class QAxisActionHandler;

}

namespace Qt3DExtras {

class QTrackballCameraControllerPrivate;

class QT3DEXTRASSHARED_EXPORT QTrackballCameraController : public Qt3DCore::QEntity
{
    Q_OBJECT
    Q_PROPERTY(Qt3DRender::QCamera *camera READ camera WRITE setCamera NOTIFY cameraChanged)
    Q_PROPERTY(float panSpeed READ panSpeed WRITE setPanSpeed NOTIFY panSpeedChanged)
    Q_PROPERTY(float zoomSpeed READ zoomSpeed WRITE setZoomSpeed NOTIFY zoomSpeedChanged)
    Q_PROPERTY(float rotationSpeed READ rotationSpeed WRITE setRotationSpeed NOTIFY rotationSpeedChanged)
    Q_PROPERTY(float zoomCameraLimit READ zoomCameraLimit WRITE setZoomCameraLimit NOTIFY zoomCameraLimitChanged)
    Q_PROPERTY(float trackballRadius READ trackballRadius WRITE setTrackballRadius NOTIFY trackballRadiusChanged)
    Q_PROPERTY(QPoint trackballCenter READ trackballCenter WRITE setTrackballCenter NOTIFY trackballCenterChanged)
    Q_PROPERTY(QSize windowSize READ windowSize WRITE setWindowSize NOTIFY windowSizeChanged)
public:
    explicit QTrackballCameraController(Qt3DCore::QNode *parent = nullptr);
    ~QTrackballCameraController();

    Qt3DRender::QCamera *camera() const;

    void setCamera(Qt3DRender::QCamera *camera);

    float panSpeed() const;
    float zoomSpeed() const;
    float rotationSpeed() const;
    float zoomCameraLimit() const;
    float trackballRadius() const;
    QPoint trackballCenter() const;
    QSize windowSize() const;

    void setPanSpeed(float v);
    void setZoomSpeed(float v);
    void setRotationSpeed(float v);
    void setZoomCameraLimit(float v);
    void setTrackballRadius(float v);
    void setTrackballCenter(const QPoint &v);
    void setWindowSize(const QSize &v);

Q_SIGNALS:
    void cameraChanged();
    void panSpeedChanged();
    void zoomSpeedChanged();
    void rotationSpeedChanged();
    void zoomCameraLimitChanged();
    void trackballRadiusChanged();
    void trackballCenterChanged();
    void windowSizeChanged();

private:
    void init();
    QQuaternion createRotation(const QPoint &firstPoint,
                               const QPoint &nextPoint, const QSize &windowSize, const QPoint &trackballCenter,
                               float trackballRadius) const;

    QVector3D projectScreenToTrackball(const QPoint &screenCoords,
                                       const QSize &windowSize, const QPoint &trackballCenter,
                                       float trackballRadius) const;


    void onTriggered(float);

private:
    Qt3DRender::QCamera *m_camera;

    Qt3DInput::QAction *m_midMouseButtonAction;
    Qt3DInput::QAction *m_leftMouseButtonAction;
    Qt3DInput::QAction *m_altKeyAction;
    Qt3DInput::QAction *m_changeProjectionAction;

    Qt3DInput::QAxis *m_panXAxis;
    Qt3DInput::QAxis *m_panYAxis;
    Qt3DInput::QAxis *m_wheelAxis;

    Qt3DInput::QActionInput *m_midMouseButtonInput;
    Qt3DInput::QActionInput *m_altKeyInput;
    Qt3DInput::QActionInput *m_changeProjectionTypeInput;

    Qt3DInput::QAnalogAxisInput *m_panXInput;
    Qt3DInput::QAnalogAxisInput *m_panYInput;
    Qt3DInput::QAnalogAxisInput *m_wheelXInput;
    Qt3DInput::QAnalogAxisInput *m_wheelYInput;

    Qt3DInput::QMouseDevice *m_mouseDevice;
    Qt3DInput::QKeyboardDevice *m_keyboardDevice;
    Qt3DInput::QMouseHandler *m_mouseHandler;
    Qt3DInput::QLogicalDevice *m_logicalDevice;
    Qt3DLogic::QFrameAction *m_frameAction;

    QPoint m_mouseLastPosition;
    QPoint m_mousePressedPosition;
    QPoint m_mouseCurrentPosition;

    QSize m_windowSize;
    QPoint m_trackballCenter;
    float m_trackballRadius;


    //Movement speed control
    float m_panSpeed;
    float m_zoomSpeed;
    float m_rotationSpeed;
    float m_zoomCameraLimit;
};

} // Qt3DExtras

QT_END_NAMESPACE

#endif // QTRACKBALLCAMERACONTROLLER_H
