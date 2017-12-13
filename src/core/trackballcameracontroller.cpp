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

#include "trackballcameracontroller.h"
#include "trackballcameracontroller_p.h"

#include <QVector3D>
#include <Qt3DCore/QTransform>
#include <Qt3DInput/QAction>
#include <Qt3DInput/QActionInput>
#include <Qt3DInput/QAnalogAxisInput>
#include <Qt3DInput/QAxis>
#include <Qt3DInput/QKeyboardDevice>
#include <Qt3DInput/QKeyboardHandler>
#include <Qt3DInput/QLogicalDevice>
#include <Qt3DInput/QMouseDevice>
#include <Qt3DInput/QMouseHandler>
#include <Qt3DLogic/QFrameAction>
#include <Qt3DRender/QCamera>
#include <QtCore/QtGlobal>

#include <math.h>

QT_BEGIN_NAMESPACE

namespace Qt3DExtras {

QTrackballCameraController::QTrackballCameraController(Qt3DCore::QNode *parent)
    : Qt3DCore::QEntity(parent), m_camera(nullptr),
      m_midMouseButtonAction(new Qt3DInput::QAction()),
      m_leftMouseButtonAction(new Qt3DInput::QAction()),
      m_altKeyAction(new Qt3DInput::QAction()),
      m_changeProjectionAction(new Qt3DInput::QAction()),
      m_panXAxis(new Qt3DInput::QAxis()), m_panYAxis(new Qt3DInput::QAxis()),
      m_wheelAxis(new Qt3DInput::QAxis()),
      m_midMouseButtonInput(new Qt3DInput::QActionInput()),
      m_altKeyInput(new Qt3DInput::QActionInput()),
      m_changeProjectionTypeInput(new Qt3DInput::QActionInput()),
      m_panXInput(new Qt3DInput::QAnalogAxisInput()),
      m_panYInput(new Qt3DInput::QAnalogAxisInput()),
      m_wheelXInput(new Qt3DInput::QAnalogAxisInput()),
      m_wheelYInput(new Qt3DInput::QAnalogAxisInput()),
      m_mouseDevice(new Qt3DInput::QMouseDevice()),
      m_keyboardDevice(new Qt3DInput::QKeyboardDevice()),
      m_mouseHandler(new Qt3DInput::QMouseHandler()),
      m_logicalDevice(new Qt3DInput::QLogicalDevice()),
      m_frameAction(new Qt3DLogic::QFrameAction()),
      m_windowSize(QSize(1920, 1080)), m_trackballRadius(0.5f), m_panSpeed(1.0),
      m_zoomSpeed(1.0), m_rotationSpeed(1.0), m_zoomCameraLimit(1.0) {
  init();
}

QTrackballCameraController::~QTrackballCameraController() {
  delete m_midMouseButtonAction;
  delete m_leftMouseButtonAction;
  delete m_altKeyAction;
  delete m_changeProjectionAction;
  delete m_panXAxis;
  delete m_panYAxis;
  delete m_wheelAxis;
  delete m_midMouseButtonInput;
  delete m_altKeyInput;
  delete m_changeProjectionTypeInput;
  delete m_panXInput;
  delete m_panYInput;
  delete m_wheelXInput;
  delete m_wheelYInput;
  delete m_mouseDevice;
  delete m_keyboardDevice;
  delete m_mouseHandler;
  delete m_logicalDevice;
  delete m_frameAction;
}

void QTrackballCameraController::init() {
  //// Actions

  // Mid button action (triggers rotation and panning)
  m_midMouseButtonInput->setButtons(QVector<int>() << Qt::MidButton);
  m_midMouseButtonInput->setSourceDevice(m_mouseDevice);
  m_midMouseButtonAction->addInput(m_midMouseButtonInput);

  // Alt key (triggers panning)
  m_altKeyInput->setButtons(QVector<int>() << Qt::Key_Shift);
  m_altKeyInput->setSourceDevice(m_keyboardDevice);
  m_altKeyAction->addInput(m_altKeyInput);

  // Projection control
  m_changeProjectionTypeInput->setButtons(QVector<int>() << Qt::Key_5);
  m_changeProjectionTypeInput->setSourceDevice(m_keyboardDevice);
  m_changeProjectionAction->addInput(m_changeProjectionTypeInput);

  //// Axes

  // Mouse X
  m_panXInput->setAxis(Qt3DInput::QMouseDevice::X);
  m_panXInput->setSourceDevice(m_mouseDevice);
  m_panXAxis->addInput(m_panXInput);

  // Mouse Y
  m_panYInput->setAxis(Qt3DInput::QMouseDevice::Y);
  m_panYInput->setSourceDevice(m_mouseDevice);
  m_panYAxis->addInput(m_panYInput);

  // Mouse wheel X
  m_wheelXInput->setAxis(Qt3DInput::QMouseDevice::WheelY);
  m_wheelXInput->setSourceDevice(m_mouseDevice);
  m_wheelAxis->addInput(m_wheelXInput);

  m_wheelYInput->setAxis(Qt3DInput::QMouseDevice::WheelX);
  m_wheelYInput->setSourceDevice(m_mouseDevice);
  m_wheelAxis->addInput(m_wheelYInput);

  //// Logical device
  m_logicalDevice->addAction(m_midMouseButtonAction);
  m_logicalDevice->addAction(m_leftMouseButtonAction);
  m_logicalDevice->addAction(m_altKeyAction);
  m_logicalDevice->addAction(m_changeProjectionAction);
  m_logicalDevice->addAxis(m_panXAxis);
  m_logicalDevice->addAxis(m_panYAxis);
  m_logicalDevice->addAxis(m_wheelAxis);

  //// Mouse handler
  Qt3DInput::QMouseHandler *mouseHandler = new Qt3DInput::QMouseHandler();
  mouseHandler->setSourceDevice(m_mouseDevice);

  QObject::connect(mouseHandler, &Qt3DInput::QMouseHandler::pressed,
                   [this](Qt3DInput::QMouseEvent *pressedEvent) {
                     pressedEvent->setAccepted(true);
                     this->m_mouseLastPosition =
                         QPoint(pressedEvent->x(), pressedEvent->y());
                   });

  QObject::connect(mouseHandler, &Qt3DInput::QMouseHandler::positionChanged,
                   [this](Qt3DInput::QMouseEvent *positionChangedEvent) {
                     positionChangedEvent->setAccepted(true);
                     this->m_mouseCurrentPosition = QPoint(
                         positionChangedEvent->x(), positionChangedEvent->y());
                   });

  QObject::connect(m_changeProjectionAction, &Qt3DInput::QAction::activeChanged,
                   [this](bool isActive) {
                     if (isActive) {
                       Qt3DRender::QCamera *camera = this->m_camera;
                       Qt3DRender::QCameraLens::ProjectionType projectionType =
                           camera->projectionType();
                       switch (projectionType) {
                       case Qt3DRender::QCameraLens::PerspectiveProjection:
                         camera->setProjectionType(
                             Qt3DRender::QCameraLens::OrthographicProjection);
                         break;
                       case Qt3DRender::QCameraLens::OrthographicProjection:
                         camera->setProjectionType(
                             Qt3DRender::QCameraLens::PerspectiveProjection);
                         break;
                       default:
                         break;
                       }
                     }
                   });

  //// FrameAction
  QObject::connect(m_frameAction, &Qt3DLogic::QFrameAction::triggered,
                   [this](float dt) { this->onTriggered(dt); });

  // Disable the logical device when the entity is disabled
  QObject::connect(this, &Qt3DCore::QEntity::enabledChanged, m_logicalDevice,
                   &Qt3DInput::QLogicalDevice::setEnabled);

  addComponent(m_frameAction);
  addComponent(m_logicalDevice);
  addComponent(mouseHandler);
}

QVector3D QTrackballCameraController::projectScreenToTrackball(
    const QPoint &screenCoords, const QSize &windowSize,
    float trackballRadius) const {
  QVector2D p2D(
      2.0 * m_camera->aspectRatio() *
          ((float)screenCoords.x() / (float)windowSize.width() - 0.5),
      -2.0 * ((float)screenCoords.y() / (float)windowSize.height() - 0.5));

  QVector3D p3D(p2D, 0);

  float r = trackballRadius;
  float z0 = r * 0.5f;

  float z = 0;
  if (r * r - p3D.lengthSquared() >= z0 * z0) {
    z = sqrt(r * r - p3D.lengthSquared());
  } else {
    // Consistent (hyperbolic):
    z = z0 * std::sqrt(r * r - z0 * z0) / p3D.length();
  }

  return QVector3D(p3D[0], p3D[1], z);
}

QQuaternion QTrackballCameraController::createRotation(
    const QPoint &firstPoint, const QPoint &nextPoint, const QSize &windowSize,
    float trackBallRadius) const {
  QVector3D lastPos3D =
      projectScreenToTrackball(firstPoint, windowSize, trackBallRadius);
  QVector3D currentPos3D =
      projectScreenToTrackball(nextPoint, windowSize, trackBallRadius);

  QVector3D posAvg = (lastPos3D + currentPos3D) / 2;

  // Compute axis of rotation:
  QVector3D dir = QVector3D::crossProduct(currentPos3D, lastPos3D);

  // Approximate rotation angle:
  float t = dir.length() / posAvg.length();

  QQuaternion q = QQuaternion::fromAxisAndAngle(dir, t);
  return q;
}

Qt3DRender::QCamera *QTrackballCameraController::camera() const {
  return m_camera;
}

void QTrackballCameraController::setCamera(Qt3DRender::QCamera *camera) {
  if (this->m_camera != camera) {
    this->m_camera = camera;
    emit cameraChanged();
  }
}

float QTrackballCameraController::panSpeed() const { return this->m_panSpeed; }

float QTrackballCameraController::zoomSpeed() const {
  return this->m_zoomSpeed;
}

float QTrackballCameraController::rotationSpeed() const {
  return this->m_rotationSpeed;
}

float QTrackballCameraController::zoomCameraLimit() const {
  return this->m_zoomCameraLimit;
}

float QTrackballCameraController::trackballRadius() const {
  return this->m_trackballRadius;
}

QSize QTrackballCameraController::windowSize() const {
  return this->m_windowSize;
}

void QTrackballCameraController::setPanSpeed(float v) {
  if (this->m_panSpeed != v) {
    this->m_panSpeed = v;
    emit panSpeedChanged();
  }
}

void QTrackballCameraController::setZoomSpeed(float v) {
  if (this->m_zoomSpeed != v) {
    this->m_zoomSpeed = v;
    emit zoomSpeedChanged();
  }
}

void QTrackballCameraController::setRotationSpeed(float v) {
  if (this->m_rotationSpeed != v) {
    this->m_rotationSpeed = v;
    emit rotationSpeedChanged();
  }
}

void QTrackballCameraController::setZoomCameraLimit(float v) {
  if (this->m_zoomCameraLimit != v) {
    this->m_zoomCameraLimit = v;
    emit zoomCameraLimitChanged();
  }
}

void QTrackballCameraController::setTrackballRadius(float v) {
  if (this->m_trackballRadius != v) {
    this->m_trackballRadius = v;
    emit trackballRadiusChanged(v);
  }
}

void QTrackballCameraController::setWindowSize(const QSize &v) {
  if (this->m_windowSize != v) {
    this->m_windowSize = v;
    emit windowSizeChanged();
  }
}

void QTrackballCameraController::onTriggered(float) {
  // Perform the control based on the input received during the frame
  if (m_camera != nullptr) {
    if (m_midMouseButtonAction->isActive()) {
      if (m_altKeyAction->isActive()) {
        // Panning mode
        m_camera->translate(m_panSpeed * QVector3D(-m_panXAxis->value(),
                                                   -m_panYAxis->value(), 0),
                            Qt3DRender::QCamera::TranslateViewCenter);
      } else {

        QQuaternion rotation =
            createRotation(m_mouseLastPosition, m_mouseCurrentPosition,
                           m_windowSize, m_trackballRadius);

        QQuaternion currentRotation = m_camera->transform()->rotation();
        QQuaternion currentRotationInversed = currentRotation.conjugated();

        QVector3D rotatedAxis =
            currentRotationInversed.rotatedVector(rotation.vector());
        float angle = m_rotationSpeed * rotation.scalar();

        m_camera->rotateAboutViewCenter(
            QQuaternion::fromAxisAndAngle(rotatedAxis, angle));
        m_mouseLastPosition = m_mouseCurrentPosition;
      }
    } else {
      float d = (m_camera->position() - m_camera->viewCenter()).length();
      if (d - m_zoomSpeed * m_wheelAxis->value() < m_zoomCameraLimit)
        return;
      m_camera->translate(m_zoomSpeed * QVector3D(0, 0, m_wheelAxis->value()),
                          Qt3DRender::QCamera::DontTranslateViewCenter);

      // Compute the size of the frustum at the view center
      float size = std::tan(m_camera->fieldOfView()) * d / 4.0f;
      m_camera->setTop(size);
      m_camera->setBottom(-size);
      m_camera->setLeft(-size * m_camera->aspectRatio());
      m_camera->setRight(size * m_camera->aspectRatio());
    }
  }
}

} // namespace Qt3DExtras

QT_END_NAMESPACE
