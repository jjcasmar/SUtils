#include "mainwindow.h"

#include "openglwindow.h"

#include <QFileDialog>
#include <QToolBar>
#include <QtWidgets/QAction>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), m_OpenGLView(new OpenGLWindow),
      m_takeImageAction(new QAction("Take screenshot", this)),
      m_takeVideoAction(new QAction("Take video", this)),
      m_stopVideoAction(new QAction("Stop video", this)) {

  // Create the central view
  setCentralWidget(QWidget::createWindowContainer(m_OpenGLView, this));
  connect(m_OpenGLView, &OpenGLWindow::frameTick, this,
          &MainWindow::OpenGLViewTickFrame);

  // Actions
  m_stopVideoAction->setEnabled(false);
  connect(m_takeImageAction, &QAction::triggered, this,
          &MainWindow::takeScreenshot);
  connect(m_takeVideoAction, &QAction::triggered, this,
          &MainWindow::startVideo);
  connect(m_stopVideoAction, &QAction::triggered, this, &MainWindow::stopVideo);

  // Create the default toolbar
  QToolBar *renderToolBar = addToolBar("Render toolbar");
  renderToolBar->addActions(
      {m_takeImageAction, m_takeVideoAction, m_stopVideoAction});
}

Qt3DCore::QEntity *MainWindow::sceneRootEntity() const {
  return m_OpenGLView->rootEntity();
}

Qt3DRender::QCamera *MainWindow::sceneCamera() const {
  return m_OpenGLView->camera();
}

Qt3DExtras::QTrackballCameraController *MainWindow::cameraController() const {
  return m_OpenGLView->trackballCameraController();
}

Qt3DExtras::QPlaneGeometry *MainWindow::floorGeometry() const {
  return m_OpenGLView->floorGeometry();
}

// Qt3DExtras::QTrackballCameraController *MainWindow::cameraController() const
//{
//    return m_OpenGLView->trackballCameraController();
//}

void MainWindow::takeScreenshot() {
  QString filename = QFileDialog::getSaveFileName(this, "Take screenshot");
  m_OpenGLView->setImageFilename(filename);
  m_OpenGLView->takeImage();
}

void MainWindow::startVideo() {
  //  m_stopVideoAction->setEnabled(true);
  //  m_takeVideoAction->setEnabled(false);
  //  m_videoFrame = 0;
  //  QString dir = QFileDialog::getExistingDirectory(nullptr, "Take video",
  //  "~");

  //  // In each frame, save an image
  //  m_takeVideoSlot =
  //      connect(m_OpenGLView, &OpenGLWindow::frameTick, [this, dir]() {
  //        QString framename;
  //        QTextStream stream(&framename);
  //        stream << dir << "/frame_" << this->m_videoFrame++ << ".png";
  //        m_OpenGLView->takeImage(framename);
  //      });
}

void MainWindow::stopVideo() {
  m_stopVideoAction->setEnabled(false);
  m_takeVideoAction->setEnabled(true);
  disconnect(m_takeVideoSlot);
}
