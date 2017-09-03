#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

namespace Qt3DCore {
class QEntity;
}

namespace Qt3DRender {
class QCamera;
}

namespace Qt3DExtras {
class QTrackballCameraController;
class QPlaneGeometry;
}

class OpenGLWindow;

class MainWindow : public QMainWindow
{
    Q_OBJECT
    Q_PROPERTY(Qt3DCore::QEntity *sceneRootEntity READ sceneRootEntity)
    Q_PROPERTY(Qt3DRender::QCamera *sceneCamera READ sceneCamera)
public:
    explicit MainWindow(QWidget *parent = 0);

    Qt3DCore::QEntity *sceneRootEntity() const;
    Qt3DRender::QCamera *sceneCamera() const;
    Qt3DExtras::QTrackballCameraController *cameraController() const;
    Qt3DExtras::QPlaneGeometry *floorGeometry() const;

public slots:
    void takeScreenshot();
    virtual void startVideo();
    virtual void stopVideo();

signals:
    void OpenGLViewTickFrame(float dt);

protected:
    OpenGLWindow *m_OpenGLView;


    QAction *m_takeImageAction;
    QAction *m_takeVideoAction;
    QAction *m_stopVideoAction;

    uint m_videoFrame;
    QMetaObject::Connection m_takeVideoSlot;
};


#endif // MAINWINDOW_H
