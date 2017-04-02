#ifndef BOUNDINGVOLUMESASPECT_H
#define BOUNDINGVOLUMESASPECT_H

#include <Qt3DCore>

class BoundingVolumesAspect : public Qt3DCore::QAbstractAspect
{
    Q_OBJECT
public:
    BoundingVolumesAspect();

    // QAbstractAspect interface
private:
    QVector<Qt3DCore::QAspectJobPtr> jobsToExecute(qint64 time);
    void onRegistered();
    void onUnregistered();
    void onEngineStartup();
    void onEngineShutdown();
};

#endif // BOUNDINGVOLUMESASPECT_H
