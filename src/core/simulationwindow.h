#ifndef SIMULATIONWINDOW_H
#define SIMULATIONWINDOW_H

#include "mainwindow.h"
#include <QThread>

class QAction;
class Simulator;

class SimulationWindow : public MainWindow
{
    Q_OBJECT
    Q_PROPERTY(Simulator *simulator READ simulator WRITE setSimulator NOTIFY simulatorChanged)
public:
    SimulationWindow(Simulator *simulator, QWidget *parent = 0);

    Simulator *simulator() const;
    void setSimulator(Simulator *simulator);

Q_SIGNALS:
    void simulatorChanged(const Simulator *simulator);

protected:
    QAction *m_stepSimulationAction;
    QAction *m_runSimulationAction;
    QAction *m_pauseSimulationAction;
    QAction *m_resetSimulationAction;

    Simulator *m_simulator;
    QThread *m_simulatorThread;

    // MainWindow interface
public slots:
    void startVideo();
    void stopVideo();
};

#endif // SIMULATIONWINDOW_H
