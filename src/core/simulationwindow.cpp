#include "simulationwindow.h"

#include <QtWidgets/QAction>
#include <QToolBar>
#include <simulator.h>
#include <openglwindow.h>

#include <QFileDialog>

//TODO When taking a video, only capture frame when a new step have been done, overriding the
//continue mode set in MainWindow

SimulationWindow::SimulationWindow(Simulator *simulator, QWidget *parent) :
    MainWindow(parent)
  ,m_simulator(simulator)
  ,m_stepSimulationAction(new QAction("Step", this))
  ,m_runSimulationAction(new QAction("Run", this))
  ,m_pauseSimulationAction(new QAction("Pause", this))
  ,m_resetSimulationAction(new QAction("Reset", this))
  ,m_simulatorThread(new QThread(this))
{
    connect(m_stepSimulationAction,
            &QAction::triggered,
            m_simulator,
            static_cast<void(Simulator::*)()>(&Simulator::step));

    connect(m_runSimulationAction, &QAction::triggered,
            m_simulator, &Simulator::run);

    connect(m_pauseSimulationAction, &QAction::triggered,
            m_simulator, &Simulator::pause);

    connect(m_resetSimulationAction, &QAction::triggered,
            m_simulator, &Simulator::reset);

    QToolBar *simulationToolbar = addToolBar("Simulation");
    simulationToolbar->addActions({m_stepSimulationAction,
                                   m_runSimulationAction,
                                   m_pauseSimulationAction,
                                   m_resetSimulationAction,
                                  });

    //Change simulators thread affinity
    m_simulator->moveToThread(m_simulatorThread);
    m_simulatorThread->start();

    //Initialize the simulator
    m_simulator->init();
}

Simulator *SimulationWindow::simulator() const
{
    return m_simulator;
}

void SimulationWindow::setSimulator(Simulator *simulator)
{
    if (m_simulator != simulator) {
        m_simulator = simulator;
        m_simulator->init();
        emit simulatorChanged(simulator);
    }
}

void SimulationWindow::startVideo()
{
    m_stopVideoAction->setEnabled(true);
    m_takeVideoAction->setEnabled(false);
    m_videoFrame = 0;
    QString dir = QFileDialog::getExistingDirectory(this, "Take video", "~");

    //In each frame, save an image
    m_takeVideoSlot = connect(m_simulator, &Simulator::stepDone,
                              [this, dir]() {
        QString framename;
        QTextStream stream(&framename);
        stream << dir << "/frame_" << this->m_videoFrame++ << ".png";
        m_OpenGLView->takeImage(framename);
    });

    if (!m_simulator->isRunning())
        m_runSimulationAction->trigger();
}

void SimulationWindow::stopVideo()
{
    m_pauseSimulationAction->trigger();
    disconnect(m_takeVideoSlot);
    m_stopVideoAction->setEnabled(false);
    m_takeVideoAction->setEnabled(true);
}
