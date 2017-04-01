#include "simulationwindow.h"

#include <QtWidgets/QAction>
#include <QToolBar>
#include <simulator.h>

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
