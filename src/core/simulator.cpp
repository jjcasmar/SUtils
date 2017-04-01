#include "simulator.h"

Simulator::Simulator() :
    t(new QTimer(this))
{
    t->setInterval(0);
}

void Simulator::step()
{
    step(m_dt);
}

void Simulator::run()
{
    t->start();
    connect(t, &QTimer::timeout,
            [this]() {
        this->step(m_dt);
    });
}

void Simulator::pause()
{
    t->stop();
}

float Simulator::dt() const
{
    return m_dt;
}

void Simulator::setDt(float dt)
{
    if (dt != m_dt) {
        m_dt = dt;
        emit dtChanged(dt);
    }
}
