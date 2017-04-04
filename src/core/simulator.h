#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <QObject>
#include <QTimer>

class Simulator : public QObject
{
    Q_OBJECT
    Q_PROPERTY(float dt READ dt WRITE setDt NOTIFY dtChanged)
public:
    Simulator();

    virtual void init() = 0;
    virtual void reset() = 0;
    virtual void step(float dt) = 0;
    void step();
    void run();
    void pause();

    float dt() const;
    void setDt(float dt);

    bool isRunning();

Q_SIGNALS:
    void dtChanged(float dt);
    void stepDone();

protected:
    float m_dt;
    QTimer *t;
    bool m_isRunning;
};

#endif // SIMULATOR_H
