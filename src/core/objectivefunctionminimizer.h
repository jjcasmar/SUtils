#ifndef OBJECTIVEFUNCTIONMINIMIZER_H
#define OBJECTIVEFUNCTIONMINIMIZER_H

#include <QObject>

class nlopt_opt;
class ObjectiveFunction;

class ObjectiveFunctionMinimizer : public QObject
{
    Q_OBJECT
    Q_PROPERTY(quint32 maxIterations READ maxIterations WRITE setMaxIterations NOTIFY maxIterationsChanged)

public:
    enum OptimizationTechnique {

    };

    ObjectiveFunctionMinimizer();

    quint32 maxIterations() const;

public Q_SLOTS:
    void setMaxIterations(quint32 maxIterations);

Q_SIGNALS:
    void maxIterationsChanged(quint32 maxIterations);

private:
    ObjectiveFunction *m_objectiveFunction;
    nlopt_opt *m_nloptOptimizer;

    quint32 m_maxIterations;
};

#endif // OBJECTIVEFUNCTIONMINIMIZER_H
