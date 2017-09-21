#include "objectivefunctionminimizer.h"

#include <nlopt.h>

ObjectiveFunctionMinimizer::ObjectiveFunctionMinimizer()
{

                       nlopt_opt a;
}

quint32 ObjectiveFunctionMinimizer::maxIterations() const
{
    return m_maxIterations;
}

void ObjectiveFunctionMinimizer::setMaxIterations(quint32 maxIterations)
{
    if (m_maxIterations != maxIterations) {
        m_maxIterations = maxIterations;
        emit maxIterationsChanged(m_maxIterations);
    }
}
