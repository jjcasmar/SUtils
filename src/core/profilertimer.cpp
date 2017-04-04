#include "profilertimer.h"

#include <iostream>

ProfilerTimer::ProfilerTimer()
{
    m_identityString = QString("Generic identity");
    start();
}

ProfilerTimer::ProfilerTimer(const QString &identityString)
{
    m_identityString = identityString;
    start();
}

ProfilerTimer::~ProfilerTimer()
{
    profile();
}

void ProfilerTimer::profile()
{
    std::cout << m_identityString.toStdString() << ": " << elapsed() << std::endl;
}
