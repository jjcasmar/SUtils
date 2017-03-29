#ifndef PROFILERTIMER_H
#define PROFILERTIMER_H

#include <QTime>
#include <QString>

class ProfilerTimer : private QTime
{
public:
    ProfilerTimer();
    ProfilerTimer(const QString &identityString);
    ~ProfilerTimer();

private:
    ProfilerTimer(const ProfilerTimer &pt);
    ProfilerTimer &operator = (const ProfilerTimer &pt);

    QString m_identityString;
};

#endif // PROFILERTIMER_H
