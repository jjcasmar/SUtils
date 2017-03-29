#ifndef TRACKBALL_HELPER_H
#define TRACKBALL_HELPER_H

#include <cmath>
#include <QPoint>
#include <QSize>
#include <QVector3D>
#include <QQuaternion>

class TrackBallHelper
{
public:
    static QQuaternion rotation(const QPoint &lastPos, const QPoint &currrentPos, const QSize &size, const QPoint &center, float radius) {

        QVector3D lastPos3D(2.0f * ((float)lastPos.x() - (float)center.x()) / (float)size.width(),
            -2.0f * ((float)lastPos.y() - (float)center.y()) / (float)size.height(), 0.0f);

        QVector3D currentPos3D(2.0f * ((float)currrentPos.x() - (float)center.x()) / (float)size.width(),
            -2.0f * ((float)currrentPos.y() - (float)center.y()) / (float)size.height(), 0.0f);

        return rotation(lastPos3D, currentPos3D, radius);
	}

    static QQuaternion rotation(QVector3D lastPos, QVector3D currentPos, float radius) {
        lastPos[2] = 0;
        currentPos[2] = 0;

        if(lastPos == currentPos)
            return QQuaternion::fromAxisAndAngle({1,0,0}, 0);

        lastPos = projectToTrackball(lastPos, radius);
        currentPos = projectToTrackball(currentPos, radius);

        QVector3D posAvg = (lastPos + currentPos) / 2;

		// Compute axis of rotation:
        QVector3D dir = QVector3D::crossProduct(lastPos,currentPos);

		// Approximate rotation angle:
        float t = dir.length() / posAvg.length();

		if(t > 1.0f)
			t = 1.0f;

		if(t < -1.0f)
			t = -1.0f;

        //DRQuaternionT<Scalar> q(dir, t);
        QQuaternion q = QQuaternion::fromAxisAndAngle(dir, -180/3.1415*t);

        return q;
	}

protected:
    static QVector3D projectToTrackball(const QVector3D &v, float radius) {

        float r = radius;

        float z0 = radius * 0.5f;

		float z = 0;
        if(r*r - v.lengthSquared() >= z0*z0)
		{
            z = sqrt(r*r - v.lengthSquared());
		}
		else
		{
			// Original (hyperbolic):
            // z = r*r / (2 * v.length());

			// Consistent (hyperbolic):
            z = z0*sqrt(r*r - z0*z0) / v.length();
		}

        return QVector3D(v[0], v[1], z);
	}
};

#endif
