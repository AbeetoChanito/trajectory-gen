#include "generator.hpp"

#include "differentialKinematics.hpp"

#include <cmath>
#include <algorithm>
#include <iostream>

namespace beegen {
Generator::Generator(std::shared_ptr<Path> path, const Constraints& constraints, double deltaDistance)
    : m_Path(path), m_Constraints(constraints), m_DifferentialKinematics(constraints), m_DeltaDistance(deltaDistance) {

}

static int PointsFitIn(double d, double deltaD) {
    double unrounded = d / deltaD;
    int rounded = static_cast<int>(unrounded);

    if (unrounded < 2)
        return rounded + 2;
    else 
        return rounded + 1;
}

void Generator::Calculate() {
    struct IntermediateProfilePoint {
        double vel;
        double distance;
        double t;
        double curvature;
    };

    int numPoints = PointsFitIn(m_Path->GetLength(), m_DeltaDistance);

    std::vector<IntermediateProfilePoint> forwardPass;
    forwardPass.reserve(numPoints);

    double vel = 0;
    double lastAngularVel = 0;

    forwardPass.push_back({0, 0, 0, m_Path->GetCurvature(0)});

    for (int i = 1; numPoints > 2 && i < numPoints; i++) {
        double d = m_DeltaDistance * i;
        double t = m_Path->GetTFromArcLength(d);

        double curvature = m_Path->GetCurvature(t);
        double angularVel = vel * curvature;
        double angularAccel = (angularVel - lastAngularVel) * (vel / m_DeltaDistance);
        lastAngularVel = angularVel;

        double maxAccel = m_Constraints.MaxAccel - std::abs(angularAccel * m_Constraints.TrackWidth / 2);
        vel = std::min(m_DifferentialKinematics.GetMaxSpeed(curvature), std::sqrt(vel * vel + 2 * maxAccel * m_DeltaDistance));

        forwardPass.push_back(IntermediateProfilePoint {vel, d, t, curvature});
    }

    forwardPass.push_back({0, m_Path->GetLength(), m_Path->GetMaxT(), m_Path->GetCurvature(m_Path->GetMaxT())});

    m_ProfilePoints.clear();
    m_ProfilePoints.reserve(numPoints);

    vel = 0;
    lastAngularVel = 0;

    for (int i = numPoints - 1; i >= 0; i--) {
        IntermediateProfilePoint correspondingProfilePoint = forwardPass[i];

        double angularVel = vel * correspondingProfilePoint.curvature;
        double angularAccel = (angularVel - lastAngularVel) * (vel / m_DeltaDistance);
        lastAngularVel = angularVel;

        double maxAccel = m_Constraints.MaxDecel - std::abs(angularAccel * m_Constraints.TrackWidth / 2);
        vel = std::min(m_DifferentialKinematics.GetMaxSpeed(correspondingProfilePoint.curvature), std::sqrt(vel * vel + 2 * maxAccel * m_DeltaDistance));

        double minVel = std::min(vel, correspondingProfilePoint.vel);

        m_ProfilePoints.push_back({
            m_Path->GetPoint(correspondingProfilePoint.t),
            minVel,
            minVel * correspondingProfilePoint.curvature,
            correspondingProfilePoint.distance,
            NAN
        });
    }

    std::reverse(m_ProfilePoints.begin(), m_ProfilePoints.end());

    double time = 0;

    for (ProfilePoint& point : m_ProfilePoints) {
        time += (point.vel == 0) ? 0 : m_DeltaDistance / point.vel;
        point.time = time;
    }
}

std::vector<Generator::ProfilePoint> Generator::GetProfile() {
    return m_ProfilePoints;
}

Generator::ProfilePoint Generator::GetAtDistance(double distance) {
    return m_ProfilePoints[static_cast<int>(distance / m_DeltaDistance)];
}

Generator::ProfilePoint Generator::GetAtTime(double time) {
    if (time == 0) {
        return m_ProfilePoints[0];
    }

    if (time == GetMaxTime()) {
        return m_ProfilePoints[m_ProfilePoints.size() - 1];
    }

    int low = 0;
    int high = m_ProfilePoints.size() - 1;
    int mid = 0;

    while (low < high) {
        mid = static_cast<int>((low + high) / 2.0);
        double timeAtMid = m_ProfilePoints[mid].time;
        
        if (timeAtMid < time) {
            low = mid + 1;
        } else if (timeAtMid > time) {
            high = mid;
        } else {
            return m_ProfilePoints[mid + 1];
        }
    }

    if (m_ProfilePoints[mid].time > time) {
        mid--;
    }

    return m_ProfilePoints[mid + 1];
}

double Generator::GetMaxLength() {
    return m_Path->GetLength();
}

double Generator::GetMaxTime() {
    return m_ProfilePoints[m_ProfilePoints.size() - 1].time;
}
}