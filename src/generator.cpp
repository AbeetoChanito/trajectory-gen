#include "generator.hpp"

#include "differentialKinematics.hpp"

#include <cmath>
#include <algorithm>

namespace beegen {
Generator::Generator(std::unique_ptr<Path> path, const Constraints& constraints, double deltaDistance)
    : m_Path(std::move(path)), m_Constraints(constraints), m_DifferentialKinematics(constraints), m_DeltaDistance(deltaDistance) {

}

static int PointsFitIn(float d, float deltaD) {
    float unrounded = d / deltaD;
    int rounded = static_cast<int>(unrounded);

    if (unrounded < 2)
        return rounded + 2;
    else 
        return rounded + 1;
}

std::vector<Generator::ProfilePoint> Generator::Calculate() {
    struct IntermediateProfilePoint {
        double vel;
        double angularVel;
        double distance;
        double t;
        double curvature;
    };

    float pathLength = m_Path->GetLength();

    int numPoints = PointsFitIn(pathLength, m_DeltaDistance);

    std::vector<IntermediateProfilePoint> forwardPass;
    forwardPass.reserve(numPoints);

    double vel = 0;
    double lastAngularVel = 0;

    forwardPass.push_back({0, 0, 0, 0, 0});

    for (int i = 1; numPoints > 2 && i < numPoints; i++) {
        double d = m_DeltaDistance * i;
        double t = m_Path->GetTFromArcLength(d);

        double curvature = m_Path->GetCurvature(t);
        double angularVel = vel * curvature;
        double angularAccel = (angularVel - lastAngularVel) * (vel / m_DeltaDistance);
        lastAngularVel = angularVel;

        double maxAccel = m_Constraints.MaxAccel - std::abs(angularAccel * m_Constraints.TrackWidth / 2);
        vel = std::min(m_DifferentialKinematics.GetMaxSpeed(curvature), std::sqrt(vel * vel + 2 * maxAccel * m_DeltaDistance));

        forwardPass.push_back(IntermediateProfilePoint {vel, angularVel, d, t, curvature});
    }

    forwardPass.push_back({0, 0, m_Path->GetLength(), m_Path->GetMaxT(), 0});

    std::vector<ProfilePoint> backwardPass;
    backwardPass.reserve(numPoints);

    vel = 0;
    lastAngularVel = 0;

    for (int i = forwardPass.size() - 1; i >= 0; i--) {
        IntermediateProfilePoint correspondingProfilePoint = forwardPass[i];

        double angularVel = vel * correspondingProfilePoint.curvature;
        double angularAccel = (angularVel - lastAngularVel) * (vel / m_DeltaDistance);
        lastAngularVel = angularVel;

        double maxAccel = m_Constraints.MaxDecel - std::abs(angularAccel * m_Constraints.TrackWidth / 2);
        vel = std::min(m_DifferentialKinematics.GetMaxSpeed(correspondingProfilePoint.curvature), std::sqrt(vel * vel + 2 * maxAccel * m_DeltaDistance));

        backwardPass.push_back({
            m_Path->GetPoint(correspondingProfilePoint.t),
            std::min(vel, correspondingProfilePoint.vel),
            correspondingProfilePoint.angularVel,
            correspondingProfilePoint.distance
        });
    }

    std::reverse(backwardPass.begin(), backwardPass.end());

    return backwardPass;
}
}