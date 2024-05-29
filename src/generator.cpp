#include "generator.hpp"

#include "differentialKinematics.hpp"

#include <cmath>
#include <algorithm>

namespace beegen {
Generator::Generator(std::unique_ptr<Path> path, const Constraints& constraints, double deltaDistance)
    : m_Path(std::move(path)), m_Constraints(constraints), m_DifferentialKinematics(constraints), m_DeltaDistance(deltaDistance) {

}

static int pointsFitIn(float d, float deltaD) {
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

    int toAllocate = pointsFitIn(pathLength, m_DeltaDistance) - 2;

    std::vector<IntermediateProfilePoint> forwardPass;
    forwardPass.reserve(toAllocate);

    double d = 0;
    double vel = 0;
    double lastAngularVel = 0;

    for (int i = 0; i < toAllocate; i++) {
        double d = m_DeltaDistance * (i + 1);
        double t = m_Path->GetTFromArcLength(d);

        double curvature = m_Path->GetCurvature(t);
        double angularVel = vel * curvature;
        double angularAccel = (angularVel - lastAngularVel) * (vel / m_DeltaDistance);
        lastAngularVel = angularVel;

        double maxAccel = m_Constraints.MaxAccel - std::abs(angularAccel * m_Constraints.TrackWidth / 2);
        vel = std::min(m_DifferentialKinematics.GetMaxSpeed(curvature), std::sqrt(vel * vel + 2 * maxAccel * m_DeltaDistance));

        forwardPass.push_back(IntermediateProfilePoint {vel, angularVel, d, t, curvature});
    }

    std::vector<ProfilePoint> backwardPass;
    backwardPass.reserve(toAllocate + 2);

    backwardPass.push_back({m_Path->GetPoint(m_Path->GetMaxT()), 0, 0, pathLength});

    vel = 0;
    lastAngularVel = 0;

    for (int i = toAllocate - 1; i >= 0; i--) {
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

    backwardPass.push_back({m_Path->GetPoint(0), 0, 0, 0});

    std::reverse(backwardPass.begin(), backwardPass.end());

    return backwardPass;
}
}