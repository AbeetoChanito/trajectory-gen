#include "generator.hpp"

#include "differentialKinematics.hpp"

#include <cmath>

namespace beegen {
Generator::Generator(std::unique_ptr<Path> path, const Constraints& constraints, double deltaDistance)
    : m_Path(std::move(path)), m_Constraints(constraints), m_DifferentialKinematics(constraints), m_DeltaDistance(deltaDistance) {

}

std::vector<Generator::IntermediateProfilePoint> Generator::CalculateForwardPass() {
    std::vector<IntermediateProfilePoint> forwardPass;

    forwardPass.push_back(IntermediateProfilePoint {0, 0, 0});

    double d = 0;
    double vel = 0;
    double lastAngularVel = 0;

    while (d <= m_Path->GetLength()) {
        d += m_DeltaDistance;

        double t = m_Path->GetTFromArcLength(d);

        double curvature = m_Path->GetCurvature(t);
        double angularVel = vel * curvature;
        double angularAccel = (angularVel - lastAngularVel) * (vel / m_DeltaDistance);
        lastAngularVel = angularVel;

        double maxAccel = m_Constraints.MaxAccel - std::abs(angularAccel * m_Constraints.TrackWidth / 2);
        vel = std::min(m_DifferentialKinematics.GetMaxSpeed(curvature), std::sqrt(vel * vel + 2 * maxAccel * m_DeltaDistance));

        forwardPass.push_back(IntermediateProfilePoint {t, vel, angularVel});
    }

    return forwardPass;
}

std::vector<Generator::IntermediateProfilePoint> Generator::CalculateBackwardPass() {
    std::vector<IntermediateProfilePoint> backwardPass;

    backwardPass.push_back(IntermediateProfilePoint {0, 0, 0});

    double d = m_Path->GetLength();
    double vel = 0;
    double lastAngularVel = 0;

    while (d >= 0) {
        d -= m_DeltaDistance;

        double t = m_Path->GetTFromArcLength(d);

        double curvature = m_Path->GetCurvature(t);
        double angularVel = vel * curvature;
        double angularAccel = (angularVel - lastAngularVel) * (vel / m_DeltaDistance);
        lastAngularVel = angularVel;

        double maxAccel = m_Constraints.MaxDecel - std::abs(angularAccel * m_Constraints.TrackWidth / 2);
        vel = std::min(m_DifferentialKinematics.GetMaxSpeed(curvature), std::sqrt(vel * vel + 2 * maxAccel * m_DeltaDistance));
        
        backwardPass.push_back(IntermediateProfilePoint {t, vel, angularVel});
    }

    return backwardPass;
}

std::vector<Generator::ProfilePoint> Generator::Calculate() {
    std::vector<Generator::ProfilePoint> profile;

    std::vector<IntermediateProfilePoint> forwardPass = CalculateForwardPass();
    std::vector<IntermediateProfilePoint> backwardPass = CalculateBackwardPass();

    for (int i = 0; i < backwardPass.size(); i++) {
        IntermediateProfilePoint forwardPoint = forwardPass[i];
        IntermediateProfilePoint backwardPoint = forwardPass[backwardPass.size() - (i + 1)];

        profile.push_back(ProfilePoint {
            m_Path->GetPoint(forwardPoint.t),
            std::min(forwardPoint.vel, backwardPoint.vel),
            forwardPoint.angularVel,
            i * m_DeltaDistance
        });
    }

    return profile;
}
}