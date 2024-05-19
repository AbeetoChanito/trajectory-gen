#include "differentialKinematics.hpp"

#include <cmath>
#include <algorithm>

namespace beegen {
DifferentialKinematics::DifferentialKinematics(const Constraints& constraints)
    : m_Constraints(constraints) {}

double DifferentialKinematics::GetMaxSpeed(double curvature) {
    double maxTurnSpeed = ((2 * m_Constraints.MaxVel / m_Constraints.TrackWidth) * m_Constraints.MaxVel) / (std::abs(curvature) * m_Constraints.MaxVel + (2 * m_Constraints.MaxVel / m_Constraints.TrackWidth));

    if (curvature == 0) return maxTurnSpeed;

    double maxSlipSpeed = std::sqrt(m_Constraints.FrictionCoeff * (1 / std::abs(curvature)) * 9.81);

    return std::min(maxTurnSpeed, maxSlipSpeed);
}

std::pair<double, double> DifferentialKinematics::GetWheelVelocities(double linearVel, double angularVel) {
    double velLeft = linearVel - angularVel * m_Constraints.TrackWidth / 2;
    double velRight = linearVel + angularVel * m_Constraints.TrackWidth / 2;
    return std::make_pair(velLeft, velRight);
} 
}