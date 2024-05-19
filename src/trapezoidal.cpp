#include "trapezoidal.hpp"

#include <cmath>
#include <exception>

namespace beegen {
TrapezoidalMotionProfile::TrapezoidalMotionProfile(const Constraints& constraints)
    : m_Constraints(constraints) {
    double NonCruiseDist = constraints.MaxVel * constraints.MaxVel / (2 * constraints.MaxAccel) + constraints.MaxVel * constraints.MaxVel / (2 * constraints.MaxDecel);
    m_CruiseVel = NonCruiseDist < constraints.Length ? constraints.MaxVel : std::sqrt(2 * constraints.Length * constraints.MaxAccel * constraints.MaxDecel / (constraints.MaxAccel + constraints.MaxDecel));
    m_AccelDist = (m_CruiseVel * m_CruiseVel - constraints.StartVel * constraints.StartVel) / (2 * constraints.MaxAccel);
    m_DecelDist = constraints.Length + (constraints.EndVel * constraints.EndVel - m_CruiseVel * m_CruiseVel) / (2 * constraints.MaxDecel);
}

double TrapezoidalMotionProfile::GetVelAtDist(double dist) {
    if (dist <= 0) {
        return m_Constraints.StartVel;
    }

    if (dist >= m_Constraints.Length) {
        return m_Constraints.EndVel;
    }

    if (dist < m_AccelDist) {
        return std::sqrt(m_Constraints.StartVel * m_Constraints.StartVel + 2 * m_Constraints.MaxAccel * dist);
    } else if (dist < m_DecelDist) {
        return m_CruiseVel;
    } else {
        return std::sqrt(m_CruiseVel * m_CruiseVel + -2 * m_Constraints.MaxDecel * (dist - m_DecelDist));
    }
}
}