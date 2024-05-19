#pragma once

#include "differentialKinematics.hpp"

namespace beegen {
class TrapezoidalMotionProfile {
    public:
        TrapezoidalMotionProfile(const Constraints& constraints);

        double GetVelAtDist(double dist);
    private:
        Constraints m_Constraints;

        double m_CruiseVel;
        double m_AccelDist;
        double m_DecelDist;
};
}
