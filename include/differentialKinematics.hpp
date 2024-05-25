#pragma once

#include <utility>

namespace beegen {
struct Constraints {
    double MaxVel;
    double MaxAccel;
    double MaxDecel;
    double FrictionCoeff;
    double TrackWidth;
};

class DifferentialKinematics {
    public:
        DifferentialKinematics(const Constraints& constraints);

        double GetMaxSpeed(double curvature);

        std::pair<double, double> GetWheelVelocities(double linearVel, double angularVel);
    private:
        const Constraints& m_Constraints;
};
}