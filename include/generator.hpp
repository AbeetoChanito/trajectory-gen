#pragma once

#include <vector>

#include "point.hpp"
#include "path.hpp"
#include "trapezoidal.hpp"

namespace beegen {
class Generator {
    public:
        struct ProfilePoint {
            Point point;
            double vel;
            double angularVel;
            double dist;
        };

        Generator(std::unique_ptr<Path> path, const Constraints& constraints, double deltaDistance = 0.01);

        std::vector<ProfilePoint> Calculate();
    private:
        struct IntermediateProfilePoint {
            double t;
            double vel;
            double angularVel;
        };

        std::unique_ptr<Path> m_Path;

        Constraints m_Constraints;

        DifferentialKinematics m_DifferentialKinematics;

        double m_DeltaDistance;

        std::vector<IntermediateProfilePoint> CalculateForwardPass();

        std::vector<IntermediateProfilePoint> CalculateBackwardPass();
};
}