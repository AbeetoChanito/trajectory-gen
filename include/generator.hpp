#pragma once

#include <vector>

#include "point.hpp"
#include "path.hpp"
#include "differentialKinematics.hpp"

namespace beegen {
class Generator {
    public:
        struct ProfilePoint {
            Point point;
            double vel;
            double angularVel;
            double dist;
            double time;
        };

        Generator(std::unique_ptr<Path> path, const Constraints& constraints, double deltaDistance = 0.01);

        void Calculate();

        std::vector<ProfilePoint> Access();
    private:
        std::unique_ptr<Path> m_Path;

        Constraints m_Constraints;

        DifferentialKinematics m_DifferentialKinematics;

        double m_DeltaDistance;

        std::vector<ProfilePoint> m_ProfilePoints;
};
}