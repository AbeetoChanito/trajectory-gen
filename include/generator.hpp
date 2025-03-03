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

        Generator(std::shared_ptr<Path> path, const Constraints& constraints, double deltaDistance = 0.01, bool binarySearch = false);

        void Calculate();

        std::vector<ProfilePoint> GetProfile();

        ProfilePoint GetAtDistance(double distance);

        ProfilePoint GetAtTime(double time);

        double GetMaxLength();

        double GetMaxTime();
    private:
        std::shared_ptr<Path> m_Path;

        Constraints m_Constraints;

        DifferentialKinematics m_DifferentialKinematics;

        double m_DeltaDistance;

        std::vector<ProfilePoint> m_ProfilePoints;

        bool m_BinarySearch;
};
}