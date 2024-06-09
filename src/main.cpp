#include "path.hpp"

#include "generator.hpp"

#include <iostream>
#include <cmath>
#include <memory>

using namespace beegen;

int main(int argc, char* argv[]) {
    std::unique_ptr<Path> cubicBezier = std::make_unique<CubicSpline>(std::initializer_list<CubicSpline::Knot> {
        CubicSpline::Knot {Point {0, 0}, 0, 50},
        CubicSpline::Knot {Point {50, 50}, M_PI / 2, 50},
        CubicSpline::Knot {Point {100, 100}, 0, 50}
    });
    
    cubicBezier->CalculateData();

    Constraints constraints {
        .MaxVel = 60,
        .MaxAccel = 200,
        .MaxDecel = 200,
        .FrictionCoeff = 400,
        .TrackWidth = 15
    };

    Generator generator(std::move(cubicBezier), constraints, 0.01);

    generator.Calculate();
    std::vector<Generator::ProfilePoint> output = generator.Access();

    for (const Generator::ProfilePoint& point : output) {
        std::cout << point.point.x << " " << point.point.y << "\n"
                << point.time << " " << point.vel << "\n"
                << point.time << " " << point.angularVel << "\n";
    }
}
