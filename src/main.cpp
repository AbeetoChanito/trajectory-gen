#include "path.hpp"

#include "generator.hpp"

#include <iostream>
#include <cmath>
#include <memory>

using namespace beegen;

int main(int argc, char* argv[]) {
    std::shared_ptr<Path> path = std::make_shared<CubicSpline>(std::initializer_list<CubicSpline::Knot> {
        CubicSpline::Knot {Point {0, 0}, 0, 50},
        CubicSpline::Knot {Point {50, 50}, M_PI / 2, 50},
        CubicSpline::Knot {Point {100, 100}, 0, 50}
    });
    
    path->CalculateData();

    Constraints constraints {
        .MaxVel = 60,
        .MaxAccel = 200,
        .MaxDecel = 200,
        .FrictionCoeff = 400,
        .TrackWidth = 10
    };

    Generator generator(path, constraints, 0.01);

    generator.Calculate();

    for (float x = 0; x < generator.GetMaxTime(); x += 0.01) {
        Generator::ProfilePoint point = generator.GetAtTime(x);

        std::cout << point.point.x << " " << point.point.y << "\n"
                << point.time << " " << point.vel << "\n"
                << point.time << " " << point.angularVel << "\n";
    }
}