#pragma once

namespace beegen {
struct Point {
    double x;
    double y;

    Point operator*(double rhs) const;

    Point operator/(double rhs) const;

    Point operator+(const Point& rhs) const;

    Point operator-(const Point& rhs) const;

    double Length() const;
};
}