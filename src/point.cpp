#include "point.hpp"

#include <cmath>

namespace beegen {
Point Point::operator*(double rhs) const {
    return {x * rhs, y * rhs};
}

Point Point::operator/(double rhs) const {
    return {x / rhs, y / rhs};
}

Point Point::operator+(const Point& rhs) const{
    return {x + rhs.x, y + rhs.y};
}

Point Point::operator-(const Point& rhs) const {
    return {x - rhs.x, y - rhs.y};
}

double Point::Length() const {
    return std::hypot(x, y);
}
}