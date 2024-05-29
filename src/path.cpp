#include "path.hpp"

#include <cmath>

namespace beegen {
void Path::CalculateData() {}

CubicBezier::CubicBezier(const Point& p0, const Point& p1, const Point& p2, const Point& p3, double tIncrement)
    : m_P0(p0), m_P1(p1), m_P2(p2), m_P3(p3), m_TIncrement(tIncrement) {
}

void CubicBezier::CalculateData() {
    if (m_LengthsAtT.size() != 0) return;

    double dist = 0;
    Point lastPoint = m_P0;

    for (double t = 0; t <= 1; t += m_TIncrement) {
        Point point = GetPoint(t);
        dist += (point - lastPoint).Length();
        lastPoint = point;
        m_LengthsAtT.push_back(dist);
    }
}

double CubicBezier::GetLength() const {
    return m_LengthsAtT[m_LengthsAtT.size() - 1];
}

Point CubicBezier::GetPoint(double t) const {
    double oneMinusT = 1 - t;

    return m_P0 * oneMinusT * oneMinusT * oneMinusT
            + m_P1 * 3 * oneMinusT * oneMinusT * t
            + m_P2 * 3 * oneMinusT * t * t
            + m_P3 * t * t * t;
}

Point CubicBezier::GetDerivative(double t) const {
    double oneMinusT = 1 - t;

    return (m_P1 - m_P0) * 3 * oneMinusT * oneMinusT
            + (m_P2 - m_P1) * 6 * t * oneMinusT
            + (m_P3 - m_P2) * 3 * t * t;
}

Point CubicBezier::GetSecondDerivative(double t) const {
    return (m_P2 - m_P1 * 2 + m_P0) * 6 * (1 - t)
            + (m_P3 - m_P2 * 2 + m_P1) * 6 * t;
}

double CubicBezier::GetCurvature(double t) const {
    Point d = GetDerivative(t);
    Point dd = GetSecondDerivative(t);
    double denominator = d.x * d.x + d.y * d.y;
    denominator = std::sqrt(denominator * denominator * denominator);
    return (d.x * dd.y - d.y * dd.x) / denominator;
}

double CubicBezier::GetTFromArcLength(double arcLength) const {
    if (arcLength >= GetLength()) return 1;

    if (arcLength <= 0) return 0;

    int low = 0;
    int high = GetLength();
    int mid = 0;

    int targetIndex = 0;

    while (low < high) {
        mid = static_cast<int>((low + high) / 2);
        double lengthAtMid = m_LengthsAtT[mid];
        
        if (lengthAtMid < arcLength) {
            low = mid + 1;
        } else if (lengthAtMid > arcLength) {
            high = mid;
        } else {
            return targetIndex * m_TIncrement;
        }
    }

    if (m_LengthsAtT[mid] > arcLength) {
        mid--;
    }

    double lengthAtMid = m_LengthsAtT[mid];

    if (lengthAtMid == arcLength) {
        return mid * m_TIncrement;
    } else {
        double interpolationAmount = (arcLength - m_LengthsAtT[mid]) / (m_LengthsAtT[mid + 1] - m_LengthsAtT[mid]);
        return (mid + interpolationAmount) * m_TIncrement;
    }
}

double CubicBezier::GetMaxT() const {
    return 1;
}

CubicSpline::CubicSpline(const std::initializer_list<Knot>& knots) {
    auto knotsArray = knots.begin();

    for (int i = 0; i < knots.size() - 1; i++) {
        Knot thisKnot = knotsArray[i];
        Knot nextKnot = knotsArray[i + 1];

        m_Beziers.push_back(CubicBezier(
            thisKnot.point,
            thisKnot.point + Point {std::cos(thisKnot.angle), std::sin(thisKnot.angle)} * thisKnot.mag,
            nextKnot.point + Point {static_cast<double>(std::cos(nextKnot.angle + M_PI)), static_cast<double>(std::sin(nextKnot.angle + M_PI))} * nextKnot.mag,
            nextKnot.point
        ));
    }
}

void CubicSpline::CalculateData() {
    for (CubicBezier& bezier : m_Beziers) {
        bezier.CalculateData();
    }
}

int CubicSpline::GetBezierAmount() const {
    return m_Beziers.size();
}

double CubicSpline::GetLength() const {
    double length = 0;

    for (const CubicBezier& bezier : m_Beziers) {
        length += bezier.GetLength();
    }

    return length;
}

#define IMPLEMENT_FOR_SPLINE(f, type) \
    type CubicSpline::Get##f(double t) const { \
        int floored = static_cast<int>(t); \
        if (floored == m_Beziers.size()) floored--; \
        return m_Beziers[floored].Get##f(t - floored); \
    } \

IMPLEMENT_FOR_SPLINE(Point, Point)
IMPLEMENT_FOR_SPLINE(Derivative, Point)
IMPLEMENT_FOR_SPLINE(SecondDerivative, Point)
IMPLEMENT_FOR_SPLINE(Curvature, double)

double CubicSpline::GetTFromArcLength(double arcLength) const {    
    if (arcLength >= GetLength()) return m_Beziers.size();

    if (arcLength <= 0) return 0;
    
    double totalLength = 0;
    int i = 0;

    while (arcLength > totalLength) {
        totalLength += m_Beziers[i++].GetLength();
    }

    totalLength -= m_Beziers[(i == 0) ? 0 : --i].GetLength();

    return i + m_Beziers[i].GetTFromArcLength(arcLength - totalLength);
}

double CubicSpline::GetMaxT() const {
    return m_Beziers.size();
}
}