#pragma once

#include <initializer_list>
#include <vector>

#include "point.hpp"

namespace beegen {
class Path {
    public:
        virtual void CalculateData();

        virtual double GetLength() const = 0;

        virtual Point GetPoint(double t) const = 0;
        virtual Point GetDerivative(double t) const = 0;
        virtual Point GetSecondDerivative(double t) const = 0;
        
        virtual double GetCurvature(double t) const = 0;

        virtual double GetTFromArcLength(double arcLength) const = 0;

        virtual double GetMaxT() const = 0;

        virtual ~Path() = default;
};

class CubicBezier : public Path {
    public:
        CubicBezier(const Point& p0, const Point& p1, const Point& p2, const Point& p3, double tIncrement = 0.01);

        void CalculateData() override;

        double GetLength() const override;

        Point GetPoint(double t) const override;
        Point GetDerivative(double t) const override;
        Point GetSecondDerivative(double t) const override;

        double GetCurvature(double t) const override;

        double GetTFromArcLength(double arcLength) const override;

        double GetMaxT() const override;
    private:
        Point m_P0;
        Point m_P1;
        Point m_P2;
        Point m_P3;

        std::vector<double> m_LengthsAtT;
        double m_TIncrement;
};

class CubicSpline : public Path {
    public:
        struct Knot {
            Point point;
            double angle;
            double mag;
        };

        CubicSpline(const std::initializer_list<Knot>& knots);

        void CalculateData() override;

        int GetBezierAmount() const;

        double GetLength() const override;

        Point GetPoint(double t) const override;
        Point GetDerivative(double t) const override;
        Point GetSecondDerivative(double t) const override;

        double GetCurvature(double t) const override;

        double GetTFromArcLength(double arcLength) const override;

        double GetMaxT() const override;
    private:
        std::vector<CubicBezier> m_Beziers;
};
}