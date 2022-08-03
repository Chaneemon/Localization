
#include <cmath>
#include <iostream>

namespace sensor_fusion{
  class Point
{
  public:
    Point(double x = 0.0, double y = 0.0) : x(x), y(y) {}
    Point(const Point& p) : x(p.x), y(p.y) {}
    static Point fromPoolarCoords(const double r, const double phi) { return Point(r * cos(phi), r * sin(phi)); }

    double length()        const { return sqrt(pow(x, 2.0) + pow(y, 2.0)); }
    double lengthSquared() const { return pow(x, 2.0) + pow(y, 2.0); }
    double angle()         const { return atan2(y, x); }

    Point normalized() { return (length() > 0.0) ? *this / length() : *this; }
    Point perpendicular() const { return Point(-y, x); }

    friend Point operator+ (const Point& p1, const Point& p2) { return Point(p1.x + p2.x, p1.y + p2.y); }
    friend Point operator- (const Point& p1, const Point& p2) { return Point(p1.x - p2.x, p1.y - p2.y); }
    friend Point operator* (const double f, const Point& p)  { return Point(f * p.x, f * p.y); }
    friend Point operator* (const Point& p, const double f)  { return Point(f * p.x, f * p.y); }
    friend Point operator/ (const Point& p, const double f)  { return (f != 0.0) ? Point(p.x / f, p.y / f) : Point(); }

    Point operator- () { return Point(-x, -y); }
    Point operator+ () { return Point( x,  y); }

    Point& operator=  (const Point& p) { if (this != &p) { x = p.x; y = p.y; } return *this; }
    Point& operator+= (const Point& p) { x += p.x; y += p.y; return *this; }
    Point& operator-= (const Point& p) { x -= p.x; y -= p.y; return *this; }

    friend bool operator== (const Point& p1, const Point& p2) { return (p1.x == p2.x && p1.y == p2.y); }
    friend bool operator!= (const Point& p1, const Point& p2) { return !(p1 == p2); }
    friend bool operator<  (const Point& p1, const Point& p2) { return (p1.lengthSquared() < p2.lengthSquared()); }
    friend bool operator<= (const Point& p1, const Point& p2) { return (p1.lengthSquared() <= p2.lengthSquared()); }
    friend bool operator>  (const Point& p1, const Point& p2) { return (p1.lengthSquared() > p2.lengthSquared()); }
    friend bool operator>= (const Point& p1, const Point& p2) { return (p1.lengthSquared() >= p2.lengthSquared()); }
    friend bool operator!  (const Point& p1) { return (p1.x == 0.0 && p1.y == 0.0); }

    friend std::ostream& operator<<(std::ostream& out, const Point& p)
    { out << "(" << p.x << ", " << p.y << ")"; return out; }

    double x;
    double y;
  };
}
