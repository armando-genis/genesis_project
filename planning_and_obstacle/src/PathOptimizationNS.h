#ifndef PATHOPTIMIZATIONNS_H
#define PATHOPTIMIZATIONNS_H

#include <cmath>

namespace PathOptimizationNS {

struct State {
    State() = default;
    State(double x, double y, double z = 0, double k = 0, double s = 0, double v = 0, double a = 0, double heading_degrees_nom = 0) :
        x(x), y(y), z(z), k(k), s(s), v(v), a(a), heading_degrees_nom(heading_degrees_nom) {}

    double x{};
    double y{};
    double z{}; // Heading
    double k{}; // Curvature
    double s{};
    double v{};
    double a{};
    double heading_degrees_nom{};
};

inline State local2Global(const State& reference, const State& target) {
    double x = target.x * cos(reference.z) - target.y * sin(reference.z) + reference.x;
    double y = target.x * sin(reference.z) + target.y * cos(reference.z) + reference.y;
    double z = reference.z + target.z;
    return {x, y, z, target.k, target.s};
}

inline double distance(const State& p1, const State& p2) {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

} // namespace PathOptimizationNS

#endif // PATHOPTIMIZATIONNS_H
