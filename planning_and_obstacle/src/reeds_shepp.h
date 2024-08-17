#ifndef REEDS_SHEPP_H
#define REEDS_SHEPP_H

#include <iostream>
#include <vector>
#include <cmath>
#include <optional>
#include <tuple>

enum class Steering { LEFT = -1, RIGHT = 1, STRAIGHT = 0 };
enum class Gear { FORWARD = 1, BACKWARD = -1 };

// Function declarations
Steering negate_steering(Steering steering);
Gear negate_gear(Gear gear);

// Struct for path elements
struct PathElement {
    double param;
    Steering steering;
    Gear gear;

    PathElement(double p, Steering s, Gear g);

    static PathElement create(double param, Steering steering, Gear gear);

    PathElement reverse_steering() const;
    PathElement reverse_gear() const;

    void print() const;
};

using Path = std::vector<PathElement>;

// Class for Reeds-Shepp path planning
class ReedsShepp {
public:
    static Path get_optimal_path(const std::tuple<double, double, double>& start, const std::tuple<double, double, double>& end);
    static double path_length(const Path& path);

private:
    static double deg2rad(double deg);
    static double mod2pi(double x);
    static double M(double angle);
    static Path timeflip(const Path& path);
    static Path reflect(const Path& path);
    static std::vector<Path> get_all_paths(double x, double y, double phi);
    static std::optional<std::pair<double, double>> R(double x, double y);
    static Path path1(double x, double y, double phi);
    static Path path2(double x, double y, double phi);
    static Path path3(double x, double y, double phi);
    static Path path4(double x, double y, double phi);
    static Path path5(double x, double y, double phi);
    static Path path6(double x, double y, double phi);
    static Path path7(double x, double y, double phi);
    static Path path8(double x, double y, double phi);
    static Path path9(double x, double y, double phi);
    static Path path10(double x, double y, double phi);
    static Path path11(double x, double y, double phi);
    static Path path12(double x, double y, double phi);
};

#endif // REEDS_SHEPP_H
