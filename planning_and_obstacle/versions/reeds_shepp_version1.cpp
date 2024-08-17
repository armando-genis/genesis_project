#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <optional>
#include <tuple>

enum class Steering { LEFT = -1, RIGHT = 1, STRAIGHT = 0 };
enum class Gear { FORWARD = 1, BACKWARD = -1 };

Steering negate_steering(Steering steering) {
    switch (steering) {
        case Steering::LEFT: return Steering::RIGHT;
        case Steering::RIGHT: return Steering::LEFT;
        case Steering::STRAIGHT: return Steering::STRAIGHT;
    }
    return Steering::STRAIGHT; // Default case (should never reach here)
}

Gear negate_gear(Gear gear) {
    switch (gear) {
        case Gear::FORWARD: return Gear::BACKWARD;
        case Gear::BACKWARD: return Gear::FORWARD;
    }
    return Gear::FORWARD; // Default case (should never reach here)
}

struct PathElement {
    double param;
    Steering steering;
    Gear gear;

    PathElement(double p, Steering s, Gear g) : param(p), steering(s), gear(g) {}

    static PathElement create(double param, Steering steering, Gear gear) {
        if (param >= 0) {
            return PathElement(param, steering, gear);
        } else {
            return PathElement(-param, steering, gear).reverse_gear();
        }
    }

    PathElement reverse_steering() const {
        return PathElement(param, negate_steering(steering), gear);
    }

    PathElement reverse_gear() const {
        return PathElement(param, steering, negate_gear(gear));
    }

    void print() const {
        std::cout << "{ Steering: " << static_cast<int>(steering) 
                  << "\tGear: " << static_cast<int>(gear) 
                  << "\tdistance: " << param << " }" << std::endl;
    }
};

using Path = std::vector<PathElement>;

class ReedsShepp {
public:
    // Computes the optimal path between the start and end positions
    static Path get_optimal_path(const std::tuple<double, double, double>& start, const std::tuple<double, double, double>& end) {
        double start_x, start_y, start_phi, end_x, end_y, end_phi;
        std::tie(start_x, start_y, start_phi) = start;
        std::tie(end_x, end_y, end_phi) = end;
        auto paths = get_all_paths(end_x - start_x, end_y - start_y, end_phi - start_phi);
        return *std::min_element(paths.begin(), paths.end(), 
                                 [](const Path& a, const Path& b) { return path_length(a) < path_length(b); });
    }

    // Calculates the total length of the given path
    static double path_length(const Path& path) {
        double length = 0.0;
        for (const auto& e : path) {
            length += e.param;
        }
        return length;
    }

private:
    // Converts degrees to radians
    static double deg2rad(double deg) {
        return deg * M_PI / 180.0;
    }

    // Normalizes an angle to the range [0, 2π)
    static double mod2pi(double x) {
        while (x < 0) x += 2 * M_PI;
        while (x >= 2 * M_PI) x -= 2 * M_PI;
        return x;
    }

    // Normalizes an angle
    static double M(double angle) {
        return mod2pi(angle);
    }

    // Reverses the gear of each element in the path
    static Path timeflip(const Path& path) {
        Path new_path;
        for (const auto& e : path) {
            new_path.push_back(e.reverse_gear());
        }
        return new_path;
    }

    // Reverses the steering of each element in the path
    static Path reflect(const Path& path) {
        Path new_path;
        for (const auto& e : path) {
            new_path.push_back(e.reverse_steering());
        }
        return new_path;
    }

    // Generates all possible paths between the start and end positions
    static std::vector<Path> get_all_paths(double x, double y, double phi) {
        using PathFunc = Path(*)(double, double, double);
        std::vector<PathFunc> path_fns = { path1, path2, path3, path4, path5, path6, path7, path8, path9, path10, path11, path12 };
        std::vector<Path> paths;

        for (auto get_path : path_fns) {
            paths.push_back(get_path(x, y, phi));
            paths.push_back(timeflip(get_path(-x, y, -phi)));
            paths.push_back(reflect(get_path(x, -y, -phi)));
            paths.push_back(reflect(timeflip(get_path(-x, -y, phi))));
        }

        for (auto& path : paths) {
            path.erase(std::remove_if(path.begin(), path.end(), [](const PathElement& e) { return e.param == 0; }), path.end());
        }

        paths.erase(std::remove_if(paths.begin(), paths.end(), [](const Path& path) { return path.empty(); }), paths.end());

        return paths;
    }

    // Computes the polar coordinates (ρ, θ) given Cartesian coordinates (x, y)
    static std::optional<std::pair<double, double>> R(double x, double y) {
        double rho = sqrt(x * x + y * y);
        double theta = atan2(y, x);
        if (rho == 0.0) {
            return std::nullopt;
        }
        return std::make_pair(rho, theta);
    }

    // Computes the path of type 1 (CSC, same turns)
    static Path path1(double x, double y, double phi) {
        phi = deg2rad(phi);
        Path path;

        auto [u, t] = *R(x - sin(phi), y - 1 + cos(phi));
        double v = M(phi - t);

        path.push_back(PathElement::create(t, Steering::LEFT, Gear::FORWARD));
        path.push_back(PathElement::create(u, Steering::STRAIGHT, Gear::FORWARD));
        path.push_back(PathElement::create(v, Steering::LEFT, Gear::FORWARD));

        return path;
    }

    // Computes the path of type 2 (CSC, opposite turns)
    static Path path2(double x, double y, double phi) {
        phi = M(deg2rad(phi));
        Path path;

        auto [rho, t1] = *R(x + sin(phi), y - 1 - cos(phi));

        if (rho * rho >= 4) {
            double u = sqrt(rho * rho - 4);
            double t = M(t1 + atan2(2, u));
            double v = M(t - phi);

            path.push_back(PathElement::create(t, Steering::LEFT, Gear::FORWARD));
            path.push_back(PathElement::create(u, Steering::STRAIGHT, Gear::FORWARD));
            path.push_back(PathElement::create(v, Steering::RIGHT, Gear::FORWARD));
        }

        return path;
    }

    // Computes the path of type 3 (C|C|C)
    static Path path3(double x, double y, double phi) {
        phi = deg2rad(phi);
        Path path;

        double xi = x - sin(phi);
        double eta = y - 1 + cos(phi);
        auto [rho, theta] = *R(xi, eta);

        if (rho <= 4) {
            double A = acos(rho / 4);
            double t = M(theta + M_PI/2 + A);
            double u = M(M_PI - 2*A);
            double v = M(phi - t - u);

            path.push_back(PathElement::create(t, Steering::LEFT, Gear::FORWARD));
            path.push_back(PathElement::create(u, Steering::RIGHT, Gear::BACKWARD));
            path.push_back(PathElement::create(v, Steering::LEFT, Gear::FORWARD));
        }

        return path;
    }

    // Computes the path of type 4 (C|CC)
    static Path path4(double x, double y, double phi) {
        phi = deg2rad(phi);
        Path path;

        double xi = x - sin(phi);
        double eta = y - 1 + cos(phi);
        auto [rho, theta] = *R(xi, eta);

        if (rho <= 4) {
            double A = acos(rho / 4);
            double t = M(theta + M_PI/2 + A);
            double u = M(M_PI - 2*A);
            double v = M(t + u - phi);

            path.push_back(PathElement::create(t, Steering::LEFT, Gear::FORWARD));
            path.push_back(PathElement::create(u, Steering::RIGHT, Gear::BACKWARD));
            path.push_back(PathElement::create(v, Steering::LEFT, Gear::BACKWARD));
        }

        return path;
    }

    // Computes the path of type 5 (CC|C)
    static Path path5(double x, double y, double phi) {
        phi = deg2rad(phi);
        Path path;

        double xi = x - sin(phi);
        double eta = y - 1 + cos(phi);
        auto [rho, theta] = *R(xi, eta);

        if (rho <= 4) {
            double u = acos(1 - rho * rho / 8);
            double A = asin(2 * sin(u) / rho);
            double t = M(theta + M_PI/2 - A);
            double v = M(t - u - phi);

            path.push_back(PathElement::create(t, Steering::LEFT, Gear::FORWARD));
            path.push_back(PathElement::create(u, Steering::RIGHT, Gear::FORWARD));
            path.push_back(PathElement::create(v, Steering::LEFT, Gear::BACKWARD));
        }

        return path;
    }

    // Computes the path of type 6 (CCu|CuC)
    static Path path6(double x, double y, double phi) {
        phi = deg2rad(phi);
        Path path;

        double xi = x + sin(phi);
        double eta = y - 1 - cos(phi);
        auto [rho, theta] = *R(xi, eta);

        if (rho <= 4) {
            if (rho <= 2) {
                double A = acos((rho + 2) / 4);
                double t = M(theta + M_PI/2 + A);
                double u = M(A);
                double v = M(phi - t + 2*u);

                path.push_back(PathElement::create(t, Steering::LEFT, Gear::FORWARD));
                path.push_back(PathElement::create(u, Steering::RIGHT, Gear::FORWARD));
                path.push_back(PathElement::create(u, Steering::LEFT, Gear::BACKWARD));
                path.push_back(PathElement::create(v, Steering::RIGHT, Gear::BACKWARD));
            } else {
                double A = acos((rho - 2) / 4);
                double t = M(theta + M_PI/2 - A);
                double u = M(M_PI - A);
                double v = M(phi - t + 2*u);

                path.push_back(PathElement::create(t, Steering::LEFT, Gear::FORWARD));
                path.push_back(PathElement::create(u, Steering::RIGHT, Gear::FORWARD));
                path.push_back(PathElement::create(u, Steering::LEFT, Gear::BACKWARD));
                path.push_back(PathElement::create(v, Steering::RIGHT, Gear::BACKWARD));
            }
        }

        return path;
    }

    // Computes the path of type 7 (C|CuCu|C)
    static Path path7(double x, double y, double phi) {
        phi = deg2rad(phi);
        Path path;

        double xi = x + sin(phi);
        double eta = y - 1 - cos(phi);
        auto [rho, theta] = *R(xi, eta);
        double u1 = (20 - rho * rho) / 16;

        if (rho <= 6 && 0 <= u1 && u1 <= 1) {
            double u = acos(u1);
            double A = asin(2 * sin(u) / rho);
            double t = M(theta + M_PI/2 + A);
            double v = M(t - phi);

            path.push_back(PathElement::create(t, Steering::LEFT, Gear::FORWARD));
            path.push_back(PathElement::create(u, Steering::RIGHT, Gear::BACKWARD));
            path.push_back(PathElement::create(u, Steering::LEFT, Gear::BACKWARD));
            path.push_back(PathElement::create(v, Steering::RIGHT, Gear::FORWARD));
        }

        return path;
    }

    // Computes the path of type 8 (C|C[pi/2]SC)
    static Path path8(double x, double y, double phi) {
        phi = deg2rad(phi);
        Path path;

        double xi = x - sin(phi);
        double eta = y - 1 + cos(phi);
        auto [rho, theta] = *R(xi, eta);

        if (rho >= 2) {
            double u = sqrt(rho * rho - 4) - 2;
            double A = atan2(2, u + 2);
            double t = M(theta + M_PI/2 + A);
            double v = M(t - phi + M_PI/2);

            path.push_back(PathElement::create(t, Steering::LEFT, Gear::FORWARD));
            path.push_back(PathElement::create(M_PI/2, Steering::RIGHT, Gear::BACKWARD));
            path.push_back(PathElement::create(u, Steering::STRAIGHT, Gear::BACKWARD));
            path.push_back(PathElement::create(v, Steering::LEFT, Gear::BACKWARD));
        }

        return path;
    }

    // Computes the path of type 9 (CSC[pi/2]|C)
    static Path path9(double x, double y, double phi) {
        phi = deg2rad(phi);
        Path path;

        double xi = x - sin(phi);
        double eta = y - 1 + cos(phi);
        auto [rho, theta] = *R(xi, eta);

        if (rho >= 2) {
            double u = sqrt(rho * rho - 4) - 2;
            double A = atan2(u + 2, 2);
            double t = M(theta + M_PI/2 - A);
            double v = M(t - phi - M_PI/2);

            path.push_back(PathElement::create(t, Steering::LEFT, Gear::FORWARD));
            path.push_back(PathElement::create(u, Steering::STRAIGHT, Gear::FORWARD));
            path.push_back(PathElement::create(M_PI/2, Steering::RIGHT, Gear::FORWARD));
            path.push_back(PathElement::create(v, Steering::LEFT, Gear::BACKWARD));
        }

        return path;
    }

    // Computes the path of type 10 (C|C[pi/2]SC)
    static Path path10(double x, double y, double phi) {
        phi = deg2rad(phi);
        Path path;

        double xi = x + sin(phi);
        double eta = y - 1 - cos(phi);
        auto [rho, theta] = *R(xi, eta);

        if (rho >= 2) {
            double t = M(theta + M_PI/2);
            double u = rho - 2;
            double v = M(phi - t - M_PI/2);

            path.push_back(PathElement::create(t, Steering::LEFT, Gear::FORWARD));
            path.push_back(PathElement::create(M_PI/2, Steering::RIGHT, Gear::BACKWARD));
            path.push_back(PathElement::create(u, Steering::STRAIGHT, Gear::BACKWARD));
            path.push_back(PathElement::create(v, Steering::RIGHT, Gear::BACKWARD));
        }

        return path;
    }

    // Computes the path of type 11 (CSC[pi/2]|C)
    static Path path11(double x, double y, double phi) {
        phi = deg2rad(phi);
        Path path;

        double xi = x + sin(phi);
        double eta = y - 1 - cos(phi);
        auto [rho, theta] = *R(xi, eta);

        if (rho >= 2) {
            double t = M(theta);
            double u = rho - 2;
            double v = M(phi - t - M_PI/2);

            path.push_back(PathElement::create(t, Steering::LEFT, Gear::FORWARD));
            path.push_back(PathElement::create(u, Steering::STRAIGHT, Gear::FORWARD));
            path.push_back(PathElement::create(M_PI/2, Steering::LEFT, Gear::FORWARD));
            path.push_back(PathElement::create(v, Steering::RIGHT, Gear::BACKWARD));
        }

        return path;
    }

    // Computes the path of type 12 (C|C[pi/2]SC[pi/2]|C)
    static Path path12(double x, double y, double phi) {
        phi = deg2rad(phi);
        Path path;

        double xi = x + sin(phi);
        double eta = y - 1 - cos(phi);
        auto [rho, theta] = *R(xi, eta);

        if (rho >= 4) {
            double u = sqrt(rho * rho - 4) - 4;
            double A = atan2(2, u + 4);
            double t = M(theta + M_PI/2 + A);
            double v = M(t - phi);

            path.push_back(PathElement::create(t, Steering::LEFT, Gear::FORWARD));
            path.push_back(PathElement::create(M_PI/2, Steering::RIGHT, Gear::BACKWARD));
            path.push_back(PathElement::create(u, Steering::STRAIGHT, Gear::BACKWARD));
            path.push_back(PathElement::create(M_PI/2, Steering::LEFT, Gear::BACKWARD));
            path.push_back(PathElement::create(v, Steering::RIGHT, Gear::FORWARD));
        }

        return path;
    }
};

int main() {
    std::vector<std::tuple<double, double, double>> ROUTE = {
        {-2, 4, 180}, {2, 4, 0}, {2, -3, 90}, {-5, -6, 240}, {-6, -7, 160}, {-7, -1, 80}
    };

    std::vector<PathElement> full_path;
    double total_length = 0.0;

    for (size_t i = 0; i < ROUTE.size() - 1; ++i) {
        auto path = ReedsShepp::get_optimal_path(ROUTE[i], ROUTE[i + 1]);
        full_path.insert(full_path.end(), path.begin(), path.end());
        total_length += ReedsShepp::path_length(path);
    }

    std::cout << "Shortest path length: " << round(total_length * 100) / 100 << std::endl;

    for (const auto& elem : full_path) {
        elem.print();
    }

    return 0;
}
