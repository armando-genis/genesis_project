#include "reeds_shepp.h"
#include <algorithm>


// Function implementations
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

// PathElement methods
PathElement::PathElement(double p, Steering s, Gear g) : param(p), steering(s), gear(g) {}

PathElement PathElement::create(double param, Steering steering, Gear gear) {
    if (param >= 0) {
        return PathElement(param, steering, gear);
    } else {
        return PathElement(-param, steering, gear).reverse_gear();
    }
}

PathElement PathElement::reverse_steering() const {
    return PathElement(param, negate_steering(steering), gear);
}

PathElement PathElement::reverse_gear() const {
    return PathElement(param, steering, negate_gear(gear));
}

void PathElement::print() const {
    std::cout << "{ Steering: " << static_cast<int>(steering) 
              << "\tGear: " << static_cast<int>(gear) 
              << "\tdistance: " << param << " }" << std::endl;
}

// ReedsShepp methods
Path ReedsShepp::get_optimal_path(const std::tuple<double, double, double>& start, const std::tuple<double, double, double>& end) {
    double start_x, start_y, start_phi, end_x, end_y, end_phi;
    std::tie(start_x, start_y, start_phi) = start;
    std::tie(end_x, end_y, end_phi) = end;
    auto paths = get_all_paths(end_x - start_x, end_y - start_y, end_phi - start_phi);
    return *std::min_element(paths.begin(), paths.end(), 
                             [](const Path& a, const Path& b) { return path_length(a) < path_length(b); });
}

double ReedsShepp::path_length(const Path& path) {
    double length = 0.0;
    for (const auto& e : path) {
        length += e.param;
    }
    return length;
}

double ReedsShepp::deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

double ReedsShepp::mod2pi(double x) {
    while (x < 0) x += 2 * M_PI;
    while (x >= 2 * M_PI) x -= 2 * M_PI;
    return x;
}

double ReedsShepp::M(double angle) {
    return mod2pi(angle);
}

Path ReedsShepp::timeflip(const Path& path) {
    Path new_path;
    for (const auto& e : path) {
        new_path.push_back(e.reverse_gear());
    }
    return new_path;
}

Path ReedsShepp::reflect(const Path& path) {
    Path new_path;
    for (const auto& e : path) {
        new_path.push_back(e.reverse_steering());
    }
    return new_path;
}

std::vector<Path> ReedsShepp::get_all_paths(double x, double y, double phi) {
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

std::optional<std::pair<double, double>> ReedsShepp::R(double x, double y) {
    double rho = sqrt(x * x + y * y);
    double theta = atan2(y, x);
    if (rho == 0.0) {
        return std::nullopt;
    }
    return std::make_pair(rho, theta);
}

Path ReedsShepp::path1(double x, double y, double phi) {
    phi = deg2rad(phi);
    Path path;

    auto [u, t] = *R(x - sin(phi), y - 1 + cos(phi));
    double v = M(phi - t);

    path.push_back(PathElement::create(t, Steering::LEFT, Gear::FORWARD));
    path.push_back(PathElement::create(u, Steering::STRAIGHT, Gear::FORWARD));
    path.push_back(PathElement::create(v, Steering::LEFT, Gear::FORWARD));

    return path;
}

Path ReedsShepp::path2(double x, double y, double phi) {
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

Path ReedsShepp::path3(double x, double y, double phi) {
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

Path ReedsShepp::path4(double x, double y, double phi) {
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

Path ReedsShepp::path5(double x, double y, double phi) {
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
        path.push_back(PathElement::create(u, Steering::STRAIGHT, Gear::FORWARD));
        path.push_back(PathElement::create(v, Steering::LEFT, Gear::BACKWARD));
    }

    return path;
}

Path ReedsShepp::path6(double x, double y, double phi) {
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

Path ReedsShepp::path7(double x, double y, double phi) {
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

Path ReedsShepp::path8(double x, double y, double phi) {
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

Path ReedsShepp::path9(double x, double y, double phi) {
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

Path ReedsShepp::path10(double x, double y, double phi) {
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

Path ReedsShepp::path11(double x, double y, double phi) {
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

Path ReedsShepp::path12(double x, double y, double phi) {
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
