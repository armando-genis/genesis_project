#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <cfloat>

// Assuming the correct include for SATCollisionChecker
#include "sat_collision_checker.h" // Ensure this include is correct

namespace PathOptimizationNS {

// Standard point struct.
struct State {
    State() = default;
    State(double x, double y, double z = 0, double k = 0, double s = 0, double v = 0, double a = 0) :
        x(x), y(y), z(z), k(k), s(s), v(v), a(a) {}
    double x{};
    double y{};
    double z{}; // Heading.
    double k{}; // Curvature.
    double s{};
    double v{};
    double a{};
};

class Box {
 public:
    enum Dir{LEFT, RIGHT, UNKNOWN};
    Box() = delete;
    Box(double x, double y, double heading, double length, double width) :
        center_{x, y, heading}, length_(length), width_(width), dir_(UNKNOWN) {}
    Box(double x, double y, double heading, double length, double width, bool is_left) :
        center_{x, y, heading}, length_(length), width_(width), dir_(is_left ? LEFT : RIGHT) {}
    State getCenter() const { return center_; }
    double getX() const { return center_.x; }
    double getY() const { return center_.y; }
    double getHeading() const { return center_.z; }
    double getLength() const { return length_; }
    double getWidth() const { return width_; }
    double distanceTo(const State &point) const {
        return std::sqrt(std::pow(center_.x - point.x, 2) + std::pow(center_.y - point.y, 2));
    }
    Dir getDir() const { return dir_; }
    void setDir(Box::Dir dir) { dir_ = dir; }
 private:
    State center_;
    double length_, width_;
    Dir dir_;
};

struct Circle {
    Circle() = default;
    Circle(double x, double y, double r) : x(x), y(y), r(r) {}
    double x{};
    double y{};
    double r{};
};

class BoxByCircles {
 public:
    BoxByCircles() = delete;
    BoxByCircles(const Box &box) : dir_(box.getDir()) {
        // Example implementation, you should adjust according to your logic
        circles_.emplace_back(box.getX(), box.getY(), box.getLength() / 2);
    }
    const std::vector<Circle> &getCircles() const { return circles_; }
    Box::Dir getDir() const { return dir_; }
 private:
    std::vector<Circle> circles_;
    Box::Dir dir_;
};

struct CoveringCircleBounds {
    CoveringCircleBounds() = default;
    struct SingleCircleBounds {
        SingleCircleBounds() = default;
        SingleCircleBounds &operator=(const std::vector<double> &bounds) {
            ub = bounds[0];
            lb = bounds[1];
            return *this;
        }
        void set(const std::vector<double> &bounds, const State &center) {
            ub = bounds[0];
            lb = bounds[1];
            x = center.x;
            y = center.y;
            heading = center.z;
        }
        double ub{}; // left
        double lb{}; // right
        double x{}, y{}, heading{};
    } c0, c1, c2, c3;
};

struct APoint {
    double x{};
    double y{};
    double s{};
    double l{};
    double g{};
    double h{};
    double dir{};
    int layer{-1};
    int offset_idx{};
    double rough_upper_bound, rough_lower_bound;
    bool is_in_open_set{false};
    APoint *parent{nullptr};
    inline double f() {
        return g + h;
    }
};

struct DpPoint {
    double x_, y_, heading_, s_, l_, cost_ = DBL_MAX, dir_, dis_to_obs_;
    int layer_index_, lateral_index_;
    double rough_upper_bound, rough_lower_bound;
    const DpPoint *parent_ = nullptr;
    bool is_feasible_ = true;
};

class PointComparator {
 public:
    bool operator()(APoint *a, APoint *b) {
        return a->f() > b->f();
    }
};

State local2Global(const State &reference, const State &target) {
    double x = target.x * cos(reference.z) - target.y * sin(reference.z) + reference.x;
    double y = target.x * sin(reference.z) + target.y * cos(reference.z) + reference.y;
    double z = reference.z + target.z;
    return {x, y, z, target.k, target.s};
}

} // namespace PathOptimizationNS

class planning_and_obstacle : public rclcpp::Node
{
private:
    // Collision checker
    fop::SATCollisionChecker collision_checker; 
    bool collision_detected = false;
    // Vehicle parameters
    static constexpr double car_width = 2.0;
    static constexpr double car_length = 4.0;
    static constexpr double rtc = 1.5;
    static constexpr double rear_d = car_length / 2 - rtc;
    static constexpr double front_d = car_length - rear_d;

    // Publishers
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    void visualize_vehicle_geometry(const std::vector<PathOptimizationNS::State> &result_path);

public:
    planning_and_obstacle();
    ~planning_and_obstacle();
};

planning_and_obstacle::planning_and_obstacle() : Node("optimal_planner_node")
{
    RCLCPP_INFO(this->get_logger(), "\033[1;32m----> optimal_planner_node initialized.\033[0m");

    // Initialize publisher
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("vehicle_markers", 10);

    // Example path
    std::vector<PathOptimizationNS::State> result_path;
    // Straight line
    for (double x = 0.0; x <= 10.0; x += 0.5) {
        result_path.push_back({x, 0.0, 0.0});
    }
    // Curve
    for (double theta = 0.0; theta <= M_PI / 2; theta += M_PI / 18) {
        double x = 10.0 + 5.0 * cos(theta);
        double y = 5.0 * sin(theta);
        double heading = theta;
        result_path.push_back({x, y, heading});
    }

    // Visualize vehicle geometry
    visualize_vehicle_geometry(result_path);
}

planning_and_obstacle::~planning_and_obstacle() {}

void planning_and_obstacle::visualize_vehicle_geometry(const std::vector<PathOptimizationNS::State> &result_path)
{
    visualization_msgs::msg::Marker vehicle_geometry_marker;
    vehicle_geometry_marker.header.frame_id = "map";
    vehicle_geometry_marker.header.stamp = this->now();
    vehicle_geometry_marker.ns = "vehicle";
    vehicle_geometry_marker.action = visualization_msgs::msg::Marker::ADD;
    vehicle_geometry_marker.pose.orientation.w = 1.0;
    vehicle_geometry_marker.id = 0;
    vehicle_geometry_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    vehicle_geometry_marker.scale.x = 0.05;
    vehicle_geometry_marker.color.a = 1.0;
    vehicle_geometry_marker.color.r = 0.5;
    vehicle_geometry_marker.color.g = 0.5;
    vehicle_geometry_marker.color.b = 0.5;

    for (const auto &state : result_path) {
        double heading = state.z;
        PathOptimizationNS::State p1, p2, p3, p4;
        p1.x = front_d;
        p1.y = car_width / 2;
        p2.x = front_d;
        p2.y = -car_width / 2;
        p3.x = -rear_d;
        p3.y = -car_width / 2;
        p4.x = -rear_d;
        p4.y = car_width / 2;

        auto tmp_relto = state;
        tmp_relto.z = heading;
        p1 = PathOptimizationNS::local2Global(tmp_relto, p1);
        p2 = PathOptimizationNS::local2Global(tmp_relto, p2);
        p3 = PathOptimizationNS::local2Global(tmp_relto, p3);
        p4 = PathOptimizationNS::local2Global(tmp_relto, p4);

        geometry_msgs::msg::Point pp1, pp2, pp3, pp4;
        pp1.x = p1.x;
        pp1.y = p1.y;
        pp1.z = 0.1;
        pp2.x = p2.x;
        pp2.y = p2.y;
        pp2.z = 0.1;
        pp3.x = p3.x;
        pp3.y = p3.y;
        pp3.z = 0.1;
        pp4.x = p4.x;
        pp4.y = p4.y;
        pp4.z = 0.1;

        vehicle_geometry_marker.points.push_back(pp1);
        vehicle_geometry_marker.points.push_back(pp2);
        vehicle_geometry_marker.points.push_back(pp2);
        vehicle_geometry_marker.points.push_back(pp3);
        vehicle_geometry_marker.points.push_back(pp3);
        vehicle_geometry_marker.points.push_back(pp4);
        vehicle_geometry_marker.points.push_back(pp4);
        vehicle_geometry_marker.points.push_back(pp1);
    }

    marker_pub_->publish(vehicle_geometry_marker);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<planning_and_obstacle>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
