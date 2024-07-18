#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
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

double distance(const PathOptimizationNS::State &p1, const PathOptimizationNS::State &p2) {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
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

    // States
    PathOptimizationNS::State start_state, end_state;
    std::vector<PathOptimizationNS::State> reference_path;
    std::vector<std::tuple<PathOptimizationNS::State, double, double>> abnormal_bounds;
    bool start_state_rcv = false, end_state_rcv = false, reference_rcv = false;

    // Publishers
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr reference_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr start_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    void visualize_vehicle_geometry(const std::vector<PathOptimizationNS::State> &result_path);
    void referenceCb(const geometry_msgs::msg::PointStamped::SharedPtr p);
    void startCb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr start);
    void goalCb(const geometry_msgs::msg::PoseStamped::SharedPtr goal);
    void processAndVisualize();

public:
    planning_and_obstacle();
    ~planning_and_obstacle();
};

planning_and_obstacle::planning_and_obstacle() : Node("optimal_planner_node")
{
    // Initialize publisher
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("vehicle_markers", 10);

    // Initialize subscribers
    reference_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/clicked_point", 10, std::bind(&planning_and_obstacle::referenceCb, this, std::placeholders::_1));
    start_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 10, std::bind(&planning_and_obstacle::startCb, this, std::placeholders::_1));
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10, std::bind(&planning_and_obstacle::goalCb, this, std::placeholders::_1));

    // Initialize timer to call processAndVisualize() every second
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&planning_and_obstacle::processAndVisualize, this)
    );

    // // Example path for visualization
    // std::vector<PathOptimizationNS::State> result_path;
    // for (double x = 0.0; x <= 10.0; x += 0.5) {
    //     result_path.push_back({x, 0.0, 0.0});
    // }
    // for (double theta = 0.0; theta <= M_PI / 2; theta += M_PI / 18) {
    //     double x = 10.0 + 5.0 * cos(theta);
    //     double y = 5.0 * sin(theta);
    //     double heading = theta;
    //     result_path.push_back({x, y, heading});
    // }

    // visualize_vehicle_geometry(result_path);

    RCLCPP_INFO(this->get_logger(), "\033[1;32m----> optimal_planner_node initialized.\033[0m");
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

void planning_and_obstacle::referenceCb(const geometry_msgs::msg::PointStamped::SharedPtr p)
{
    if (start_state_rcv && end_state_rcv) {
        reference_path.clear();
    }
    PathOptimizationNS::State reference_point;
    reference_point.x = p->point.x;
    reference_point.y = p->point.y;
    reference_path.emplace_back(reference_point);
    start_state_rcv = end_state_rcv = false;
    reference_rcv = reference_path.size() >= 6;
    RCLCPP_INFO(this->get_logger(), "Received a reference point");
}

void planning_and_obstacle::startCb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr start)
{
    start_state.x = start->pose.pose.position.x;
    start_state.y = start->pose.pose.position.y;

    // Extract yaw from the quaternion
    tf2::Quaternion q(
        start->pose.pose.orientation.x,
        start->pose.pose.orientation.y,
        start->pose.pose.orientation.z,
        start->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    start_state.z = yaw;

    if (reference_rcv) {
        start_state_rcv = true;
    }

    // Create a marker for RViz
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "initial_state";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = start_state.x;
    marker.pose.position.y = start_state.y;
    marker.pose.position.z = 0.0;
    marker.pose.orientation = tf2::toMsg(q);
    marker.scale.x = 2.0;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker_pub_->publish(marker);

    RCLCPP_INFO(this->get_logger(), "Received initial state");
}

void planning_and_obstacle::goalCb(const geometry_msgs::msg::PoseStamped::SharedPtr goal)
{
    end_state.x = goal->pose.position.x;
    end_state.y = goal->pose.position.y;

    // Extract yaw from the quaternion
    tf2::Quaternion q(
        goal->pose.orientation.x,
        goal->pose.orientation.y,
        goal->pose.orientation.z,
        goal->pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    end_state.z = yaw;

    if (reference_rcv) {
        end_state_rcv = true;
    }

    // Create a marker for RViz
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "goal_state";
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = end_state.x;
    marker.pose.position.y = end_state.y;
    marker.pose.position.z = 0.0;
    marker.pose.orientation = tf2::toMsg(q);
    marker.scale.x = 2.0;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker_pub_->publish(marker);

    RCLCPP_INFO(this->get_logger(), "Received goal state");
}

void planning_and_obstacle::processAndVisualize()
{
    // log start state rcv and end state rcv
    // RCLCPP_INFO(this->get_logger(), "Start state received: %d, End state received: %d", start_state_rcv, end_state_rcv);

    if (!start_state_rcv || !end_state_rcv) {
        return; // Do not process if initial and goal states are not received
    }

    // log hellow
    RCLCPP_INFO(this->get_logger(), "Hello");

    visualization_msgs::msg::Marker reference_marker;
    reference_marker.header.frame_id = "map";
    reference_marker.header.stamp = this->now();
    reference_marker.ns = "reference";
    reference_marker.action = visualization_msgs::msg::Marker::ADD;
    reference_marker.pose.orientation.w = 1.0;
    reference_marker.id = 0;
    reference_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    reference_marker.scale.x = 0.5;
    reference_marker.scale.y = 0.5;
    reference_marker.scale.z = 0.5;
    reference_marker.color.a = 1.0;
    reference_marker.color.r = 1.0;
    reference_marker.color.g = 0.0;
    reference_marker.color.b = 0.0;

    if (reference_path.size() >= 2) {
        const auto &p1 = reference_path[reference_path.size() - 2];
        const auto &p2 = reference_path.back();
        if (distance(p1, p2) <= 0.001) {
            reference_path.clear();
            reference_rcv = false;
        }
    }

    for (const auto &point : reference_path) {
        geometry_msgs::msg::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = 1.0;
        reference_marker.points.push_back(p);
    }
    marker_pub_->publish(reference_marker);

    // Visualize start and end points
    visualization_msgs::msg::Marker start_marker, end_marker;
    start_marker.header.frame_id = end_marker.header.frame_id = "map";
    start_marker.header.stamp = end_marker.header.stamp = this->now();
    start_marker.ns = end_marker.ns = "points";
    start_marker.action = end_marker.action = visualization_msgs::msg::Marker::ADD;
    start_marker.id = 1;
    end_marker.id = 2;
    start_marker.type = end_marker.type = visualization_msgs::msg::Marker::ARROW;
    start_marker.scale.x = end_marker.scale.x = 2.0;
    start_marker.scale.y = end_marker.scale.y = 0.3;
    start_marker.scale.z = end_marker.scale.z = 0.3;
    start_marker.color.a = end_marker.color.a = 1.0;
    start_marker.color.r = end_marker.color.r = 0.0;
    start_marker.color.g = 1.0;
    start_marker.color.b = end_marker.color.b = 1.0;
    end_marker.color.g = 1.0;
    end_marker.color.b = 0.0;

    geometry_msgs::msg::Pose start_pose, end_pose;
    start_pose.position.x = start_state.x;
    start_pose.position.y = start_state.y;
    start_pose.position.z = 1.0;
    tf2::Quaternion start_quat;
    start_quat.setRPY(0, 0, start_state.z);
    start_pose.orientation = tf2::toMsg(start_quat);
    start_marker.pose = start_pose;

    end_pose.position.x = end_state.x;
    end_pose.position.y = end_state.y;
    end_pose.position.z = 1.0;
    tf2::Quaternion end_quat;
    end_quat.setRPY(0, 0, end_state.z);
    end_pose.orientation = tf2::toMsg(end_quat);
    end_marker.pose = end_pose;

    marker_pub_->publish(start_marker);
    marker_pub_->publish(end_marker);

    // Further processing and visualization of paths can be added here
}




int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<planning_and_obstacle>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
