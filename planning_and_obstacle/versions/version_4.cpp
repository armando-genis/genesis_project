#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <vector>
#include <cmath>
#include <cfloat>
#include <sstream>
#include <iomanip>

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
    std::vector<bool> collision_vector;
    // Vehicle parameters
    static constexpr double car_width = 2.0;
    static constexpr double car_length = 4.0;
    static constexpr double rtc = 1.5;
    static constexpr double rear_d = car_length / 2 - rtc;
    static constexpr double front_d = car_length - rear_d;


    // Occupancy Grid Map
    nav_msgs::msg::OccupancyGrid occ_map_data_;
    double resolution;
    double originX;
    double originY;
    uint32_t width;
    uint32_t height;

    // Vehicle geometry
    geometry_msgs::msg::Polygon vehicle_geometry;
    

    // States
    PathOptimizationNS::State start_state, end_state;
    std::vector<PathOptimizationNS::State> reference_path;
    bool start_state_rcv = false, end_state_rcv = false, reference_rcv = false;

    // Publishers
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr polygon_pub_;

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr reference_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr start_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_map_sub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    void visualize_vehicle_geometry(const std::vector<PathOptimizationNS::State> &result_path);
    void referenceCb(const geometry_msgs::msg::PointStamped::SharedPtr p);
    void startCb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr start);
    void goalCb(const geometry_msgs::msg::PoseStamped::SharedPtr goal);
    void processAndVisualize();
    void gridMapdata(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    bool checkCollisionAtStart(const PathOptimizationNS::State state, const std::string& hex_color );
    geometry_msgs::msg::Polygon createObstaclePolygon(double x, double y);


public:
    planning_and_obstacle();
    ~planning_and_obstacle();
};

planning_and_obstacle::planning_and_obstacle() : Node("optimal_planner_node")
{
    // Initialize publisher
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("vehicle_markers", 10);
    polygon_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("vehicle_polygon", 10);

    // Initialize subscribers
    reference_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/clicked_point", 10, std::bind(&planning_and_obstacle::referenceCb, this, std::placeholders::_1));
    start_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 10, std::bind(&planning_and_obstacle::startCb, this, std::placeholders::_1));
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10, std::bind(&planning_and_obstacle::goalCb, this, std::placeholders::_1));
    grid_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/grid_map", 10, std::bind(&planning_and_obstacle::gridMapdata, this, std::placeholders::_1));


    // Initialize timer to call processAndVisualize() every second
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&planning_and_obstacle::processAndVisualize, this)
    );

    // add the vehicle geometry to the vehicle_geometry polygon
    geometry_msgs::msg::Point32 p1, p2, p3, p4;
    p1.x = front_d;
    p1.y = car_width / 2;
    p2.x = front_d;
    p2.y = -car_width / 2;
    p3.x = -rear_d;
    p3.y = -car_width / 2;
    p4.x = -rear_d;
    p4.y = car_width / 2;

    vehicle_geometry.points.push_back(p1);
    vehicle_geometry.points.push_back(p2);
    vehicle_geometry.points.push_back(p3);
    vehicle_geometry.points.push_back(p4);

    vehicle_geometry.points.push_back(vehicle_geometry.points.front());

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

    if (checkCollisionAtStart(start_state, "#37FF02")) 
    {
        RCLCPP_INFO(this->get_logger(), "\033[1;31m----> Collision detected at start state.\033[0m");
    } 
    else 
    {
        RCLCPP_INFO(this->get_logger(), "\033[1;32m----> No collision detected at start state.\033[0m");
    }

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

    checkCollisionAtStart(end_state,"#FF3002");

    RCLCPP_INFO(this->get_logger(), "Received goal state");
}

void planning_and_obstacle::processAndVisualize()
{
    // RCLCPP_INFO(this->get_logger(), "Start state received: %d, End state received: %d", start_state_rcv, end_state_rcv);


    if (!start_state_rcv || !end_state_rcv ) {
        return; // Do not process if initial and goal states are not received
    }

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

}

bool planning_and_obstacle::checkCollisionAtStart(const PathOptimizationNS::State state, const std::string& hex_color)
{

    // Define the 5 meter offset
    double offset = 5.0;

    // Define the bounds for the region to check
    double min_x = std::max(originX, state.x - offset);
    double max_x = std::min(originX + width * resolution, state.x + offset);
    double min_y = std::max(originY, state.y - offset);
    double max_y = std::min(originY + height * resolution, state.y + offset);

    // Convert bounds to grid indices
    uint32_t min_i = std::max(0, static_cast<int>((min_y - originY) / resolution));
    uint32_t max_i = std::min(static_cast<int>(height) - 1, static_cast<int>((max_y - originY) / resolution));
    uint32_t min_j = std::max(0, static_cast<int>((min_x - originX) / resolution));
    uint32_t max_j = std::min(static_cast<int>(width) - 1, static_cast<int>((max_x - originX) / resolution));

    // Convert hex color to RGBA
    if (hex_color.size() != 7 || hex_color[0] != '#') {
        throw std::invalid_argument("Hex color must be in the format #RRGGBB");
    }

    unsigned int hex_value;
    std::stringstream ss;
    ss << std::hex << hex_color.substr(1);
    ss >> hex_value;

    float r = ((hex_value >> 16) & 0xFF) / 255.0f;
    float g = ((hex_value >> 8) & 0xFF) / 255.0f;
    float b = (hex_value & 0xFF) / 255.0f;
    float a = 1.0f; // Assuming full opacity if alpha is not specified

    // log the hex color and the RGBA values
    RCLCPP_INFO(this->get_logger(), "Hex color: %s", hex_color.c_str());
    RCLCPP_INFO(this->get_logger(), "RGBA: %f, %f, %f, %f", r, g, b, a);

    // Publish the vehicle geometry as a polygon
    visualization_msgs::msg::Marker polygon_marker;
    polygon_marker.header.frame_id = "map";
    polygon_marker.header.stamp = this->now();
    polygon_marker.ns = "vehicle_polygon";
    polygon_marker.action = visualization_msgs::msg::Marker::ADD;
    polygon_marker.id = 10;
    polygon_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    polygon_marker.scale.x = 0.05;
    polygon_marker.color.r = r;
    polygon_marker.color.g = g;
    polygon_marker.color.b = b;
    polygon_marker.color.a = a;

    // Create the vehicle polygon at the start state position
    geometry_msgs::msg::Polygon vehicle_poly;
    for (const auto& point : vehicle_geometry.points) {
        geometry_msgs::msg::Point32 p_point32;
        geometry_msgs::msg::Point p_point;

        double x = state.x + point.x * cos(state.z) - point.y * sin(state.z);
        double y = state.y + point.x * sin(state.z) + point.y * cos(state.z);

        p_point32.x = x;
        p_point32.y = y;
        p_point.x = x;
        p_point.y = y;

        polygon_marker.points.push_back(p_point);
        vehicle_poly.points.push_back(p_point32);
    }

    polygon_pub_->publish(polygon_marker);

    // for (uint32_t i = 0; i < height; ++i) {
    //     for (uint32_t j = 0; j < width; ++j) {
    //         double x = originX + j * resolution;
    //         double y = originY + i * resolution;
    //         if (x >= min_x && x <= max_x && y >= min_y && y <= max_y) {
    //             if (occ_map_data_.data[i * width + j] > 0) {
    //                 auto obstacle_poly = createObstaclePolygon(x, y);
    //                 if (collision_checker.check_collision(vehicle_poly, obstacle_poly)) {
    //                     return true; // Collision detected
    //                 }
    //             }
    //         }
    //     }
    // }
    // return false;

    // Check each relevant cell in the occupancy grid
    for (uint32_t i = min_i; i <= max_i; ++i) {
        for (uint32_t j = min_j; j <= max_j; ++j) {
            if (occ_map_data_.data[i * width + j] > 0) {
                double x = originX + j * resolution;
                double y = originY + i * resolution;
                auto obstacle_poly = createObstaclePolygon(x, y);

                if (collision_checker.check_collision(vehicle_poly, obstacle_poly)) {
                    return true; // Collision detected
                }
            }
        }
    }

    return false;
}

geometry_msgs::msg::Polygon planning_and_obstacle::createObstaclePolygon(double x, double y) {
    geometry_msgs::msg::Polygon obstacle_poly;
    double half_res = resolution / 2.0;

    geometry_msgs::msg::Point32 p1, p2, p3, p4;
    p1.x = x - half_res;
    p1.y = y - half_res;
    p2.x = x + half_res;
    p2.y = y - half_res;
    p3.x = x + half_res;
    p3.y = y + half_res;
    p4.x = x - half_res;
    p4.y = y + half_res;

    obstacle_poly.points.push_back(p1);
    obstacle_poly.points.push_back(p2);
    obstacle_poly.points.push_back(p3);
    obstacle_poly.points.push_back(p4);
    obstacle_poly.points.push_back(p1); // Close the polygon

    return obstacle_poly;
}

void planning_and_obstacle::gridMapdata(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    occ_map_data_ = *msg;
    resolution = occ_map_data_.info.resolution; 
    originX = occ_map_data_.info.origin.position.x;
    originY = occ_map_data_.info.origin.position.y;
    width = occ_map_data_.info.width;
    height = occ_map_data_.info.height;

    // log the resolution, origin and size of the map
    // RCLCPP_INFO(this->get_logger(), "Resolution: %f", resolution);
    // RCLCPP_INFO(this->get_logger(), "Origin: (%f, %f)", originX, originY);
    // RCLCPP_INFO(this->get_logger(), "Size: %d x %d", width, height);
    // RCLCPP_INFO(this->get_logger(), "Occupancy grid map received");
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<planning_and_obstacle>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
