#include "rrt_planner.h"
#include "sat_collision_checker.h"
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <algorithm>

/**
 * @brief Construct a new RRTPlanner object and initialize random seed.
 */
RRTPlanner::RRTPlanner() : epsilon_min_(0.1), epsilon_max_(0.5), tolerance_(0.5), max_node_num_(1000) {
    srand((unsigned)time(NULL));
}

/**
 * @brief Set the occupancy grid used for collision checking.
 * 
 * @param occ_map Occupancy grid map.
 */
void RRTPlanner::setOccupancyGrid(const nav_msgs::msg::OccupancyGrid &occ_map) {
    occ_map_ = occ_map;
}

/**
 * @brief Initialize the RRT planner with map parameters.
 * 
 * @param originX Origin X coordinate of the map.
 * @param originY Origin Y coordinate of the map.
 * @param resolution Resolution of the occupancy grid.
 * @param width Width of the occupancy grid.
 * @param height Height of the occupancy grid.
 */
void RRTPlanner::initialize(double originX, double originY, double resolution, uint32_t width, uint32_t height) {
    originX_ = originX;
    originY_ = originY;
    resolution_ = resolution;
    width_ = width;
    height_ = height;
}

/**
 * @brief Generate a path from start to goal using the RRT algorithm.
 * 
 * @param start Start pose of the path.
 * @param goal Goal pose of the path.
 * @param plan Vector to store the planned path.
 * @return true if a path is found, false otherwise.
 */
bool RRTPlanner::makePlan(const geometry_msgs::msg::PoseStamped &start, const geometry_msgs::msg::PoseStamped &goal, std::vector<geometry_msgs::msg::PoseStamped> &plan) {
    tree_.clear();
    tree_.emplace_back(start.pose.position.x, start.pose.position.y, 0, -1);

    while (tree_.size() < static_cast<size_t>(max_node_num_)) {
        std::pair<double, double> q_rand = SampleFree();
        if (ExtendTree(q_rand, goal)) {
            Node node = tree_.back();
            while (node.parent_id != -1) {
                geometry_msgs::msg::PoseStamped pose;
                pose.pose.position.x = node.x;
                pose.pose.position.y = node.y;
                plan.push_back(pose);
                node = tree_[node.parent_id];
            }
            std::reverse(plan.begin(), plan.end());
            return true;
        }
    }
    return false;
}

/**
 * @brief Check if a specific point in the map is an obstacle.
 * 
 * @param x X coordinate of the point.
 * @param y Y coordinate of the point.
 * @return true if the point is an obstacle, false otherwise.
 */
bool RRTPlanner::isObstacle(double x, double y) {
    int idx = static_cast<int>((y - originY_) / resolution_) * width_ + static_cast<int>((x - originX_) / resolution_);
    return occ_map_.data[idx] > 0;
}

/**
 * @brief Check if there is an obstacle between two points, considering the car's dimensions.
 * 
 * @param x1 X coordinate of the first point.
 * @param y1 Y coordinate of the first point.
 * @param x2 X coordinate of the second point.
 * @param y2 Y coordinate of the second point.
 * @param car_length Length of the car.
 * @param car_width Width of the car.
 * @param yaw Yaw angle of the car.
 * @return true if there is an obstacle between the points, false otherwise.
 */
bool RRTPlanner::isObstacleBetween(double x1, double y1, double x2, double y2, double car_length, double car_width, double yaw) {
    fop::SATCollisionChecker collision_checker;

    // Create the vehicle polygon
    geometry_msgs::msg::Polygon vehicle_poly;
    double half_car_length = car_length / 2.0;
    double half_car_width = car_width / 2.0;

    geometry_msgs::msg::Point32 p1, p2, p3, p4;
    p1.x = half_car_length;
    p1.y = half_car_width;
    p2.x = half_car_length;
    p2.y = -half_car_width;
    p3.x = -half_car_length;
    p3.y = -half_car_width;
    p4.x = -half_car_length;
    p4.y = half_car_width;

    vehicle_poly.points.push_back(p1);
    vehicle_poly.points.push_back(p2);
    vehicle_poly.points.push_back(p3);
    vehicle_poly.points.push_back(p4);
    vehicle_poly.points.push_back(vehicle_poly.points.front()); // Close the polygon

    double dist = std::hypot(x2 - x1, y2 - y1);
    int steps = static_cast<int>(dist / resolution_);
    double dx = (x2 - x1) / steps;
    double dy = (y2 - y1) / steps;

    for (int i = 0; i <= steps; ++i) {
        double x = x1 + i * dx;
        double y = y1 + i * dy;

        // Check collision for the vehicle polygon at each step
        geometry_msgs::msg::Polygon vehicle_poly_transformed;
        for (const auto& point : vehicle_poly.points) {
            geometry_msgs::msg::Point32 p;
            p.x = x + point.x * cos(yaw) - point.y * sin(yaw);
            p.y = y + point.x * sin(yaw) + point.y * cos(yaw);
            vehicle_poly_transformed.points.push_back(p);
        }

        // Check each relevant cell in the occupancy grid
        int cell_x = static_cast<int>((x - originX_) / resolution_);
        int cell_y = static_cast<int>((y - originY_) / resolution_);
        if (cell_x >= 0 && cell_x < static_cast<int>(width_) && cell_y >= 0 && cell_y < static_cast<int>(height_) && occ_map_.data[cell_y * width_ + cell_x] > 0) {
            auto obstacle_poly = createObstaclePolygon(x, y);
            if (collision_checker.check_collision(vehicle_poly_transformed, obstacle_poly)) {
                return true; // Collision detected
            }
        }
    }
    return false;
}

/**
 * @brief Create a polygon representing an obstacle in the occupancy grid.
 * 
 * @param x X coordinate of the center of the obstacle.
 * @param y Y coordinate of the center of the obstacle.
 * @return geometry_msgs::msg::Polygon representing the obstacle.
 */
geometry_msgs::msg::Polygon RRTPlanner::createObstaclePolygon(double x, double y) {
    geometry_msgs::msg::Polygon obstacle_poly;
    double half_res = resolution_ / 2.0;

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

/**
 * @brief Check if the node has reached the goal.
 * 
 * @param node Current node.
 * @param goal Goal pose.
 * @return true if the node is within tolerance of the goal, false otherwise.
 */
bool RRTPlanner::isArrivedGoal(const Node &node, const geometry_msgs::msg::PoseStamped &goal) {
    return std::hypot(node.x - goal.pose.position.x, node.y - goal.pose.position.y) < tolerance_;
}

/**
 * @brief Sample a random point in the free space.
 * 
 * @return std::pair<double, double> Coordinates of the sampled point.
 */
std::pair<double, double> RRTPlanner::SampleFree() {
    double x = originX_ + static_cast<double>(rand()) / RAND_MAX * width_ * resolution_;
    double y = originY_ + static_cast<double>(rand()) / RAND_MAX * height_ * resolution_;
    return {x, y};
}

/**
 * @brief Steer a node towards a random point.
 * 
 * @param nearX X coordinate of the nearest node.
 * @param nearY Y coordinate of the nearest node.
 * @param randX X coordinate of the random point.
 * @param randY Y coordinate of the random point.
 * @return std::pair<double, double> Coordinates of the new node.
 */
std::pair<double, double> RRTPlanner::SteerNode(double nearX, double nearY, double randX, double randY) {
    double dist = std::hypot(randX - nearX, randY - nearY);
    if (dist <= epsilon_max_ && dist >= epsilon_min_) {
        return {randX, randY};
    } else {
        double theta = std::atan2(randY - nearY, randX - nearX);
        return {nearX + epsilon_max_ * std::cos(theta), nearY + epsilon_max_ * std::sin(theta)};
    }
}

/**
 * @brief Find the nearest node in the tree to a given point.
 * 
 * @param x X coordinate of the point.
 * @param y Y coordinate of the point.
 * @return int Index of the nearest node.
 */
int RRTPlanner::NearestNode(double x, double y) {
    int nearest_id = 0;
    double min_dist = std::hypot(x - tree_[0].x, y - tree_[0].y);
    for (size_t i = 1; i < tree_.size(); ++i) {
        double dist = std::hypot(x - tree_[i].x, y - tree_[i].y);
        if (dist < min_dist) {
            min_dist = dist;
            nearest_id = i;
        }
    }
    return nearest_id;
}

/**
 * @brief Add a new node to the tree.
 * 
 * @param parent_id Index of the parent node.
 * @param x X coordinate of the new node.
 * @param y Y coordinate of the new node.
 */
void RRTPlanner::addNewNode(int parent_id, double x, double y) {
    tree_.emplace_back(x, y, tree_.size(), parent_id);
}

/**
 * @brief Extend the tree towards a random point.
 * 
 * @param q_rand Coordinates of the random point.
 * @param goal Goal pose.
 * @return true if the tree is extended, false otherwise.
 */
bool RRTPlanner::ExtendTree(const std::pair<double, double> &q_rand, const geometry_msgs::msg::PoseStamped &goal) {
    int nearest_id = NearestNode(q_rand.first, q_rand.second);
    std::pair<double, double> q_new = SteerNode(tree_[nearest_id].x, tree_[nearest_id].y, q_rand.first, q_rand.second);
    double yaw = std::atan2(q_new.second - tree_[nearest_id].y, q_new.first - tree_[nearest_id].x);
    if (!isObstacleBetween(tree_[nearest_id].x, tree_[nearest_id].y, q_new.first, q_new.second, 4.0, 2.0, yaw)) {
        addNewNode(nearest_id, q_new.first, q_new.second);
        if (isArrivedGoal(tree_.back(), goal)) {
            addNewNode(tree_.size() - 1, goal.pose.position.x, goal.pose.position.y);
            return true;
        }
    }
    return false;
}
