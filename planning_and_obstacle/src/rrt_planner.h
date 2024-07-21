#ifndef RRT_PLANNER_H
#define RRT_PLANNER_H

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <vector>

struct Node {
    double x, y;
    int id, parent_id;
    Node(double x, double y, int id, int parent_id) : x(x), y(y), id(id), parent_id(parent_id) {}
};

class RRTPlanner {
public:
    RRTPlanner();
    void setOccupancyGrid(const nav_msgs::msg::OccupancyGrid &occ_map);
    void initialize(double originX, double originY, double resolution, uint32_t width, uint32_t height);
    bool makePlan(const geometry_msgs::msg::PoseStamped &start, const geometry_msgs::msg::PoseStamped &goal, std::vector<geometry_msgs::msg::PoseStamped> &plan);
    
private:
    nav_msgs::msg::OccupancyGrid occ_map_;
    double originX_, originY_, resolution_;
    uint32_t width_, height_;
    std::vector<Node> tree_;
    double epsilon_min_, epsilon_max_, tolerance_;
    int max_node_num_;

    bool isObstacle(double x, double y);
    bool isObstacleBetween(double x1, double y1, double x2, double y2, double car_length, double car_width, double yaw);
    bool isArrivedGoal(const Node &node, const geometry_msgs::msg::PoseStamped &goal);
    std::pair<double, double> SampleFree();
    std::pair<double, double> SteerNode(double nearX, double nearY, double randX, double randY);
    int NearestNode(double x, double y);
    void addNewNode(int parent_id, double x, double y);
    bool ExtendTree(const std::pair<double, double> &q_rand, const geometry_msgs::msg::PoseStamped &goal);

    geometry_msgs::msg::Polygon createObstaclePolygon(double x, double y);
};

#endif // RRT_PLANNER_H
