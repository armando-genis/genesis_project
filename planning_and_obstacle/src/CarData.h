#ifndef CAR_DATA_H
#define CAR_DATA_H

#include <geometry_msgs/msg/polygon.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <vector>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>
#include <visualization_msgs/msg/marker.hpp>
#include <rclcpp/rclcpp.hpp>  // Add this include for rclcpp::Time

#include "PathOptimizationNS.h"
#include "sat_collision_checker.h"

class CarData
{
public:
    // Vehicle parameters with default values
    double car_width;
    double car_length;
    double rtc;
    double rear_d;
    double front_d;
    geometry_msgs::msg::Polygon vehicle_geometry;
    geometry_msgs::msg::Polygon vehicle_poly_rotated;
    
    fop::SATCollisionChecker collision_checker; // Collision checker

    // Constructor with default parameters
    CarData(double width = 2.0, double length = 4.0, double rtc_value = 1.5);

    bool checkCollisionAtStart(const PathOptimizationNS::State state, const nav_msgs::msg::OccupancyGrid &occ_map_data_, double resolution, double originX, double originY, uint32_t width, uint32_t height);

    geometry_msgs::msg::Polygon createObstaclePolygon(double x, double y, double resolution);

    void createVehicleGeometry();

    // Create and return a visualization marker for the vehicle polygon
    visualization_msgs::msg::Marker createVehiclePolygonMarker(const std::string &hex_color, const rclcpp::Time &timestamp) const;

    // Getters for car data
    double getCarWidth() const { return car_width; }
    double getCarLength() const { return car_length; }
    double getRtc() const { return rtc; }
    double getRearD() const { return rear_d; }
    double getFrontD() const { return front_d; }
    geometry_msgs::msg::Polygon getVehicleGeometry() const { return vehicle_geometry; }
};

#endif // CAR_DATA_H
