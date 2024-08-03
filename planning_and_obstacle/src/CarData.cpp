#include "CarData.h"

CarData::CarData(double width, double length, double rtc_value)
    : car_width(width), car_length(length), rtc(rtc_value), rear_d(length / 2 - rtc_value), front_d(length - rear_d)
{
    createVehicleGeometry();
}

bool CarData::checkCollisionAtStart(const PathOptimizationNS::State state, const nav_msgs::msg::OccupancyGrid &occ_map_data_, double resolution, double originX, double originY, uint32_t width, uint32_t height)
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

    geometry_msgs::msg::Polygon vehicle_poly;

    for (const auto &point : vehicle_geometry.points) {
        geometry_msgs::msg::Point32 p_point32;

        double x = state.x + point.x * cos(state.z) - point.y * sin(state.z);
        double y = state.y + point.x * sin(state.z) + point.y * cos(state.z);

        p_point32.x = x;
        p_point32.y = y;

        vehicle_poly.points.push_back(p_point32);
    }

    vehicle_poly_rotated = vehicle_poly;

    // Check each relevant cell in the occupancy grid
    for (uint32_t i = min_i; i <= max_i; ++i) {
        for (uint32_t j = min_j; j <= max_j; ++j) {
            if (occ_map_data_.data[i * width + j] > 0) {
                double x = originX + j * resolution;
                double y = originY + i * resolution;
                auto obstacle_poly = createObstaclePolygon(x, y, resolution);

                if (collision_checker.check_collision(vehicle_poly, obstacle_poly)) {
                    return true; // Collision detected
                }
            }
        }
    }

    return false;
}

geometry_msgs::msg::Polygon CarData::createObstaclePolygon(double x, double y, double resolution)
{
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

void CarData::createVehicleGeometry()
{
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
}

visualization_msgs::msg::Marker CarData::createVehiclePolygonMarker(const std::string &hex_color, const rclcpp::Time &timestamp) const
{
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
    float a = 1.0f;

    visualization_msgs::msg::Marker polygon_marker;
    polygon_marker.header.frame_id = "map";
    polygon_marker.header.stamp = timestamp;
    polygon_marker.ns = "vehicle_polygon";
    polygon_marker.action = visualization_msgs::msg::Marker::ADD;
    polygon_marker.id = 10;
    polygon_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    polygon_marker.scale.x = 0.05;
    polygon_marker.color.r = r;
    polygon_marker.color.g = g;
    polygon_marker.color.b = b;
    polygon_marker.color.a = a;

    for (const auto &point : vehicle_poly_rotated.points) {
        geometry_msgs::msg::Point p_point;
        p_point.x = point.x;
        p_point.y = point.y;
        p_point.z = 0.0;
        polygon_marker.points.push_back(p_point);
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Number of points in the polygon: %d", vehicle_geometry.points.size());

    return polygon_marker;
}
