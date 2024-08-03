#ifndef OBSTACLES_DETECTION_H
#define OBSTACLES_DETECTION_H

#include <Eigen/Dense>
#include <vector>
#include "Map.h"
#include "CarData.h"

class ObstaclesDetection {
public:
    // Check if the car is in an invalid position (e.g., colliding with an obstacle)
    static bool HasCarInvalidPosition(const Eigen::Vector3f& position, float heading, const CarData& carData, const Map& map) {
        // Convert the car's position to map cell coordinates
        IntVector2 cellPos = map.ConvertWorldToCell(position);

        // Check if the position is within the map bounds
        if (!map.IsCellWithinGrid(cellPos)) {
            return true;
        }

        // Check for collisions with obstacles using the car's dimensions and heading
        std::vector<Eigen::Vector3f> corners = carData.GetCorners(position, heading);
        for (const auto& corner : corners) {
            IntVector2 cornerCell = map.ConvertWorldToCell(corner);
            if (!map.IsCellWithinGrid(cornerCell) || map.IsCellOccupied(cornerCell)) {
                return true;
            }
        }

        return false;
    }

    // Check if the trailer is colliding with the drag vehicle (main car)
    static bool IsTrailerCollidingWithDragVehicle(const Eigen::Vector3f& carPosition, float carHeading, const CarData& carData,
                                                  const Eigen::Vector3f& trailerPosition, float trailerHeading, const CarData& trailerData) {
        // Get the corners of both the car and the trailer
        std::vector<Eigen::Vector3f> carCorners = carData.GetCorners(carPosition, carHeading);
        std::vector<Eigen::Vector3f> trailerCorners = trailerData.GetCorners(trailerPosition, trailerHeading);

        // Check for collisions between the car and trailer corners
        for (const auto& carCorner : carCorners) {
            for (const auto& trailerCorner : trailerCorners) {
                if ((carCorner - trailerCorner).norm() < collisionThreshold) {
                    return true;
                }
            }
        }

        return false;
    }

private:
    static constexpr float collisionThreshold = 0.1f; // Adjust as necessary
};

#endif // OBSTACLES_DETECTION_H
