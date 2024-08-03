#ifndef VEHICLE_SIMULATION_MODELS_H
#define VEHICLE_SIMULATION_MODELS_H

#include <Eigen/Dense>
#include <cmath>

class VehicleSimulationModels {
public:
    // Calculate the new position of the rear wheel after moving with a given heading and distance
    static Eigen::Vector3f CalculateNewPosition(float heading, float beta, float distance, const Eigen::Vector3f& rearWheelPos) {
        float newX = rearWheelPos.x() + distance * std::cos(heading + beta);
        float newY = rearWheelPos.y() + distance * std::sin(heading + beta);
        return Eigen::Vector3f(newX, newY, 0.0f);
    }

    // Calculate the new heading of the vehicle after moving with a given heading and distance
    static float CalculateNewHeading(float heading, float beta) {
        return heading + beta;
    }

    // Calculate the new heading of the trailer
    static float CalculateNewTrailerHeading(float thetaOld, float thetaOldDragVehicle, float D, float d) {
        return thetaOld + (D / d) * std::sin(thetaOldDragVehicle - thetaOld);
    }
};

#endif // VEHICLE_SIMULATION_MODELS_H
