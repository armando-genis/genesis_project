#ifndef CAR_DATA_H
#define CAR_DATA_H

#include <Eigen/Dense>
#include <vector>

class CarData {
public:
    CarData(float length, float width, float wheelBase) 
        : length(length), width(width), wheelBase(wheelBase) {}

    float GetLength() const { return length; }
    float GetWidth() const { return width; }
    float GetWheelBase() const { return wheelBase; }

    // Returns the corners of the car given its position and heading
    std::vector<Eigen::Vector3f> GetCorners(const Eigen::Vector3f& position, float heading) const {
        float halfLength = length / 2.0f;
        float halfWidth = width / 2.0f;

        Eigen::Vector3f frontLeft  = position + Eigen::Vector3f(-halfLength, 0, halfWidth).rotated(Eigen::AngleAxisf(heading, Eigen::Vector3f::UnitY()));
        Eigen::Vector3f frontRight = position + Eigen::Vector3f(halfLength, 0, halfWidth).rotated(Eigen::AngleAxisf(heading, Eigen::Vector3f::UnitY()));
        Eigen::Vector3f rearLeft   = position + Eigen::Vector3f(-halfLength, 0, -halfWidth).rotated(Eigen::AngleAxisf(heading, Eigen::Vector3f::UnitY()));
        Eigen::Vector3f rearRight  = position + Eigen::Vector3f(halfLength, 0, -halfWidth).rotated(Eigen::AngleAxisf(heading, Eigen::Vector3f::UnitY()));

        return {frontLeft, frontRight, rearLeft, rearRight};
    }

    // Returns the attachment point of the trailer given the car's position and heading
    Eigen::Vector3f GetTrailerAttachmentPoint(const Eigen::Vector3f& position, float heading) const {
        return position + Eigen::Vector3f(-wheelBase, 0, 0).rotated(Eigen::AngleAxisf(heading, Eigen::Vector3f::UnitY()));
    }

    // Returns the rear wheel position of the trailer given the attachment point and trailer heading
    Eigen::Vector3f GetTrailerRearWheelPos(const Eigen::Vector3f& attachmentPoint, float trailerHeading) const {
        return attachmentPoint + Eigen::Vector3f(-wheelBase, 0, 0).rotated(Eigen::AngleAxisf(trailerHeading, Eigen::Vector3f::UnitY()));
    }

private:
    float length;
    float width;
    float wheelBase;
};

#endif // CAR_DATA_H
