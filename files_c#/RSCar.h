#ifndef RSCAR_H
#define RSCAR_H

#include <Eigen/Dense>
#include <cmath>

class RSCar {
public:
    enum class Steering { Left, Right, Straight };
    enum class Gear { Forward, Back };

    Steering steering;
    Gear gear;
    Eigen::Vector3f pos;
    float heading;

    RSCar(const Eigen::Vector3f& pos, float headingInRadians)
        : pos(pos), heading(WrapAngle(headingInRadians)), gear(Gear::Forward), steering(Steering::Straight) {}

    RSCar(const Eigen::Vector3f& pos, float headingInRadians, Gear gear, Steering steering)
        : pos(pos), heading(WrapAngle(headingInRadians)), gear(gear), steering(steering) {}

    RSCar(const RSCar& car)
        : pos(car.pos), heading(car.heading), gear(car.gear), steering(car.steering) {}

    RSCar ChangeData(float newXPos, float newZPos, float newHeading) const {
        return RSCar(Eigen::Vector3f(newXPos, pos.y(), newZPos), newHeading);
    }

    float HeadingInDegrees() const {
        return heading * 180.0f / M_PI;
    }

    float HeadingInRad() const {
        return heading;
    }

    float X() const {
        return pos.x();
    }

    float Z() const {
        return pos.z();
    }

private:
    static float WrapAngle(float angle) {
        while (angle < 0) {
            angle += 2 * M_PI;
        }
        while (angle >= 2 * M_PI) {
            angle -= 2 * M_PI;
        }
        return angle;
    }
};

#endif // RSCAR_H
