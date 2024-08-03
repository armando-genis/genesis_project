#ifndef REEDSSHEPP_H
#define REEDSSHEPP_H

#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include <iostream>


#include "RSCar.h"
#include "PathSegmentLengths.h"
#include "PathWords.h"
#include "PathLengthMath.h"
#include "PathSettings.h"

class ReedsShepp {
public:
    static std::vector<RSCar> GetShortestPath(const Vector3& startPos, float startHeading, const Vector3& endPos, float endHeading, float turningRadius, float wpDistance, bool generateOneWp) {
        RSCar carStart(startPos, startHeading);
        RSCar carEnd(endPos, endHeading);

        RSCar carEndMod = NormalizeGoalCar(carStart, carEnd, turningRadius);

        std::vector<RSCar> shortestPath = FindShortestPath(carStart, carEnd, carEndMod, wpDistance, turningRadius, generateOneWp);

        return shortestPath;
    }

    static float GetShortestDistance(const Vector3& startPos, float startHeading, const Vector3& endPos, float endHeading, float turningRadius) {
        RSCar carStart(startPos, startHeading);
        RSCar carEnd(endPos, endHeading);

        RSCar carEndMod = NormalizeGoalCar(carStart, carEnd, turningRadius);

        PathSegmentLengths bestPathLengths;
        PathWords bestWord;

        float shortestPathLength = FindShortestPathLength(carEndMod, bestPathLengths, bestWord);

        if (std::isinf(shortestPathLength)) {
            std::cerr << "Can't find a Reeds-Shepp path" << std::endl;
            return shortestPathLength;
        }

        shortestPathLength *= turningRadius;
        return shortestPathLength;
    }

private:
    static float FindShortestPathLength(const RSCar& carEndMod, PathSegmentLengths& bestPathLengths, PathWords& bestWord) {
        int numPathWords = static_cast<int>(PathWords::Count);

        float shortestPathLength = std::numeric_limits<float>::infinity();
        bestWord = PathWords::Count;

        for (int w = 0; w < numPathWords; ++w) {
            PathWords word = static_cast<PathWords>(w);
            PathSegmentLengths pathSegmentLengths;
            float pathLength = PathLengthMath::GetLength(carEndMod, word, pathSegmentLengths);

            if (pathLength < shortestPathLength) {
                shortestPathLength = pathLength;
                bestWord = word;
                bestPathLengths = pathSegmentLengths;
            }
        }

        return shortestPathLength;
    }

    static RSCar NormalizeGoalCar(const RSCar& carStart, const RSCar& carEnd, float turningRadius) {
        Vector3 posDiff = carEnd.pos - carStart.pos;
        posDiff /= turningRadius;

        float headingDiff = PathLengthMath::WrapAngleInRadians(2.0f * M_PI - (carEnd.headingInRad - carStart.headingInRad));

        Vector3 newEndPos = RotateVector(posDiff, -carStart.headingInDegrees + 90.0f);

        return RSCar(newEndPos, headingDiff);
    }

    static std::vector<RSCar> FindShortestPath(const RSCar& carStart, const RSCar& carEnd, const RSCar& carEndMod, float wpDistance, float turningRadius, bool generateOneWp) {
        PathSegmentLengths bestPathLengths;
        PathWords bestWord;

        float shortestPathLength = FindShortestPathLength(carEndMod, bestPathLengths, bestWord);

        if (std::isinf(shortestPathLength)) {
            std::cerr << "Can't find a Reeds-Shepp path" << std::endl;
            return {};
        }

        return AddWaypoints(bestWord, bestPathLengths, carStart, carEnd, wpDistance, turningRadius, generateOneWp);
    }

    static std::vector<RSCar> AddWaypoints(PathWords word, const PathSegmentLengths& pathSegmentLengths, const RSCar& carStart, const RSCar& carEnd, float wpDistance, float turningRadius, bool generateOneWp) {
        auto pathSettings = PathSettings::GetSettings(word, pathSegmentLengths);

        if (pathSettings.empty()) {
            std::cerr << "Can't find settings for a path" << std::endl;
            return {};
        }

        Vector3 pos = carStart.pos;
        float heading = carStart.headingInRad;
        float stepDistance = 0.05f;
        float driveDistance = 0.0f;

        std::vector<RSCar> waypoints;
        waypoints.emplace_back(pos, heading, pathSettings[0].gear, pathSettings[0].steering);

        for (const auto& segmentSettings : pathSettings) {
            int n = static_cast<int>(std::ceil((segmentSettings.length * turningRadius) / stepDistance));
            float stepLength = (segmentSettings.length * turningRadius) / n;

            float steeringWheelPos = (segmentSettings.steering == RSCar::Steering::Left) ? -1.0f : 1.0f;

            if (segmentSettings.gear == RSCar::Gear::Back) {
                steeringWheelPos *= -1.0f;
            }

            for (int j = 0; j < n; ++j) {
                float dx = stepLength * std::sin(heading);
                float dz = stepLength * std::cos(heading);

                if (segmentSettings.gear == RSCar::Gear::Back) {
                    dx = -dx;
                    dz = -dz;
                }

                pos = {pos.x + dx, pos.y, pos.z + dz};

                if (segmentSettings.steering != RSCar::Steering::Straight) {
                    heading += (stepLength / turningRadius) * steeringWheelPos;
                }

                driveDistance += stepLength;

                if (driveDistance > wpDistance) {
                    waypoints.emplace_back(pos, heading, segmentSettings.gear, segmentSettings.steering);
                    driveDistance -= wpDistance;

                    if (generateOneWp) {
                        return waypoints;
                    }
                }
            }

            waypoints.emplace_back(pos, heading, segmentSettings.gear, segmentSettings.steering);
        }

        waypoints.back().pos = carEnd.pos;

        return waypoints;
    }

    static Vector3 RotateVector(const Vector3& vec, float angle) {
        float rad = angle * M_PI / 180.0f;
        float cosA = std::cos(rad);
        float sinA = std::sin(rad);

        return {
            vec.x * cosA - vec.z * sinA,
            vec.y,
            vec.x * sinA + vec.z * cosA
        };
    }
};

#endif // REEDSSHEPP_H
