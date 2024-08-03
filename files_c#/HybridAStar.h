#ifndef HYBRIDASTAR_H
#define HYBRIDASTAR_H

#include <vector>
#include <cmath>
#include <unordered_set>
#include <unordered_map>
#include <limits>
#include <algorithm>
#include "Node.h"
#include "Heap.h"
#include "ReedsShepp.h"
#include "VehicleSimulationModels.h"
#include "ObstaclesDetection.h"
#include "Parameters.h"

class HybridAStar {
public:
    static std::vector<Node> GeneratePath(const Car& startCar, const Car& endCar, std::vector<Node>& allExpandedNodes, const Car* startTrailer = nullptr) {
        // Initialize data structures
        int mapWidth = mapWidth_;
        const auto& cellData = cellData_;

        Heap<Node> openNodes(200000);
        std::vector<std::vector<std::unordered_set<int>>> closedCells(mapWidth, std::vector<std::unordered_set<int>>(mapWidth));
        std::vector<std::vector<std::unordered_map<int, Node>>> lowestCostNodes(mapWidth, std::vector<std::unordered_map<int, Node>>(mapWidth));

        // Initialize trailer-related structures
        std::vector<std::vector<std::unordered_set<int>>> closedCellsTrailer;
        std::vector<std::vector<std::unordered_set<int>>> lowestCostNodesTrailer;
        if (startTrailer) {
            closedCellsTrailer.resize(mapWidth, std::vector<std::unordered_set<int>>(mapWidth));
            lowestCostNodesTrailer.resize(mapWidth, std::vector<std::unordered_set<int>>(mapWidth));
        }

        // Create the first node
        IntVector2 startCellPos = ConvertWorldToCell(startCar.GetRearWheelPos());
        Node node(nullptr, startCar.GetRearWheelPos(), startCar.GetHeadingInRadians(), false);
        if (startTrailer) {
            node.SetTrailerHeadingInRadians(startTrailer->GetHeadingInRadians());
        }
        node.AddCosts(0.0f, cellData[startCellPos.x][startCellPos.y].heuristics);
        openNodes.Add(node);

        // Main loop variables
        Node finalNode;
        bool found = false;
        bool resign = false;
        int iterations = 0;
        IntVector2 goalCellPos = ConvertWorldToCell(endCar.GetRearWheelPos());

        while (!found && !resign) {
            if (iterations > 400000) {
                std::cerr << "Stuck in infinite loop" << std::endl;
                break;
            }
            iterations++;

            if (openNodes.Count() == 0) {
                resign = true;
                std::cerr << "Failed to find a path" << std::endl;
            } else {
                Node nextNode = openNodes.RemoveFirst();
                IntVector2 cellPos = ConvertWorldToCell(nextNode.GetRearWheelPos());
                int roundedHeading = RoundValue(nextNode.GetHeadingInDegrees(), headingResolution);

                // Process closed cells
                auto& closedHeadingsInThisCell = closedCells[cellPos.x][cellPos.y];
                bool haveAlreadyClosedCell = false;
                if (!closedHeadingsInThisCell.count(roundedHeading)) {
                    closedHeadingsInThisCell.insert(roundedHeading);
                } else if (!startTrailer) {
                    haveAlreadyClosedCell = true;
                }

                // Trailer processing
                if (startTrailer) {
                    int roundedHeadingTrailer = RoundValue(nextNode.GetTrailerHeadingInDegrees(), headingResolutionTrailer);
                    auto& closedTrailerHeadingsInThisCell = closedCellsTrailer[cellPos.x][cellPos.y];
                    if (!closedTrailerHeadingsInThisCell.count(roundedHeadingTrailer)) {
                        closedTrailerHeadingsInThisCell.insert(roundedHeadingTrailer);
                    } else {
                        haveAlreadyClosedCell = true;
                    }
                }

                // Skip processing if cell is already closed
                if (haveAlreadyClosedCell) {
                    iterations--;
                    continue;
                }

                allExpandedNodes.push_back(nextNode);

                // Check if the vehicle has reached the target
                float distanceSqrToGoal = (nextNode.GetRearWheelPos() - endCar.GetRearWheelPos()).squaredNorm();
                float headingDifference = std::abs(endCar.GetHeadingInDegrees() - nextNode.GetHeadingInDegrees());

                if ((distanceSqrToGoal < posAccuracy * posAccuracy || (cellPos.x == goalCellPos.x && cellPos.y == goalCellPos.y)) &&
                    headingDifference < headingAccuracy) {
                    found = true;
                    finalNode = nextNode;
                    finalNode.SetRearWheelPos(endCar.GetRearWheelPos());
                } else {
                    auto children = GetChildrenToNode(nextNode, cellData, startCar.GetCarData(), endCar, startTrailer);
                    for (auto& child : children) {
                        IntVector2 childCell = ConvertWorldToCell(child.GetRearWheelPos());
                        int roundedChildHeading = RoundValue(child.GetHeadingInDegrees(), headingResolution);

                        if (closedCells[childCell.x][childCell.y].count(roundedChildHeading) && !startTrailer) {
                            continue;
                        } else if (closedCells[childCell.x][childCell.y].count(roundedChildHeading) && startTrailer) {
                            int roundedTrailerHeading = RoundValue(child.GetTrailerHeadingInDegrees(), headingResolutionTrailer);
                            if (closedCellsTrailer[childCell.x][childCell.y].count(roundedTrailerHeading)) {
                                continue;
                            }
                        }

                        float costSoFar = child.GetGCost();
                        auto& nodesWithLowestCost = lowestCostNodes[childCell.x][childCell.y];
                        if (nodesWithLowestCost.count(roundedChildHeading) && !startTrailer) {
                            if (costSoFar < nodesWithLowestCost[roundedChildHeading].GetGCost()) {
                                auto& existingNode = nodesWithLowestCost[roundedChildHeading];
                                child.StealDataFromThisNode(existingNode);
                                openNodes.UpdateItem(existingNode);
                            }
                            continue;
                        } else if (nodesWithLowestCost.count(roundedChildHeading) && startTrailer) {
                            int roundedTrailerHeading = RoundValue(child.GetTrailerHeadingInDegrees(), headingResolutionTrailer);
                            if (lowestCostNodesTrailer[childCell.x][childCell.y].count(roundedTrailerHeading)) {
                                if (costSoFar < nodesWithLowestCost[roundedChildHeading].GetGCost()) {
                                    auto& existingNode = nodesWithLowestCost[roundedChildHeading];
                                    child.StealDataFromThisNode(existingNode);
                                    openNodes.UpdateItem(existingNode);
                                }
                                continue;
                            }
                        } else {
                            nodesWithLowestCost[roundedChildHeading] = child;
                            if (startTrailer) {
                                int roundedTrailerHeading = RoundValue(child.GetTrailerHeadingInDegrees(), headingResolutionTrailer);
                                lowestCostNodesTrailer[childCell.x][childCell.y].insert(roundedTrailerHeading);
                            }
                        }

                        if (ObstaclesDetection::HasCarInvalidPosition(child.GetRearWheelPos(), child.GetHeading(), startCar.GetCarData(), map_)) {
                            continue;
                        }

                        if (startTrailer) {
                            Eigen::Vector3f trailerAttachmentPoint = startCar.GetCarData().GetTrailerAttachmentPoint(child.GetRearWheelPos(), child.GetHeadingInRadians());
                            Eigen::Vector3f trailerRearWheelPos = startTrailer->GetCarData().GetTrailerRearWheelPos(trailerAttachmentPoint, child.GetTrailerHeadingInRadians());
                            if (ObstaclesDetection::HasCarInvalidPosition(trailerRearWheelPos, child.GetTrailerHeadingInRadians(), startTrailer->GetCarData(), map_)) {
                                continue;
                            }
                            if (ObstaclesDetection::IsTrailerCollidingWithDragVehicle(child.GetRearWheelPos(), child.GetHeadingInRadians(), startCar.GetCarData(),
                                                                                     trailerRearWheelPos, child.GetTrailerHeadingInRadians(), startTrailer->GetCarData())) {
                                continue;
                            }
                        }

                        openNodes.Add(child);
                    }
                }
            }
        }

        return GenerateFinalPath(finalNode);
    }

    static void SetMap(const std::vector<std::vector<int>>& data, float resolution, float originX, float originY, int width, int height) {
        mapData_ = data;
        resolution_ = resolution;
        originX_ = originX;
        originY_ = originY;
        mapWidth_ = width;
        mapHeight_ = height;
        // Convert occupancy grid data to cell data
        cellData_.resize(mapHeight_, std::vector<Cell>(mapWidth_));
        for (int y = 0; y < mapHeight_; ++y) {
            for (int x = 0; x < mapWidth_; ++x) {
                cellData_[y][x].heuristics = data[y][x]; // Simplified, you may need a proper conversion
            }
        }
    }

private:
    static std::vector<std::vector<int>> mapData_;
    static float resolution_;
    static float originX_;
    static float originY_;
    static int mapWidth_;
    static int mapHeight_;
    static std::vector<std::vector<Cell>> cellData_;

    static std::vector<Node> GetChildrenToNode(const Node& currentNode, const std::vector<std::vector<Cell>>& cellData, 
                                               const CarData& carData, const Car& endCar, const Car* startTrailer) {
        std::vector<Node> childNodes;
        float heading = currentNode.GetHeading();

        for (float driveDistance : driveDistances) {
            for (float alpha : steeringAngles) {
                float beta = (driveDistance / carData.GetWheelBase()) * std::tan(alpha);
                Eigen::Vector3f newRearWheelPos = VehicleSimulationModels::CalculateNewPosition(heading, beta, driveDistance, currentNode.GetRearWheelPos());
                float newHeading = VehicleSimulationModels::CalculateNewHeading(heading, beta);

                IntVector2 cellPos = ConvertWorldToCell(newRearWheelPos);
                if (!IsCellWithinGrid(cellPos)) {
                    continue;
                }

                Node childNode(&currentNode, newRearWheelPos, newHeading, driveDistance < 0.0f);
                float heuristics = HeuristicsToReachGoal(cellData, cellPos, childNode, endCar, carData);
                childNode.AddCosts(CostToReachNode(childNode, cellData), heuristics);

                if (startTrailer) {
                    float thetaOld = currentNode.GetTrailerHeadingInRadians();
                    float thetaOldDragVehicle = currentNode.GetHeadingInRadians();
                    float D = driveDistance;
                    float d = startTrailer->GetCarData().GetWheelBase();
                    float newTrailerHeading = VehicleSimulationModels::CalculateNewTrailerHeading(thetaOld, thetaOldDragVehicle, D, d);
                    childNode.SetTrailerHeadingInRadians(newTrailerHeading);
                    if (childNode.IsReversing()) {
                        childNode.SetGCost(childNode.GetGCost() + Parameters::GetTrailerReverseCost());
                    }
                }

                childNodes.push_back(childNode);
            }
        }

        // Expand Reeds-Shepp curve
        IntVector2 goalCell = ConvertWorldToCell(endCar.GetRearWheelPos());
        float distanceToEnd = cellData[goalCell.x][goalCell.y].distanceToTarget;
        float testProbability = std::clamp((maxReedsSheppDist - distanceToEnd) / maxReedsSheppDist, 0.0f, 1.0f) * 0.2f;
        float probability = static_cast<float>(rand()) / RAND_MAX;

        if ((distanceToEnd < maxReedsSheppDist && probability < testProbability) || (distanceToEnd < 40.0f && probability < 0.005f)) {
            auto shortestPath = ReedsShepp::GetShortestPath(currentNode.GetRearWheelPos(), currentNode.GetHeading(), endCar.GetRearWheelPos(), endCar.GetHeadingInRadians(),
                                                            carData.GetTurningRadius(), driveDistance, true);
            if (!shortestPath.empty() && shortestPath.size() > 1) {
                auto carToAdd = shortestPath[1];
                bool isReversing = carToAdd.gear == RSCar::Gear::Back;
                IntVector2 cellPos = ConvertWorldToCell(carToAdd.pos);
                if (IsCellWithinGrid(cellPos)) {
                    Node childNode(&currentNode, carToAdd.pos, carToAdd.headingInRad, isReversing);
                    float heuristics = HeuristicsToReachGoal(cellData, cellPos, childNode, endCar, carData);
                    childNode.AddCosts(CostToReachNode(childNode, cellData), heuristics);
                    childNodes.push_back(childNode);
                }
            }
        }

        return childNodes;
    }

    static float HeuristicsToReachGoal(const std::vector<std::vector<Cell>>& cellData, const IntVector2& cellPos, const Node& node, const Car& endCar, const CarData& carData) {
        float heuristics = cellData[cellPos.x][cellPos.y].heuristics;
        if (cellData[cellPos.x][cellPos.y].distanceToTarget < 20.0f) {
            float RS_distance = ReedsShepp::GetShortestDistance(node.GetRearWheelPos(), node.GetHeading(), endCar.GetRearWheelPos(), endCar.GetHeadingInRadians(), carData.GetTurningRadius());
            if (RS_distance > heuristics) {
                heuristics = RS_distance;
            }
        }
        return heuristics;
    }

    static float CostToReachNode(const Node& node, const std::vector<std::vector<Cell>>& cellData) {
        const Node* previousNode = node.GetPreviousNode();
        IntVector2 cellPos = ConvertWorldToCell(node.GetRearWheelPos());

        float costSoFar = previousNode->GetGCost();
        float distanceCost = (node.GetRearWheelPos() - previousNode->GetRearWheelPos()).norm();
        float voronoiCost = Parameters::GetObstacleCost() * cellData[cellPos.x][cellPos.y].voronoiFieldCell.voronoiFieldValue;
        float reverseCost = node.IsReversing() ? Parameters::GetReverseCost() : 0.0f;
        float switchMotionCost = 0.0f;
        if ((node.IsReversing() && !previousNode->IsReversing()) || (!node.IsReversing() && previousNode->IsReversing())) {
            switchMotionCost = Parameters::GetSwitchingDirectionOfMovementCost();
        }

        return costSoFar + distanceCost * (1.0f + voronoiCost + reverseCost) + switchMotionCost;
    }

    static std::vector<Node> GenerateFinalPath(const Node& finalNode) {
        std::vector<Node> finalPath;
        const Node* currentNode = &finalNode;
        while (currentNode != nullptr) {
            finalPath.push_back(*currentNode);
            currentNode = currentNode->GetPreviousNode();
        }
        std::reverse(finalPath.begin(), finalPath.end());
        if (finalPath.size() > 1) {
            finalPath[0].SetReversing(finalPath[1].IsReversing());
        }
        return finalPath;
    }

    static IntVector2 ConvertWorldToCell(const Eigen::Vector3f& pos) {
        int x = static_cast<int>((pos.x() - originX_) / resolution_);
        int y = static_cast<int>((pos.y() - originY_) / resolution_);
        return IntVector2(x, y);
    }

    static bool IsCellWithinGrid(const IntVector2& cellPos) {
        return cellPos.x >= 0 && cellPos.x < mapWidth_ && cellPos.y >= 0 && cellPos.y < mapHeight_;
    }

    static int RoundValue(float value, float resolution) {
        return static_cast<int>(std::round(value / resolution) * resolution);
    }

    static constexpr float driveDistance = std::sqrt((Parameters::GetCellWidth() * Parameters::GetCellWidth()) * 2.0f) + 0.01f;
    static constexpr float driveDistances[2] = {driveDistance, -driveDistance};
    static constexpr float maxAngle = 40.0f;
    static constexpr float steeringAngles[3] = {-maxAngle * M_PI / 180.0f, 0.0f, maxAngle * M_PI / 180.0f};
    static constexpr float posAccuracy = 1.0f;
    static constexpr float headingAccuracy = 10.0f;
    static constexpr float headingResolution = 15.0f;
    static constexpr float headingResolutionTrailer = 15.0f;
    static constexpr float maxReedsSheppDist = 15.0f;
};

// Initialize static members
std::vector<std::vector<int>> HybridAStar::mapData_;
float HybridAStar::resolution_;
float HybridAStar::originX_;
float HybridAStar::originY_;
int HybridAStar::mapWidth_;
int HybridAStar::mapHeight_;
std::vector<std::vector<Cell>> HybridAStar::cellData_;

#endif // HYBRIDASTAR_H
