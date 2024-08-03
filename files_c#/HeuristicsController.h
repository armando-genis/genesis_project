#ifndef HEURISTICSCONTROLLER_H
#define HEURISTICSCONTROLLER_H

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include "PathOptimizationNS.h"

class IntVector2 {
public:
    int x, y;
    IntVector2(int x = 0, int y = 0) : x(x), y(y) {}

    IntVector2 operator+(const IntVector2& other) const {
        return IntVector2(x + other.x, y + other.y);
    }
};

class FlowFieldNode {
public:
    bool isWalkable;
    PathOptimizationNS::State centerPos;
    IntVector2 gridPos;
    float totalCostFlowField;

    // Default constructor
    FlowFieldNode() : isWalkable(false), centerPos(), gridPos(), totalCostFlowField(std::numeric_limits<float>::max()) {}

    FlowFieldNode(bool isWalkable, const PathOptimizationNS::State& centerPos, const IntVector2& gridPos)
        : isWalkable(isWalkable), centerPos(centerPos), gridPos(gridPos), totalCostFlowField(std::numeric_limits<float>::max()) {}
};

class FlowField {
public:
    static void Generate(std::vector<FlowFieldNode*>& startNodes, std::vector<std::vector<FlowFieldNode>>& nodesArray, bool includeCorners) {
        std::queue<FlowFieldNode*> nodeQueue;

        // Initialize the start nodes
        for (auto& startNode : startNodes) {
            startNode->totalCostFlowField = 0;
            nodeQueue.push(startNode);
        }

        // Directions for moving in 4 or 8 directions
        std::vector<IntVector2> directions = {
            {1, 0}, {-1, 0}, {0, 1}, {0, -1}
        };

        if (includeCorners) {
            directions.push_back({1, 1});
            directions.push_back({1, -1});
            directions.push_back({-1, 1});
            directions.push_back({-1, -1});
        }

        while (!nodeQueue.empty()) {
            FlowFieldNode* currentNode = nodeQueue.front();
            nodeQueue.pop();

            for (const auto& direction : directions) {
                IntVector2 neighborPos = currentNode->gridPos + direction;

                // Check if neighbor is within bounds
                if (neighborPos.x >= 0 && static_cast<size_t>(neighborPos.x) < nodesArray.size() &&
                    neighborPos.y >= 0 && static_cast<size_t>(neighborPos.y) < nodesArray[0].size()) {
                    FlowFieldNode& neighborNode = nodesArray[neighborPos.x][neighborPos.y];

                    if (neighborNode.isWalkable) {
                        float newCost = currentNode->totalCostFlowField + Distance(currentNode->centerPos, neighborNode.centerPos);

                        if (newCost < neighborNode.totalCostFlowField) {
                            neighborNode.totalCostFlowField = newCost;
                            nodeQueue.push(&neighborNode);
                        }
                    }
                }
            }
        }
    }

private:
    static float Distance(const PathOptimizationNS::State& a, const PathOptimizationNS::State& b) {
        return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
    }
};

class HeuristicsController {
public:
    HeuristicsController(double resolution, double originX, double originY, int width, int height, const std::vector<int8_t>& mapData)
        : resolution_(resolution), originX_(originX), originY_(originY), width_(width), height_(height), mapData_(mapData) {
        euclideanHeuristics.resize(width_, std::vector<float>(height_, 0.0f));
        flowFieldHeuristics.resize(width_, std::vector<float>(height_, 0.0f));
    }

    void GenerateFinalHeuristics(std::vector<int8_t>& mapData) {
        for (int x = 0; x < width_; ++x) {
            for (int y = 0; y < height_; ++y) {
                float heuristic = std::max(flowFieldHeuristics[x][y], euclideanHeuristics[x][y]);

                if (mapData[x + y * width_] > 0) {
                    heuristic = 10000.0f;
                }
                mapData[x + y * width_] = static_cast<int8_t>(heuristic);
            }
        }
    }

    void EuclideanDistance(const PathOptimizationNS::State& targetCellPos) {
        PathOptimizationNS::State targetPos = targetCellPos;

        for (int x = 0; x < width_; ++x) {
            for (int y = 0; y < height_; ++y) {
                if (mapData_[x + y * width_] <= 0) {
                    PathOptimizationNS::State thisCellPos;
                    thisCellPos.x = x * resolution_ + originX_;
                    thisCellPos.y = y * resolution_ + originY_;
                    float heuristic = Distance(targetPos, thisCellPos);
                    euclideanHeuristics[x][y] = heuristic;
                }
            }
        }
    }

    void DynamicProgramming(const PathOptimizationNS::State& targetPos) {
        std::vector<std::vector<FlowFieldNode>> nodesArray(width_, std::vector<FlowFieldNode>(height_));

        for (int x = 0; x < width_; ++x) {
            for (int y = 0; y < height_; ++y) {
                bool isWalkable = mapData_[x + y * width_] <= 0;
                nodesArray[x][y] = FlowFieldNode(isWalkable, PathOptimizationNS::State{x * resolution_ + originX_, y * resolution_ + originY_, 0}, IntVector2(x, y));
            }
        }

        std::vector<FlowFieldNode*> startNodes;
        startNodes.push_back(&nodesArray[targetPos.x][targetPos.y]);

        FlowField::Generate(startNodes, nodesArray, true);

        for (int x = 0; x < width_; ++x) {
            for (int y = 0; y < height_; ++y) {
                flowFieldHeuristics[x][y] = nodesArray[x][y].totalCostFlowField * 0.92621f;
            }
        }
    }

private:
    double resolution_;
    double originX_;
    double originY_;
    int width_;
    int height_;
    const std::vector<int8_t>& mapData_;
    std::vector<std::vector<float>> euclideanHeuristics;
    std::vector<std::vector<float>> flowFieldHeuristics;

    float Distance(const PathOptimizationNS::State& a, const PathOptimizationNS::State& b) {
        return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
    }
};

#endif // HEURISTICSCONTROLLER_H
