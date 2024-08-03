#ifndef MAP_H
#define MAP_H

#include <vector>
#include <Eigen/Dense>

struct Cell {
    float heuristics;
    float distanceToTarget;
    struct VoronoiFieldCell {
        float voronoiFieldValue;
    } voronoiFieldCell;
    bool occupied;
};

class Map {
public:
    Map(int width, int height, float resolution, const std::vector<std::vector<Cell>>& cellData)
        : width(width), height(height), resolution(resolution), cellData(cellData) {}

    int GetWidth() const { return width; }
    int GetHeight() const { return height; }
    float GetResolution() const { return resolution; }

    const std::vector<std::vector<Cell>>& GetCellData() const { return cellData; }

    // Converts a world position to a map cell position
    Eigen::Vector2i ConvertWorldToCell(const Eigen::Vector3f& position) const {
        int x = static_cast<int>((position.x() - originX) / resolution);
        int y = static_cast<int>((position.z() - originY) / resolution);
        return Eigen::Vector2i(x, y);
    }

    // Checks if a cell is within the grid bounds
    bool IsCellWithinGrid(const Eigen::Vector2i& cell) const {
        return cell.x() >= 0 && cell.x() < width && cell.y() >= 0 && cell.y() < height;
    }

    // Checks if a cell is occupied
    bool IsCellOccupied(const Eigen::Vector2i& cell) const {
        if (!IsCellWithinGrid(cell)) {
            return true; // Out of bounds cells are considered occupied
        }
        return cellData[cell.x()][cell.y()].occupied;
    }

private:
    int width;
    int height;
    float resolution;
    float originX = 0.0f;
    float originY = 0.0f;
    std::vector<std::vector<Cell>> cellData;
};

#endif // MAP_H
