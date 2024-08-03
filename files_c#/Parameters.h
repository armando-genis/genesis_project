#ifndef PARAMETERS_H
#define PARAMETERS_H

class Parameters {
public:
    // Getters for different parameters
    static float GetCellWidth() { return cellWidth; }
    static float GetObstacleCost() { return obstacleCost; }
    static float GetReverseCost() { return reverseCost; }
    static float GetSwitchingDirectionOfMovementCost() { return switchingDirectionOfMovementCost; }
    static float GetTrailerReverseCost() { return trailerReverseCost; }

private:
    // Parameters
    static constexpr float cellWidth = 1.0f; // Width of each cell in the map
    static constexpr float obstacleCost = 1.0f; // Cost associated with obstacles
    static constexpr float reverseCost = 1.0f; // Cost associated with reversing
    static constexpr float switchingDirectionOfMovementCost = 1.0f; // Cost for switching direction
    static constexpr float trailerReverseCost = 1.0f; // Additional cost for trailer reversing
};

#endif // PARAMETERS_H
